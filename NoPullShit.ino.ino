#include <avr/wdt.h>
#include <EEPROM.h>

#include "config.h"
#include "pwm_audio.h"
#include "hx711.h"

static int32_t pull_threshold = 1000;
static pwm_audio audio;
static hx711 scale(&DOUT_PIN, DOUT_MASK, &SCK_PORT, SCK_MASK, &RATE_PORT, RATE_MASK);
static bool alert = false;
static int8_t alert_count = 0;
static int16_t config_timer = 0;
static int16_t alert_timer = 0;
static int32_t baseline = 0;

const PROGMEM uint8_t wave_sine[] = { 0x7f, 0x88, 0x92, 0x9b, 0xa4, 0xad, 0xb6, 0xbf, 0xc7, 0xcf, 0xd6, 0xdc, 0xe3, 0xe8, 0xed, 0xf2, 0xf5, 0xf8, 0xfb, 0xfc, 0xfd, 0xfe, 0xfd, 0xfc, 0xfa, 0xf7, 0xf4, 0xf0, 0xeb, 0xe6, 0xe0, 0xd9, 0xd2, 0xcb, 0xc3, 0xbb, 0xb2, 0xa9, 0xa0, 0x96, 0x8d, 0x83, 0x7a, 0x70, 0x67, 0x5d, 0x54, 0x4b, 0x42, 0x3a, 0x32, 0x2b, 0x24, 0x1d, 0x17, 0x12, 0x0d, 0x09, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x02, 0x05, 0x08, 0x0b, 0x10, 0x15, 0x1a, 0x21, 0x27, 0x2e, 0x36, 0x3e, 0x47, 0x50, 0x59, 0x62, 0x6b, 0x75, };

/* 
 * Dog wave
 * f = 932Hz        
 * f * 1 @ -52.3dB
 * f * 2 @ -35.6dB
 * f * 3 @ -17.5dB
 * f * 4 @  -5.7dB
 * f * 5 @  -9.8dB
 * f * 6 @ -33.4dB
 */
const PROGMEM uint8_t wave_dog[] = { 0x7f, 0xb1, 0xdb, 0xf6, 0xfd, 0xf1, 0xd3, 0xaa, 0x7d, 0x53, 0x34, 0x23, 0x22, 0x30, 0x48, 0x66, 0x84, 0x9d, 0xae, 0xb5, 0xb2, 0xa7, 0x98, 0x88, 0x7a, 0x6f, 0x69, 0x67, 0x69, 0x6d, 0x72, 0x76, 0x7a, 0x7d, 0x80, 0x83, 0x87, 0x8b, 0x90, 0x94, 0x96, 0x94, 0x8e, 0x83, 0x75, 0x65, 0x56, 0x4b, 0x48, 0x4f, 0x60, 0x79, 0x97, 0xb5, 0xcd, 0xdb, 0xda, 0xc9, 0xaa, 0x80, 0x53, 0x2a, 0x0c, 0x00, 0x07, 0x22, 0x4c, };

void deep_sleep()
{
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  __asm__  __volatile__("sleep");//in line assembler to go to sleep
}

void watchdog_start()
{
  cli();
  _WD_CONTROL_REG = (1 << WDCE) | (1 << WDE);
  _WD_CONTROL_REG = (1 << WDIE) |
                    (1 << WDP2) | (1 << WDP1); // 1 second
  sei();
}

void setup()
{
  // set everything unused to outputs
  DDRB = 0xff &
         ~(1 << 3);  // PB3 is PWM out;
  DDRC = 0xff;
  DDRD = 0xff &
#ifdef DEBUG
         ~(1 << 1) & // PD1 = TX
#endif
         ~(1 << 2);  // PD2 is DOUT

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Reset");
#endif

  // Read the configured force from the EEPROM
  int32_t f;
  EEPROM.get(EE_FORCE_ADDRESS, f);
  if (f != -1)
  {
      pull_threshold = f;
  }

  SCK_PORT &= ~SCK_MASK;  // Set SCK low to initiate conversion
  scale.set_rate(true); // 80Hz

  // Interrupt on DOUT going low. Causes wake-up
  attachInterrupt(0, pin_dout_interrupt, FALLING);

  // Disable ADC
  ADCSRA &= ~(1 << 7);

  // ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  audio.init();

  watchdog_start();

  // Make sure there's at least one transition (DOUT falling) to get shit going
  baseline = scale.read();
}

bool standby = true;
void standby_checker(int32_t value_diff)
{
  static uint16_t standby_timer = 0;

  if (!audio.is_beeping() && value_diff < IDLE_DELTA_FORCE_THRESHOLD)
  {
    if (!standby)
    {
      if (standby_timer++ > STANDBY_TIMEOUT)
      {
#ifdef DEBUG
        Serial.println("Going to standby");
#endif

#ifndef ALWAYS_80HZ
        scale.set_rate(true); // 80Hz rate (50ms settling time, saves mucho power
#endif
        
        standby = true;
      }
    }
  }
  else
  {
    standby_timer = 0;
    
    if (standby)
    {
#ifdef DEBUG
      Serial.println("Activity! Waking up from standby");
#endif

#ifndef ALWAYS_80HZ
      scale.set_rate(false); // 10Hz default rate
#endif

      standby = false;
    }
  }

  if (standby)
  {
    scale.sleep();
  }
}

void loop()
{
  static int32_t last_value;
  // Go to sleep, unless we're currenly beeping. We will wake up when DOUT goes low. This means a conversion is ready.
  if (!audio.is_beeping())
  {
    deep_sleep();
  }

  wdt_reset();

  // If we're in standby, we just got woken up by the watchdog. Wake up the scale and go back to sleep
  if (standby && !audio.is_beeping())
  {
    scale.wake_up();
    deep_sleep();
  }

  LED_PORT |= LED_MASK;
  int32_t v = scale.read() - baseline;
  LED_PORT &= ~LED_MASK;
  
  if (config_timer)
  {
    config_timer--;
    
    /* Use the maximum force within the configuration period */
    if (v > pull_threshold)
    {
      pull_threshold = v;    
    }

    if (!config_timer)
    {
      // Let the user know the time is up
      audio.alert();
      delay(100);
      audio.alert();
      delay(100);
      audio.alert();

      // Save the configured value in EEPROM
      EEPROM.put(EE_FORCE_ADDRESS, pull_threshold);
    }
  }
  else
  {
    if (v > pull_threshold)
    {
#ifdef DEBUG
      audio.beep(wave_sine, sizeof(wave_sine));
#else
      audio.beep(wave_dog, sizeof(wave_dog));
#endif
    }
    else
    {
      audio.beep_disable();
    }
  }

  if (alert_timer)
  {
    alert_timer--;
    if (!alert_timer)
    {
      alert_count = 0;
    }
  }

  if (!config_timer && v < REVERSE_FORCE)
  {
    alert_timer = ALERT_TIMEOUT;
    if (!alert)
    {
      alert = true;
      
      if (++alert_count <= 2)
      {
        // Single beep to let the user know the push was registered
        audio.alert();
      }
      else
      {
        alert_count = 0;

        audio.alert();
        delay(100);
        audio.alert();
        delay(100);
        audio.alert();

        config_timer = CONFIG_TIME;
        pull_threshold = 0;
      }
    }
  }
  else
  {
    alert = false;
  }

  standby_checker(abs(last_value - v));

#ifdef DEBUG
  Serial.print("value: ");
  Serial.println(v);
  Serial.flush();
#endif

  last_value = v;
}

void pin_dout_interrupt() { }

ISR(WDT_vect)
{
#ifdef DEBUG
  Serial.println("Watchdog Interrupt");
  Serial.flush();
#endif
}


