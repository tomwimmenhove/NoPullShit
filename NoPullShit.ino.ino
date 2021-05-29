#include <avr/wdt.h>
#include <EEPROM.h>

#include "config.h"
#include "pwm_audio.h"
#include "hx711.h"
#include "waves.h"

static int32_t pull_threshold = 1000;
static bool alert = false;
static int8_t alert_count = 0;
static int16_t config_timer = 0;
static int16_t alert_timer = 0;
static int32_t baseline = 0;

static pwm_audio audio;
static hx711 scale(&DOUT_PIN, DOUT_MASK, &SCK_PORT, SCK_MASK, &RATE_PORT, RATE_MASK);

void deep_sleep()
{
  // Disable sleep - this enables the sleep mode
  SMCR |= (1 << 2); // power down mode
  SMCR |= 1;        // enable sleep

  // BOD disable - this must be called right before the sleep instruction
  cli();
  MCUCR |= (3 << 5);                      // set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); // then set the BODS bit and clear the BODSE bit at the same time
  sei();
  
  __asm__ __volatile__ ( "sleep" "\n\t" :: );

  SMCR &= ~1; // disable sleep
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
         ~(1 << 0) & // PD0 = RX
         ~(1 << 2);  // PD2 is DOUT

  PORTD |= (1 << 1); // TX
  
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

#ifdef DEBUG
  LED_PORT |= LED_MASK;
#endif
  int32_t v = scale.read() - baseline;
#ifdef DEBUG
  LED_PORT &= ~LED_MASK;
#endif
  
  if (config_timer)
  {
    config_timer--;
    
    /* Use the maximum force within the configuration period */
    if (v > pull_threshold && v > HIST_FORCE)
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
    if (!audio.is_beeping())
    {
      if (v > pull_threshold + HIST_FORCE)
      {
        audio.beep(BEEP_WAVE, sizeof(BEEP_WAVE));
      }
    }
    else
    {
      if (v < pull_threshold - HIST_FORCE)
      {
        audio.beep_disable();        
      }
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


