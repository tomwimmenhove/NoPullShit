#include <avr/wdt.h>
#include <avr/sleep.h>
#include <EEPROM.h>

#include "config.h"
#include "pwm_audio.h"
#include "hx711.h"
#include "waves.h"

static int32_t pull_threshold = MIN_PULL_FORCE;
static bool alert = false;
static int8_t alert_count = 0;
static int16_t config_timer = 0;
static int16_t alert_timer = 0;
static int32_t baseline = 0;
static int16_t update_baseline_timer = 0;

static int32_t value_min = 0;
static int32_t value_max = 0;
static int8_t min_max_counter = 0;
static bool pulling = false;
static bool dying = false;

static pwm_audio audio;
static hx711 scale(&DOUT_PIN, DOUT_MASK, &SCK_PORT, SCK_MASK, &RATE_PORT, RATE_MASK);

void deep_sleep()
{
  // Disable sleep - this enables the sleep mode
  SMCR = (1 << 2); // power down mode
  SMCR |= 1;        // enable sleep

  // BOD disable - this must be called right before the sleep instruction
  cli();
  MCUCR |= (3 << 5);                      // set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); // then set the BODS bit and clear the BODSE bit at the same time
  sei();
  
  __asm__ __volatile__ ( "sleep" "\n\t" :: );

  SMCR &= ~1; // disable sleep
}

void watchdog_start(uint8_t wdp)
{
  cli();
  _WD_CONTROL_REG = (1 << WDCE) | (1 << WDE);
  _WD_CONTROL_REG = (1 << WDIE) | wdp;
  sei();
}

void setup()
{
  // set everything unused to outputs
  DDRB = 0xff &
         ~(1 << 3);  // PB3 is PWM out;
  DDRC = 0xff &
         ~(1 << 0);  // ADC0 is batt monitor
  DDRD = 0xff &
         ~(1 << 0) & // PD0 = RX
         ~(1 << 2);  // PD2 is DOUT

  PORTD |= (1 << 1); // TX
  BATT_PLDWN_PORT |= BATT_PLDWN_MASK; // Batt monitor disable

# ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Reset");
# endif

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

  watchdog_start((1 << WDP2)  | (1 << WDP0) | (1 << WDP1)); // 1 second

  // Make sure there's at least one transition (DOUT falling) to get shit going
  baseline = scale.read();
}

EMPTY_INTERRUPT (ADC_vect);

int16_t getReading (const byte port, bool sleep)
{
  BATT_PLDWN_PORT &= ~BATT_PLDWN_MASK; // Enable batt monitor
  ADCSRA |= (1 << 7); // Enable ADC  

  ADCSRA = (1 << ADEN) | (1 << ADIF);  // enable ADC, turn off any pending interrupt
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);   // prescaler of 128
  ADMUX = (1 << REFS1) | (1 << REFS0) | (port & 0x07);  // 1.1v internal

  if (sleep)
  {
    cli();
    set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
    sleep_enable();
  }
  
  // start the conversion
  ADCSRA |= (1 << ADSC) | (1 << ADIE);

  if (sleep)
  {
    sei();
    sleep_cpu();
    sleep_disable();
  }

  // Wait for conversion to finish
  while (ADCSRA & (1 << ADSC)) { }
  int16_t result = ADC;
  
  ADCSRA &= ~(1 << 7); // Disable ADC
  BATT_PLDWN_PORT |= BATT_PLDWN_MASK; // Disable att monitor

  return result;
}

void batt_monitor(bool sleep)
{
  bool dead = false;
  while (true)
  {
    int16_t batt = getReading(0, sleep);

    dying = batt < BATT_DYING;

    if (batt >= dead ? BATT_WAKE_AT : BATT_COMA_BELOW)
    {
      break;
    }

    // Battery too low. Go to sleep.
    if (!dead)
    {
      Serial.println("Battery too low. Going into coma");
      Serial.flush();
      dead = true;
      watchdog_start((1 << WDP3) | (1 << WDP0)); // 8 seconds
      //watchdog_start((1 << WDP2) | (1 << WDP1)); // 1 second
    }
    scale.sleep();
    delayMicroseconds(60);
    deep_sleep();
    wdt_reset();
  }
  
  if (dead)
  {
    Serial.println("We're back");
    Serial.flush();
    watchdog_start((1 << WDP2) | (1 << WDP1)); // 1 second
  }
}

void update_baseline(int32_t& value)
{
# ifdef DEBUG
  Serial.print("Updating baseline: ");
  Serial.println(value + baseline);
  Serial.flush();
# endif
  baseline = value + baseline;
  value = 0;
}

bool standby_checker(int32_t value, int32_t last_value)
{
  static bool standby = true;
  static uint16_t standby_timer = 0;

  int32_t value_diff;
  if (standby)
  {
    value_diff = abs(value - last_value);
  }
  else
  {
    if (value < value_min) value_min = value;
    if (value > value_max) value_max = value;
    if (min_max_counter++ > 8)
    {
      value_diff = value_max - value_min;
      value_min = value_max = value;
      min_max_counter = 0;
    }
  }  

  if (value_diff < IDLE_DELTA_FORCE_THRESHOLD)
  {
    if (!standby)
    {
      if (standby_timer++ > STANDBY_TIMEOUT)
      {
        standby = true;
      }
    }
  }
  else
  {
    standby_timer = 0;
    
    if (standby)
    {
      standby = false;
    }
  }

  return standby;
}

void loop()
{
  static int32_t last_value;
  static bool was_standby = true;
  static int32_t new_pull_threshold;
  static uint16_t blink_timer = 0;

  // Go to sleep, unless we're currenly beeping. We will wake up when DOUT goes low. This means a conversion is ready.
  if (!audio.is_beeping())
  {
    deep_sleep();
  }

  wdt_reset();

  // If we're in standby, we just got woken up by the watchdog. Wake up the scale and go back to sleep
  if (was_standby && !audio.is_beeping())
  {
    scale.wake_up();
    deep_sleep();
  }

  batt_monitor(was_standby);
  
# ifdef DEBUG
  LED_PORT |= LED_MASK;
# endif
  int32_t value = scale.read() - baseline;
  
# ifdef DEBUG
  LED_PORT &= ~LED_MASK;

  //Serial.print("value: ");
  //Serial.println(value);
  //Serial.flush();
# endif

  if (!was_standby)
  {
    if (config_timer)
    {
      config_timer--;
      
      /* Use the maximum force within the configuration period */
      if (value > new_pull_threshold)
      {
        new_pull_threshold = value;    
      }
  
      if (!config_timer)
      {
        if (new_pull_threshold > MIN_PULL_FORCE)
        {
          pull_threshold = new_pull_threshold;
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
    }
    else
    {
      if (!audio.is_beeping())
      {
        if (value > pull_threshold + HIST_FORCE)
        {
          audio.beep(BEEP_WAVE, sizeof(BEEP_WAVE));
          pulling = true;
        }
      }
      else
      {
        if (value < pull_threshold - HIST_FORCE)
        {
          audio.beep_disable();
          pulling = false;
        }
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

  if (!config_timer && value < REVERSE_FORCE)
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
        new_pull_threshold = 0;
      }
    }
  }
  else
  {
    alert = false;
  }
  
  bool standby = standby_checker(value, last_value);

  // We entered stand by mode
  if (standby != was_standby && standby)
  {
#   ifdef DEBUG
    Serial.println("Going to standby");
    Serial.flush();
#   endif

#   ifndef ALWAYS_80HZ
    scale.set_rate(true); // 80Hz rate (50ms settling time, saves mucho power
#   endif
#   ifdef UPDATE_BASELINE_ON_STANDBY
    update_baseline(value);
#   endif
    audio.beep_disable();

    LED_PORT &= ~LED_MASK;
    blink_timer = 0;
  }

  // We returned from stand by mode
  if (standby != was_standby && !standby)
  {
#   ifdef DEBUG
    Serial.println("Activity! Waking up from standby");
    Serial.flush();
#   endif

#   ifndef ALWAYS_80HZ
    scale.set_rate(false); // 10Hz default rate
#   endif
  }

  if (standby)
  {
    scale.sleep();

    if (update_baseline_timer++ >= UPDATE_BASELINE_AFTER)
    {
      pulling = false;
      update_baseline_timer = 0;
      update_baseline(value);
    }
  }
  else
  {
    update_baseline_timer = 0;

    if (blink_timer < BLINK_TIME)
    {
      LED_PORT |= LED_MASK;      
    }
    else
    {
      LED_PORT &= ~LED_MASK;
    }

    uint16_t blink_interval = dying ? BLINK_INTERVAL_DYING : BLINK_INTERVAL;
    if (blink_timer >= blink_interval)
    {      
      blink_timer = 0;
    }
    else
    {
      blink_timer++;      
    }
  }

  last_value = value;
  was_standby = standby;
}

void pin_dout_interrupt() { }

ISR(WDT_vect)
{
# ifdef DEBUG
  //Serial.println("Watchdog Interrupt");
  //Serial.flush();
# endif
}


