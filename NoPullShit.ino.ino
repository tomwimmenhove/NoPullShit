#include <avr/wdt.h>

#define DEBUG
//#define CRAZY_DEBUG

#define ALWAYS_80HZ

#define LED_PORT PORTB
#define LED_MASK (1 << PINB5)

#define SCK_PORT PORTB
#define SCK_MASK (1 << PINB2)

#define DOUT_PORT PIND
#define DOUT_MASK (1 << PIND2)

#define RATE_PORT PORTD
#define RATE_MASK (1 << PIND3)

#define PWM_DDR DDRB
#define PWM_MASK (1 << PINB3)

#ifdef ALWAYS_80HZ
#define RATE 80
#else
#define RATE 10
#endif

#ifdef DEBUG
uint16_t standby_threshold = 5 * RATE;
#else
uint16_t standby_threshold = 60 * RATE;
#endif

int32_t sens_threshold = 1000;

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

const PROGMEM uint8_t wave_alert[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, };

const uint8_t* wave;
uint8_t wave_size;

void setup_timer()
{
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20); // No prescaler. slock source = clkT2S = clkIO = 16MHz, Fs = 16MHz/256 = 62.5KHz
  OCR2A = 0;
}

int32_t bang_24()
{
  int32_t x = 0;

  for (uint8_t i = 0; i < 24; ++i)
  {
    x <<= 1;

    SCK_PORT |= SCK_MASK;
    SCK_PORT &= ~SCK_MASK;

    if (DOUT_PORT & DOUT_MASK)
    {
      x |= 1;
    }
  }

  return (x << 8) >> 8; // Return sign-extended
}

void deep_sleep()
{
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  __asm__  __volatile__("sleep");//in line assembler to go to sleep
}

int32_t read()
{
  LED_PORT |= LED_MASK;
  // Make sure SCK is low
  SCK_PORT &= ~SCK_MASK;
  
  // Wait until MISO goes low
#ifdef CRAZY_DEBUG
  int32_t n = 0; // ~1884589/sec vs ~162972/sec = ~10Hz.
#endif
  while (DOUT_PORT & DOUT_MASK)
  {
#ifdef CRAZY_DEBUG
    n++;
#endif
    // wait...
  }

#ifdef CRAZY_DEBUG
  Serial.print("n: "); Serial.println(n);
#endif

  //LED_PORT |= LED_MASK;
  int32_t x = bang_24();
  //LED_PORT &= ~LED_MASK;

  /* I don't know what the fuck this is. Without it, sometimes the chip returns all ones.
     I found this on https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide/all in the library
  */
  for (unsigned int i = 0; i < 128; i++)
  {
    SCK_PORT |= SCK_MASK;
    SCK_PORT &= ~SCK_MASK;
  }

  LED_PORT &= ~LED_MASK;

  return x;
}

void watchdog_start()
{
  cli();
  _WD_CONTROL_REG = (1 << WDCE) | (1 << WDE);
  _WD_CONTROL_REG = (1 << WDIE) |
                    (1 << WDP2) | (1 << WDP1); // 1 second
  sei();
}

int32_t baseline = 0;

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

  SCK_PORT &= ~SCK_MASK;  // Set SCK low to initiate conversion
  RATE_PORT |= RATE_MASK; // 80Hz

  // Interrupt on DOUT going low. Causes wake-up
  attachInterrupt(0, pin_dout_interrupt, FALLING);

  // Disable ADC
  ADCSRA &= ~(1 << 7);

  // ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  setup_timer();

  watchdog_start();

  // Make sure there's at least one transition (DOUT falling) to get shit going
  baseline = read();
}

bool beeping = false;
int8_t wave_pos = 0;

void beep_enable()
{
  if (!beeping)
  {
#ifdef DEBUG
    wave = wave_sine;
    wave_size = sizeof(wave_sine);
#else
    wave = wave_dog;
    wave_size = sizeof(wave_dog);
#endif
    wave_pos = 0;
    TIMSK2 |= (1 << OCIE2A); // Enable interrupt
    PWM_DDR |= PWM_MASK;
    beeping = true;
  }
}

void beep_disable()
{
  if (beeping)
  {
    //TIMSK2 = 0;//~(1 << OCIE2A); // Disable interrupt
    //PWM_DDR &= ~PWM_MASK;
    beeping = false;
  }
}

bool standby = true;
void standby_checker(int32_t value_diff)
{
  static uint16_t standby_timer = 0;

  if (!beeping && value_diff < sens_threshold)
  {
    if (!standby)
    {
      if (standby_timer++ > standby_threshold)
      {
#ifdef DEBUG
        Serial.println("Going to standby");
#endif

#ifndef ALWAYS_80HZ
        RATE_PORT |= RATE_MASK; // 80Hz rate (50ms settling time, saves mucho power
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
      RATE_PORT &= ~RATE_MASK; // 10Hz default rate
#endif

      standby = false;
    }
  }

  if (standby)
  {
      // Put the scale to sleep
      SCK_PORT |= SCK_MASK;
      delayMicroseconds(60);
  }
}

void loop()
{
  static int32_t last_value;
  // We will wake up when DOUT goes low. This means a conversion is ready.
  if (!beeping)
  {
    // Can't fall asleep during beeping!
    deep_sleep();
  }

  wdt_reset();

  // If we're in standby, we just got woken up by the watchdog. Wake up the scale and go back to sleep
  if (standby && !beeping)
  {
    SCK_PORT &= ~SCK_MASK;    
    deep_sleep();
  }

  int32_t v = read() - baseline;
  if (v > 1000)
  {
    beep_enable();
  }
  else
  {
    beep_disable();
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

ISR(TIMER2_COMPA_vect)
{
  OCR2A = pgm_read_byte(&wave[wave_pos++]);

  if (wave_pos == wave_size)
  {
    // Checking for this here will remove 'clicks'
    if (!beeping)
    {
      TIMSK2 = 0;//~(1 << OCIE2A); // Disable interrupt
      PWM_DDR &= ~PWM_MASK;
    }
    wave_pos = 0;
  }
}


