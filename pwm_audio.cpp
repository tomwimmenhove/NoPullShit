#include "pwm_audio.h"

static bool beeping = false;
static int8_t wave_pos = 0;
static uint8_t* wave = nullptr;
static uint8_t wave_size = 0;

void pwm_audio::init()
{
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20); // No prescaler. slock source = clkT2S = clkIO = 16MHz, Fs = 16MHz/256 = 62.5KHz
  OCR2A = 0;
}

void pwm_audio::beep(uint8_t* wave, uint8_t len)
{
  ::wave = wave;
  wave_size = len;
  beep_enable();
}
  
void pwm_audio::beep_enable()
{
  if (!beeping)
  {
    wave_pos = 0;
    TIMSK2 |= (1 << OCIE2A); // Enable interrupt
    PWM_DDR |= PWM_MASK;
    beeping = true;
  }
}

void pwm_audio::beep_disable()
{
  beeping = false;
}

void pwm_audio::alert()
{
  beep(wave_alert, sizeof(wave_alert));
  delay(20);
  beep_disable();
}

bool pwm_audio::is_beeping()
{
  return beeping;
}

const PROGMEM uint8_t pwm_audio::wave_alert[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, };

ISR(TIMER2_COMPA_vect)
{
  OCR2A = pgm_read_byte(&wave[wave_pos++]);

  if (wave_pos == wave_size)
  {
    // Checking for this here will remove 'clicks'
    // XXX: Should. Doesn't.
    if (!beeping)
    {
      TIMSK2 = 0;//~(1 << OCIE2A); // Disable interrupt
      PWM_DDR &= ~PWM_MASK;
    }
    wave_pos = 0;
  }
}
