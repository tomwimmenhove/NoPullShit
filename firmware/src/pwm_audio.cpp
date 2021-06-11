#include <util/delay.h>
#include <avr/interrupt.h>

#include "pwm_audio.h"
#include "waves.h"

static bool beeping = false;
static volatile int8_t wave_pos = 0;
static const uint8_t* wave = nullptr;
static uint8_t wave_size = 0;
static uint8_t volume = 255;

void pwm_audio::init()
{
	TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20); // No prescaler. slock source = clkT2S = clkIO = 16MHz, Fs = 16MHz/256 = 62.5KHz
	OCR2A = 0;
}

void pwm_audio::beep(const uint8_t* wave, uint8_t len)
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
		TIMSK2 |= (1 << OCIE2A); // Enable timer interrupt
		PWM_DDR |= PWM_MASK;
		beeping = true;
	}
}

void pwm_audio::beep_disable()
{
	if (beeping)
	{
		beeping = false;
		while (wave_pos) { } // Wait for zero-crossing
		TIMSK2 = 0; // Disable timer interrupt
		PWM_DDR &= ~PWM_MASK;
		PWM_PORT &= ~PWM_MASK;
	}
}

void pwm_audio::alert()
{
	uint8_t tmp_volume = volume;
	volume = 255;
	beep(wave_alert, sizeof(wave_alert));
	_delay_ms(20);
	beep_disable();
	volume = tmp_volume;
}

bool pwm_audio::is_beeping()
{
	return beeping;
}

void pwm_audio::set_volume(uint8_t volume)
{
	::volume = volume;
}

uint8_t pwm_audio::get_volume()
{
	return volume;
}

ISR(TIMER2_COMPA_vect)
{
	OCR2A = (pgm_read_byte(&wave[wave_pos++]) * volume) >> 8;

	if (wave_pos == wave_size)
	{
		wave_pos = 0;
	}
}
