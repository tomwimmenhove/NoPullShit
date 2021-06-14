/*
 * audio.cpp
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#include <util/delay.h>
#include <avr/interrupt.h>

#include "audio.h"
#include "waves.h"

static bool enabled = false;
static volatile int8_t wave_pos = 0;
static const uint8_t* wave = nullptr;
static uint8_t wave_size = 0;
static uint8_t volume = 255;

void audio::init()
{
	TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20); // No prescaler. Clock source = clkT2S = clkIO = 16MHz, Fs = 16MHz/256 = 62.5KHz
	OCR2A = 0;
}

void audio::play(const uint8_t* wave, uint8_t len)
{
	::wave = wave;
	wave_size = len;
	enable();
}

void audio::enable()
{
	if (!enabled)
	{
		wave_pos = 0;
		TIMSK2 |= (1 << OCIE2A); // Enable timer interrupt
		PWM_DDR |= PWM_MASK;
		enabled = true;
	}
}

void audio::disable()
{
	if (enabled)
	{
		enabled = false;
		while (wave_pos) { } // Wait for zero-crossing
		TIMSK2 = 0; // Disable timer interrupt
		PWM_DDR &= ~PWM_MASK;
		//PWM_PORT &= ~PWM_MASK;
		OCR2A = 0;
	}
}

void audio::alert(int ms)
{
	uint8_t tmp_volume = volume;
	volume = 255;
	play(wave_alert, sizeof(wave_alert));
	while (ms--)
	{
		_delay_ms(1);
	}
	disable();
	volume = tmp_volume;
}

bool audio::is_enabled()
{
	return enabled;
}

void audio::set_volume(uint8_t volume)
{
	::volume = volume;
}

uint8_t audio::get_volume()
{
	return volume;
}

ISR(TIMER2_COMPA_vect)
{
	// This volume control does terrible things to the DC offset. I don't think I care.
	OCR2A = (pgm_read_byte(&wave[wave_pos++]) * volume) >> 8;

	if (wave_pos == wave_size)
	{
		wave_pos = 0;
	}
}
