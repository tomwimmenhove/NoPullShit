/*
 * battery.cpp
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "config.h"
#include "battery.h"
#include "watchdog.h"
#ifdef DEBUG
#	include "serial.h"
#endif

bool battery::dying = false;

void battery::check(bool sleep, void(*on_sleep)())
{
	bool dead = false;
	uint8_t wdt_saved;
	while (true)
	{
		BATT_PLDWN_PORT &= ~BATT_PLDWN_MASK; // Enable batt monitor
		int16_t batt = getReading(0, sleep);
		BATT_PLDWN_PORT |= BATT_PLDWN_MASK; // Disable batt monitor

		dying = batt < BATT_DYING;

		if (batt >= dead ? BATT_WAKE_AT : BATT_COMA_BELOW)
		{
			break;
		}

		// Battery too low. Go to sleep.
		if (!dead)
		{
#			ifdef DEBUG
			serial::puts("Battery too low. Going into coma\n");
			serial::flush();
#			endif
			dead = true;
			wdt_saved = watchdog::save();
			watchdog::set((1 << WDP3) | (1 << WDP0)); // 8 seconds
		}
		on_sleep();
		watchdog::deep_sleep();
		wdt_reset();
	}

	if (dead)
	{
#		ifdef DEBUG
		serial::puts("We're back\n");
		serial::flush();
#		endif
		watchdog::set(wdt_saved);
	}
}

int16_t battery::getReading(const uint8_t port, bool sleep)
{
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

	return result;
}

ISR(ADC_vect) { }
