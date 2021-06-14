/*
 * watchdog.cpp
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "watchdog.h"

void watchdog::set(uint8_t wdp)
{
	cli();
	_WD_CONTROL_REG = (1 << WDCE) | (1 << WDE);
	_WD_CONTROL_REG = (1 << WDIE) | wdp;
	sei();
}

uint8_t watchdog::save()
{
	return _WD_CONTROL_REG & ((1 << WDP0) | (1 << WDP1) | (1 << WDP2) | (1 << WDP3));
}

void watchdog::deep_sleep()
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

ISR(WDT_vect)
{
#	ifdef DEBUG
	//serial::puts("Watchdog Interrupt\n");
	//serial::flush();
#	endif
}
