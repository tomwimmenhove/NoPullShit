#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "sleepy_adc.h"

int16_t getReading (const uint8_t port, bool sleep)
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

