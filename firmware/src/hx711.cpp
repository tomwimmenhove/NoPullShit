#include <avr/wdt.h>
#include <util/delay.h>
#include "hx711.h"

int32_t hx711::read()
{
	// Make sure SCK is low
	*clk_port &= ~clk_mask;

	// Wait until MISO goes low
	while (*dout_pin & dout_mask) { }

	int32_t x = bang24();

	/* I don't know what the fuck this is. Without it, sometimes the chip returns all ones.
	   I found this on https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide/all in the library
	   */
	for (unsigned int i = 0; i < 128; i++)
	{
		*clk_port |= clk_mask;
		*clk_port &= ~clk_mask;
	}

	return x;
}

void hx711::sleep()
{
	*clk_port |= clk_mask;
	_delay_us(60);
}

void hx711::wake_up()
{
	*clk_port &= ~clk_mask;  
}

void hx711::set_rate(bool rate)
{
	if (rate)
	{
		*rate_port |= rate_mask;
	}
	else
	{
		*rate_port &= ~rate_mask;    
	}
}

int32_t hx711::bang24()
{
	int32_t x = 0;

	for (uint8_t i = 0; i < 24; ++i)
	{
		x <<= 1;

		*clk_port |= clk_mask;
		*clk_port &= ~clk_mask;

		if (*dout_pin & dout_mask)
		{
			x |= 1;
		}
	}

	return (x << 8) >> 8; // Return sign-extended
}
