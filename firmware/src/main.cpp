/*
 * main.cpp
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include "config.h"
#include "audio.h"
#include "hx711.h"
#include "waves.h"
#include "standby.h"
#include "battery.h"
#include "watchdog.h"
#ifdef DEBUG
#	include "serial.h"
#endif

static int32_t baseline = 0;

static hx711 scale(&DOUT_PIN, DOUT_MASK, &SCK_PORT, SCK_MASK, &RATE_PORT, RATE_MASK);
static standby stdby;

void update_baseline(int32_t& value)
{
#	ifdef DEBUG
	serial::puts("Updating baseline: ");
	serial::puti(value + baseline);
	serial::putc('\n');
	serial::flush();
#	endif
	baseline = value + baseline;
	value = 0;
}

uint8_t calculate_volume(int32_t value, int32_t threshold)
{
	int32_t volume = MIN_VOLUME + (255 - MIN_VOLUME) * (value - threshold) / (threshold * MAX_OVERPULL / 256);
	if (volume < MIN_VOLUME)
	{
		return MIN_VOLUME;
	}
	else if (volume > 255)
	{
		return 255;
	}

	return volume;
}

enum e_main_state
{
	standby,
	normal,
	wait_for_depress,
	wait_for_press,
	config
};

void led_off()
{
	LEDA_PORT &= ~LEDA_MASK;
	LEDB_PORT &= ~LEDB_MASK;
}

void led_on(bool red)
{
	if (red)
	{
		LEDA_PORT |= LEDA_MASK;
		LEDB_PORT &= ~LEDB_MASK;
	}
	else
	{
		LEDA_PORT &= ~LEDA_MASK;
		LEDB_PORT |= LEDB_MASK;
	}
}

int main()
{
	e_main_state mainstate = e_main_state::standby;
	int32_t last_value = 0;
	int32_t new_pull_threshold;
	uint16_t blink_timer = 0;
	int32_t pull_threshold = MIN_PULL_FORCE;
	int8_t alert_count = 0;
	int16_t config_timer = 0;
	int16_t alert_timer = 0;
	int16_t update_baseline_timer = 0;

	// set everything unused to outputs
	DDRB = 0xff &
			~(1 << 3);  // PB3 is PWM out;
	DDRC = 0xff &
			~(1 << 0) & // ADC0 is batt monitor
			~(1 << 2) & // PC2 = CHRG
			~(1 << 3);  // PC3 = STDBY
	DDRD = 0xff &
			~(1 << 0) & // PD0 = RX
			~(1 << 3);  // PD3 is DOUT

	//PORTD |= (1 << 1); // TX
	PORTD &= ~(1 << 1); // TX
	BATT_PLDWN_PORT |= BATT_PLDWN_MASK; // Batt monitor disable

#	ifdef DEBUG
	serial::init();
	serial::puts("Reset\n");
	serial::flush();
#	endif

	// Read the configured force from the EEPROM
	int32_t f = eeprom_read_dword((const uint32_t*) EE_FORCE_ADDRESS);
	if (f != -1)
	{
		pull_threshold = f;
	}

	SCK_PORT &= ~SCK_MASK;  // Set SCK low to initiate conversion
	scale.set_rate(true); // 80Hz

	EICRA |= (1 << ISC11);	// INT1 on falling edge of DOUT
	EIMSK |= (1 << INT1);	// Turns on INT1

	// Disable ADC
	ADCSRA &= ~(1 << 7);

	audio::init();

	watchdog::set((1 << WDP2)  | (1 << WDP0) | (1 << WDP1)); // 1 second

	// Make sure there's at least one transition (DOUT falling) to get shit going
	baseline = scale.read();

	// Main loop
	for(;;)
	{
		// Go to sleep, unless we're currently beeping. We will wake up when DOUT goes low. This means a conversion is ready.
		if (!audio::is_enabled())
		{
			watchdog::deep_sleep();
		}

		wdt_reset();

		battery::check(mainstate == e_main_state::standby, [] () { scale.sleep(); } );

		if (mainstate == e_main_state::standby)
		{
			if (!audio::is_enabled())
			{
				scale.wake_up();
				watchdog::deep_sleep();
			}
		}

		int32_t value = scale.read() - baseline;

		// Main state machine
		switch(mainstate)
		{
		case e_main_state::standby:
			// Go away; I'm sleeping
			break;

		case e_main_state::normal:
			// Check if the dog started or stopped pulling
			if (!audio::is_enabled())
			{
				if (value > pull_threshold + HIST_FORCE)
				{
					audio::set_volume(calculate_volume(value, pull_threshold));
					audio::play(BEEP_WAVE, sizeof(BEEP_WAVE));
				}
			}
			else
			{
				if (value < pull_threshold - HIST_FORCE)
				{
					audio::disable();
				}
				else
				{
					audio::set_volume(calculate_volume(value, pull_threshold));
				}
			}

			// Check if the user pressed the load cell
			if (value < REVERSE_FORCE - HIST_FORCE)
			{
				audio::alert(20);
				mainstate = e_main_state::wait_for_depress;
				alert_count = 0;
			}

			break;

		case e_main_state::wait_for_depress:
			// Wait until the user releases the load cell
			if (value > REVERSE_FORCE + HIST_FORCE)
			{
				mainstate = e_main_state::wait_for_press;
				alert_timer = ALERT_TIMEOUT;
			}

			break;

		case e_main_state::wait_for_press:
			// Wait for the user to press the load cell again
			if (value < REVERSE_FORCE - HIST_FORCE)
			{
				alert_count++;

				// After being pressed 3 times quickly
				if (alert_count == 2)
				{
					// 3 quick beeps
					audio::alert(20);
					_delay_ms(100);
					audio::alert(20);
					_delay_ms(100);
					audio::alert(20);

					// And go to config mode
					config_timer = 0;
					new_pull_threshold = MIN_PULL_FORCE;

					mainstate = e_main_state::config;

					break;
				}

				audio::alert(20);
				mainstate = e_main_state::wait_for_depress;
			}
			else
			{
				alert_timer--;
				if (alert_timer == 0)
				{
					mainstate = e_main_state::normal;
				}
			}
			break;

		case e_main_state::config:
			/* Use the maximum force within the configuration period */
			if (value > new_pull_threshold)
			{
				new_pull_threshold = value;
			}

			config_timer++;

#ifdef		CONFIG_BEEP_TIME
			if (config_timer % CONFIG_BEEP_TIME == 0)
			{
				audio::alert(5);
			}
#endif

			// Until we're out of time...
			if (config_timer >= CONFIG_TIME)
			{
#ifndef			CONFIG_BEEP_TIME
				audio::alert(200);
				_delay_ms(100);
				audio::alert(200);
				_delay_ms(100);
#endif

				mainstate = e_main_state::normal;

				break;
			}

			// ...or the user stopped it by pressing again
			if (value < REVERSE_FORCE - HIST_FORCE)
			{
				if (new_pull_threshold > MIN_PULL_FORCE)
				{
					pull_threshold = new_pull_threshold;

					// Let the user know the new the new settings is saved
					audio::alert(20);
					_delay_ms(100);
					audio::alert(20);
					_delay_ms(100);
					audio::alert(20);

					// Save the configured value in EEPROM
					eeprom_write_dword((uint32_t*) EE_FORCE_ADDRESS, pull_threshold);
				}
				else
				{
					audio::alert(200);
					_delay_ms(100);
					audio::alert(200);
					_delay_ms(100);
				}

				mainstate = e_main_state::normal;
			}

			break;
		}

		bool standby = stdby.check(value, last_value);

		// We entered stand by mode
		if (mainstate != e_main_state::standby && standby)
		{
#			ifdef DEBUG
			serial::puts("Going to standby\n");
			serial::flush();
#			endif

#			ifndef ALWAYS_80HZ
			scale.set_rate(true); // 80Hz rate (50ms settling time, saves mucho power)
#			endif
#			ifdef UPDATE_BASELINE_ON_STANDBY
			update_baseline(value);
#			endif
			audio::disable();

			led_off();
			blink_timer = 0;

			mainstate = e_main_state::standby;
		}

		// We returned from stand by mode
		if (mainstate == e_main_state::standby && !standby)
		{
#			ifdef DEBUG
			serial::puts("Activity! Waking up from standby\n");
			serial::flush();
#			endif

#			ifndef ALWAYS_80HZ
			scale.set_rate(false); // 10Hz default rate
#			endif

			mainstate = e_main_state::normal;
		}

		if (battery::is_standby())
		{
			led_on(false);
		}
		else if (battery::is_charging())
		{
			led_on(true);
		}
		else
		{
			led_off();
		}

		if (mainstate == e_main_state::standby)
		{
			scale.sleep();

			if (update_baseline_timer++ >= UPDATE_BASELINE_AFTER)
			{
				update_baseline_timer = 0;
				update_baseline(value);
			}
		}
		else
		{
			update_baseline_timer = 0;

			if (!battery::is_standby() && !battery::is_charging())
			{
				if (blink_timer < BLINK_TIME)
				{
					led_on(battery::is_dying());
				}
				else
				{
					led_off();
				}
			}

			if (blink_timer >= BLINK_INTERVAL)
			{      
				blink_timer = 0;
			}
			else
			{
				blink_timer++;      
			}
		}

		last_value = value;
	}
	return 0;
}

/* DOUT going low vector */
ISR (INT1_vect) { }
