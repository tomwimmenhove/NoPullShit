#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include "config.h"
#include "pwm_audio.h"
#include "hx711.h"
#include "waves.h"
#include "standby.h"
#include "sleepy_adc.h"
#include "serial.h"

static int32_t baseline = 0;
static bool dying = false;

static pwm_audio audio;
static hx711 scale(&DOUT_PIN, DOUT_MASK, &SCK_PORT, SCK_MASK, &RATE_PORT, RATE_MASK);
static standby stdby;
static serial ser;

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

void batt_monitor(bool sleep)
{
	bool dead = false;
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
			ser.puts("Battery too low. Going into coma\n");
			ser.flush();
#			endif
			dead = true;
			watchdog_start((1 << WDP3) | (1 << WDP0)); // 8 seconds
			//watchdog_start((1 << WDP2) | (1 << WDP1)); // 1 second
		}
		scale.sleep();
		_delay_us(60);
		deep_sleep();
		wdt_reset();
	}

	if (dead)
	{
#		ifdef DEBUG
		ser.puts("We're back\n");
		ser.flush();
#		endif
		watchdog_start((1 << WDP2) | (1 << WDP1)); // 1 second
	}
}

void update_baseline(int32_t& value)
{
#	ifdef DEBUG
	ser.puts("Updating baseline: ");
	ser.puti(value + baseline);
	ser.putc('\n');
	ser.flush();
#	endif
	baseline = value + baseline;
	value = 0;
}

uint8_t calculate_colume(int32_t value, int32_t threshold)
{
	int32_t volume = (255 - MIN_VOLUME) * (value - threshold) * 256 / (threshold * MAX_OVERPULL / 256) / 256;
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
			~(1 << 0);  // ADC0 is batt monitor
	DDRD = 0xff &
			~(1 << 0) & // PD0 = RX
			~(1 << 3);  // PD3 is DOUT

	//PORTD |= (1 << 1); // TX
	PORTD &= ~(1 << 1); // TX
	BATT_PLDWN_PORT |= BATT_PLDWN_MASK; // Batt monitor disable

#	ifdef DEBUG
	ser.puts("Reset\n");
	ser.flush();
#	endif

	// Read the configured force from the EEPROM
	int32_t f = eeprom_read_dword((const uint32_t*) EE_FORCE_ADDRESS);
	if (f != -1)
	{
		pull_threshold = f;
	}

	SCK_PORT &= ~SCK_MASK;  // Set SCK low to initiate conversion
	scale.set_rate(true); // 80Hz

	EICRA |= (1 << ISC11);	// INT1 on falling egde of DOUT
	EIMSK |= (1 << INT1);	// Turns on INT1

	// Disable ADC
	ADCSRA &= ~(1 << 7);

	audio.init();

	watchdog_start((1 << WDP2)  | (1 << WDP0) | (1 << WDP1)); // 1 second

	// Make sure there's at least one transition (DOUT falling) to get shit going
	baseline = scale.read();

	// Main loop
	for(;;)
	{
		// Go to sleep, unless we're currenly beeping. We will wake up when DOUT goes low. This means a conversion is ready.
		if (!audio.is_beeping())
		{
			deep_sleep();
		}

		wdt_reset();

		batt_monitor(mainstate == e_main_state::standby);

		if (mainstate == e_main_state::standby)
		{
			if (!audio.is_beeping())
			{
				scale.wake_up();
				deep_sleep();
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
			if (!audio.is_beeping())
			{
				if (value > pull_threshold + HIST_FORCE)
				{
					audio.set_volume(calculate_colume(value, pull_threshold));

					audio.beep(BEEP_WAVE, sizeof(BEEP_WAVE));
				}
			}
			else
			{
				if (value < pull_threshold - HIST_FORCE)
				{
					audio.beep_disable();
				}
				else
				{
					audio.set_volume(calculate_colume(value, pull_threshold));
				}
			}

			// Check if the user pressed the load cell
			if (value < REVERSE_FORCE - HIST_FORCE)
			{
				audio.alert();
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
					audio.alert();
					_delay_ms(100);
					audio.alert();
					_delay_ms(100);
					audio.alert();

					// And go to config mode
					config_timer = CONFIG_TIME;
					new_pull_threshold = MIN_PULL_FORCE;

					mainstate = e_main_state::config;

					break;
				}

				audio.alert();
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

			// Until we're out of time
			config_timer--;
			if (config_timer == 0)
			{
				if (new_pull_threshold > MIN_PULL_FORCE)
				{
					pull_threshold = new_pull_threshold;
					// Let the user know the time is up
					audio.alert();
					_delay_ms(100);
					audio.alert();
					_delay_ms(100);
					audio.alert();

					// Save the configured value in EEPROM
					eeprom_write_dword((uint32_t*) EE_FORCE_ADDRESS, pull_threshold);
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
			ser.puts("Going to standby\n");
			ser.flush();
#			endif

#			ifndef ALWAYS_80HZ
			scale.set_rate(true); // 80Hz rate (50ms settling time, saves mucho power
#			endif
#			ifdef UPDATE_BASELINE_ON_STANDBY
			update_baseline(value);
#			endif
			audio.beep_disable();

			LED_PORT &= ~LED_MASK;
			blink_timer = 0;

			mainstate = e_main_state::standby;
		}

		// We returned from stand by mode
		if (mainstate == e_main_state::standby && !standby)
		{
#			ifdef DEBUG
			ser.puts("Activity! Waking up from standby\n");
			ser.flush();
#			endif

#			ifndef ALWAYS_80HZ
			scale.set_rate(false); // 10Hz default rate
#			endif

			mainstate = e_main_state::normal;
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
	}
	return 0;
}

ISR(WDT_vect)
{
#	ifdef DEBUG
	//ser.puts("Watchdog Interrupt\n");
	//ser.flush();
#	endif
}

/* DOUT going low vector */
ISR (INT1_vect) { }
