/*
 * config.h
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#ifndef CONFIG_H
#define CONFIG_H

//#define DEBUG

#define BEEP_WAVE wave_dog

// Minimum volume (0..255) when pulling just over the maximum pull force (255 effectively disables variable volume)
#define MIN_VOLUME		(255)

// Fraction of the maximum pull force over which to go from minimum to maximum volume (1..255 -> 0..100%)
#define MAX_OVERPULL	(128)

#define ALWAYS_80HZ
#define UPDATE_BASELINE_ON_STANDBY

#ifdef ALWAYS_80HZ
#define RATE 80
#else
#define RATE 10
#endif

#define STANDBY_TIMEOUT				(20 * RATE)

#define BATT_COMA_BELOW				(400)
#define BATT_DYING					(750)
#define BATT_WAKE_AT				(500)

#define BLINK_INTERVAL 				(RATE * 2)
#define BLINK_TIME					(RATE / 40)

#define CONFIG_TIME					(5 * RATE)
#define CONFIG_BEEP_TIME			(RATE / 2)
#define ALERT_TIMEOUT				(RATE * 3 / 4)
#define UPDATE_BASELINE_AFTER		(5)

#define REVERSE_FORCE				(-40000)
#define HIST_FORCE					(1000)
#define IDLE_DELTA_FORCE_THRESHOLD	(1000)
#define MIN_PULL_FORCE				(10000)

// Pin definitions:
#define LEDA_PORT PORTD
#define LEDA_MASK (1 << PIND6)
#define LEDB_PORT PORTD
#define LEDB_MASK (1 << PIND5)

#define CHRG_PIN PINC
#define CHRG_PORT PORTC
#define CHRG_MASK (1 << PINC3)

#define STDBY_PIN PINC
#define STDBY_PORT PORTC
#define STDBY_MASK (1 << PINC2)

#define RATE_PORT PORTD
#define RATE_MASK (1 << PIND2)

#define SCK_PORT PORTD
#define SCK_MASK (1 << PIND4)

#define DOUT_PIN PIND
#define DOUT_MASK (1 << PIND3)

#define BATT_PLDWN_PORT PORTC
#define BATT_PLDWN_MASK (1 << PINC1)

#define PWM_DDR DDRB
#define PWM_PORT PORTB
#define PWM_MASK (1 << PINB3)

#define EE_FORCE_ADDRESS 0

#endif /* CONFIG_H */
