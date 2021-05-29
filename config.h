#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG

#define BEEP_WAVE wave_sine

#define ALWAYS_80HZ
#define UPDATE_BASELINE_ON_STANDBY

#ifdef ALWAYS_80HZ
#define RATE 80
#else
#define RATE 10
#endif

#ifdef DEBUG
#define STANDBY_TIMEOUT (5 * RATE)
#else
#define STANDBY_TIMEOUT (60 * RATE)
#endif

#define BATT_MINIMUM                (850)

#define CONFIG_TIME                 (2 * RATE)
#define ALERT_TIMEOUT               (RATE * 3 / 4)
#define UPDATE_BASELINE_AFTER       (5)

#define REVERSE_FORCE               (-40000)
#define HIST_FORCE                  (1000)
#define IDLE_DELTA_FORCE_THRESHOLD  (1000)
#define MIN_PULL_FORCE              (10000)

// Pin definitions:
#define LED_PORT PORTB
#define LED_MASK (1 << PINB5)

#define SCK_PORT PORTB
#define SCK_MASK (1 << PINB2)

#define DOUT_PIN PIND
#define DOUT_MASK (1 << PIND2)

#define RATE_PORT PORTD
#define RATE_MASK (1 << PIND3)

#define BATT_PLDWN_PORT PORTC
#define BATT_PLDWN_MASK (1 << PINC1)

#define PWM_DDR DDRB
#define PWM_PORT PORTB
#define PWM_MASK (1 << PINB3)

#define EE_FORCE_ADDRESS 0

#endif /* CONFIG_H */
