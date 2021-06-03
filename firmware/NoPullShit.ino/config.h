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

#define STANDBY_TIMEOUT (5 * RATE)

#define BATT_COMA_BELOW             (400)
#define BATT_DYING                  (750)
#define BATT_WAKE_AT                (500)

#define BLINK_INTERVAL              (RATE * 2)
#define BLINK_INTERVAL_DYING        (RATE / 2)
#define BLINK_TIME                  (RATE / 40)

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
