#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG

//#define ALWAYS_80HZ

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

#define CONFIG_TIME (2 * RATE)
#define ALERT_TIMEOUT (RATE)
#define REVERSE_FORCE (-40000)
#define IDLE_DELTA_FORCE_THRESHOLD 1000

// Pin definitions:
#define LED_PORT PORTB
#define LED_MASK (1 << PINB5)

#define SCK_PORT PORTB
#define SCK_MASK (1 << PINB2)

#define DOUT_PIN PIND
#define DOUT_MASK (1 << PIND2)

#define RATE_PORT PORTD
#define RATE_MASK (1 << PIND3)

#define PWM_DDR DDRB
#define PWM_MASK (1 << PINB3)

#define EE_FORCE_ADDRESS 0

#endif /* CONFIG_H */
