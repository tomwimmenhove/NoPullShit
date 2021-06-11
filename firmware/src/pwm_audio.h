#ifndef PWM_AUDIO_H
#define PWM_AUDIO_H

#include <avr/wdt.h>
#include "config.h"
#include <avr/pgmspace.h>

class pwm_audio
{
public:
	void init();
	void beep(const uint8_t* wave, uint8_t len);
	void beep_enable();
	void beep_disable();
	void alert();
	bool is_beeping();
	void set_volume(uint8_t volume);
	uint8_t get_volume();
};

#endif /* PWM_AUDIO_H */

