/*
 * audio.h
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#ifndef AUDIO_H
#define AUDIO_H

#include <avr/wdt.h>
#include "config.h"
#include <avr/pgmspace.h>

class audio
{
public:
	void static init();
	void static play(const uint8_t* wave, uint8_t len);
	void static enable();
	void static disable();
	void static alert(int ms);
	bool static is_enabled();
	void static set_volume(uint8_t volume);
	uint8_t static get_volume();
};

#endif /* AUDIO_H */

