/*
 * battery.h
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#ifndef SRC_BATTERY_H_
#define SRC_BATTERY_H_

#include <stdint.h>

class battery {
public:
	static void check(bool sleep, void(*on_sleep)());
	static bool is_dying() { return dying; }

private:
	static int16_t getReading (const uint8_t port, bool sleep);

	static bool dying;
};

#endif /* SRC_BATTERY_H_ */
