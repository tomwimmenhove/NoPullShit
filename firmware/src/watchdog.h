/*
 * watchdog.h
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

class watchdog {
public:
	static void set(uint8_t wdp);

	static uint8_t save();

	static void deep_sleep();
};

#endif /* SRC_WATCHDOG_H_ */
