/*
 * standby.h
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#ifndef STANDBY_H
#define STANDBY_H

#include <stdint.h>

#include "config.h"

class standby
{
public:
	inline bool is_standby() { return standby_flag; }
	bool check(int32_t value, int32_t last_value);

private:
	bool standby_flag = true;
	uint16_t standby_timer = 0;
	int32_t value_min = 0;
	int32_t value_max = 0;
	int8_t min_max_counter = 0;
};

#endif /* STANDBY_H */
