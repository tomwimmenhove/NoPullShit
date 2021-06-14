/*
 * stadby.cpp
 *
 *  Created on: Jun 14, 2021
 *      Author: Tom Wimmenhove
 */

#include <stdlib.h>

#include "standby.h"

bool standby::check(int32_t value, int32_t last_value)
{
	int32_t value_diff = abs(value - last_value);
	if (!standby_flag)
	{
		if (value < value_min) value_min = value;
		if (value > value_max) value_max = value;
		if (min_max_counter++ > 8)
		{
			value_diff = value_max - value_min;
			value_min = value_max = value;
			min_max_counter = 0;
		}
	}  

	if (value_diff < IDLE_DELTA_FORCE_THRESHOLD)
	{
		if (!standby_flag)
		{
			if (standby_timer++ > STANDBY_TIMEOUT)
			{
				standby_flag = true;
			}
		}
	}
	else
	{
		standby_timer = 0;

		if (standby_flag)
		{
			standby_flag = false;
		}
	}

	return standby_flag;
}
