#include <stdlib.h>

#include "standby.h"

bool standby::check(int32_t value, int32_t last_value)
{
	int32_t value_diff = abs(value - last_value);
	if (!standby)
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
		if (!standby)
		{
			if (standby_timer++ > STANDBY_TIMEOUT)
			{
				standby = true;
			}
		}
	}
	else
	{
		standby_timer = 0;

		if (standby)
		{
			standby = false;
		}
	}

	return standby;
}
