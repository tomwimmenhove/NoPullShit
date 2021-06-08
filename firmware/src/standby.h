#ifndef STANDBY_H
#define STANDBY_H

#include <stdint.h>

#include "config.h"

class standby
{
public:
	inline bool is_standby() { return standby; }
	bool check(int32_t value, int32_t last_value);

private:
	bool standby = true;
	uint16_t standby_timer = 0;
	int32_t value_min = 0;
	int32_t value_max = 0;
	int8_t min_max_counter = 0;
};

#endif /* STANDBY_H */
