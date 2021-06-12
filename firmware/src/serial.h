#ifndef SERIAL_H
#define SERIAL_H

#include "config.h"

#ifdef DEBUG

class serial
{
public:
	serial();

	void putc(char c);
	void puts(const char* s);
	void puti(int32_t x);
	void flush();
};

#endif /* DEBUG */

#endif /* SERIAL_H */
