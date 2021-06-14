#ifndef SERIAL_H
#define SERIAL_H

#include "config.h"

#ifdef DEBUG

class serial
{
public:
	static void init();

	static void putc(char c);
	static void puts(const char* s);
	static void puti(int32_t x);
	static void flush();
};

#endif /* DEBUG */

#endif /* SERIAL_H */
