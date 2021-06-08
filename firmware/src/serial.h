#ifndef SERIAL_H
#define SERIAL_H

class serial
{
public:
	serial();

	void putc(char c);
	void puts(const char* s);
	void puti(int32_t x);
	void flush();
};

#endif /* SERIAL_H */
