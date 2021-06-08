#ifndef HX711_H
#define HX711_H

class hx711
{
public:
	hx711(volatile uint8_t* dout_pin,  uint8_t dout_mask,
			volatile uint8_t* clk_port,  uint8_t clk_mask,
			volatile uint8_t* rate_port, uint8_t rate_mask)
		: dout_pin(dout_pin),   dout_mask(dout_mask),
		clk_port(clk_port),   clk_mask(clk_mask),
		rate_port(rate_port), rate_mask(rate_mask)
	{ }

	int32_t read();
	void sleep();
	void wake_up();
	void set_rate(bool rate);

private:
	int32_t bang24();

private:
	volatile uint8_t *dout_pin;
	uint8_t dout_mask;

	volatile uint8_t *clk_port;
	uint8_t clk_mask;

	volatile uint8_t *rate_port;
	uint8_t rate_mask;
};

#endif /* HX711_H */
