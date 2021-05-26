#include <avr/wdt.h>

int pin_dout = 2;
int pin_sck = 11;

void deep_sleep()
{
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  __asm__  __volatile__("sleep");//in line assembler to go to sleep
}

int32_t read()
{
  // Wait until MISO goes low
  int32_t n = 0;
  while (digitalRead(pin_dout) != 0)
  {
    n++;
    // wait...
  }

  Serial.print("n: ");
  Serial.println(n);

  digitalWrite(LED_BUILTIN, 1);
  uint8_t b2 = shiftIn(pin_dout, pin_sck, MSBFIRST);
  uint8_t b1 = shiftIn(pin_dout, pin_sck, MSBFIRST);
  uint8_t b0 = shiftIn(pin_dout, pin_sck, MSBFIRST);
  digitalWrite(LED_BUILTIN, 0);

  /* I don't know what the fuck this is. Without it, sometimes the chip returns all ones. 
   * I found this on https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide/all in the library
   */
  for (unsigned int i = 0; i < 128; i++)
  {
    digitalWrite(pin_sck, HIGH);
    digitalWrite(pin_sck, LOW);
  }

  return (((int32_t)b2 << 24) | ((int32_t)b1 << 16) | ((int32_t)b0 << 8)) >> 8;
}

void watchdog_start()
{
  cli();
  _WD_CONTROL_REG = (1 << WDCE) | (1 << WDE);
  _WD_CONTROL_REG = (1 << WDIE) |
                    (1 << WDP2) | (1 << WDP1); // 1 second
  sei();
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);

  pinMode(pin_dout, INPUT);  // Set pin 15 (MISO) as input
  pinMode(pin_sck, OUTPUT);  // Set pint 16 (SCK) as output
  
  Serial.begin(115200);
  Serial.println("Reset");

  digitalWrite(pin_sck, 0);  // Set SCK low to initiate conversion

  // Interrupt on DOUT going low. Causes wake-up
  attachInterrupt(0, pin_dout_interrupt, FALLING);

  // Disable ADC
  ADCSRA &= ~(1 << 7);
  
  // ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  watchdog_start();

  // Make sure there's at least one transition (DOUT falling) to get shit going
  read();
}

void loop()
{
  // We will wake up when DOUT goes low. This means a conversion is ready.
  deep_sleep();

  wdt_reset();

  digitalWrite(LED_BUILTIN, 1);
  int32_t v = read();
  digitalWrite(LED_BUILTIN, 0);
  //deep_sleep();

  Serial.print("value: ");
  Serial.println(v);
}

void pin_dout_interrupt() { }

ISR(WDT_vect)
{
  Serial.println("Watchdog Interrupt");
}

