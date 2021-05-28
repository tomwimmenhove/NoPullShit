#ifndef PWM_AUDIO_H
#define PWM_AUDIO_H

#include <avr/wdt.h>
#include "config.h"
#include <Arduino.h>

class pwm_audio
{
public:
  void init();
  void beep(uint8_t* wave, uint8_t len);
  void beep_enable();
  void beep_disable();
  void alert();
  bool is_beeping();

private:
  static const PROGMEM uint8_t wave_alert[40];
};

#endif /* PWM_AUDIO_H */

