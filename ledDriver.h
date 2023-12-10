#include "receiver.h"

#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#define HEADING_ARC_ANGLE 20 // Region in which the led will be turned on
#define INVERSE_HEADING_ARC_ANGLE 35


enum LedMode 
  {
    FAILSAFE,
    INITIALISING,

    SIMPLE_ARC, 
    INVERTED_ARC,
    ANIMATED_ARC,
    WAKA_WAKA,

    OCTAGON,
    LOADING
  };

class LedDriver{
  private:
    int LED_PIN;
    LedMode mode;
    unsigned long prev_time; // For animation
    int arc_addition;

  public:
    LedDriver(int _led_pin);
    void updateLed(float _heading_error); // Needs to be run every loop

    void setPattern(LedMode _led_mode);
    LedMode getPattern();
    void dispalyPattern();
};

#endif