#include "ledDriver.h"
#include <Arduino.h>

LedDriver::LedDriver(int _led_pin){
  LED_PIN = _led_pin;
  pinMode(LED_PIN, OUTPUT);

  mode = FAILSAFE;
  prev_time = millis();
  arc_addition = 0;
}

void LedDriver::updateLed(float _heading_error){
  switch(mode){
    case FAILSAFE:
      digitalWrite(LED_PIN, HIGH);
      break;

    case INITIALISING:
      if(millis() % 2000 > 1000){
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }

    case SIMPLE_ARC:
      if(abs(_heading_error) < HEADING_ARC_ANGLE){
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      break;
    
    case INVERTED_ARC:
      if(abs(_heading_error) > INVERSE_HEADING_ARC_ANGLE){
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      break;

    case ANIMATED_ARC:
      arc_addition = cos(2.0f * 3.14f * millis() * 0.001) * 15;
      if(abs(_heading_error) < HEADING_ARC_ANGLE + arc_addition){
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      break;

    case WAKA_WAKA:
      arc_addition = abs(cos(2.0f * 3.14f * millis() * 0.001) * INVERSE_HEADING_ARC_ANGLE);
      if(abs(_heading_error) > arc_addition){
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      break;

    case OCTAGON:
      if(abs(cos(4 * _heading_error * DEG_TO_RAD)) > 0.9){
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      break;

    case LOADING:
      if(abs(_heading_error) < HEADING_ARC_ANGLE){
        digitalWrite(LED_PIN, HIGH);
      } else if(abs(_heading_error) < HEADING_ARC_ANGLE + 40){
        digitalWrite(LED_PIN, LOW);
      } else{
        float angle = abs(sin(2.0f*3.14f*millis() * 0.001 * 0.2))*360;

        if(_heading_error < 0){
          _heading_error += 360;
        }

        if((int)(2*millis()*0.2f) % 1000 > 500){
          angle = 360 - angle;
          if(_heading_error < angle){
            digitalWrite(LED_PIN, HIGH);
          }else{
            digitalWrite(LED_PIN, LOW);
          }
        } else {
          if(_heading_error < angle){
            digitalWrite(LED_PIN, LOW);
          }else{
            digitalWrite(LED_PIN, HIGH);
          }
        }
      }
  }
}

void LedDriver::setPattern(LedMode _led_mode){ if(mode != _led_mode){mode = _led_mode;} }

LedMode LedDriver::getPattern(){return mode;}

void LedDriver::dispalyPattern(){
  switch(mode){
    case FAILSAFE:
      Serial.println("FAILSAFE");
      break;
    case SIMPLE_ARC:
      Serial.println("SIMPLE_ARC");
      break;
    case INVERTED_ARC:
      Serial.println("INVERTED_ARC");
      break;
    case OCTAGON:
      Serial.println("OCTAGON");
      break;
    default:
      Serial.println("UNSPECIFIED");
  }
}


