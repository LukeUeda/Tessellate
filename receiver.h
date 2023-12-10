#include "core_pins.h"
#include <cstdlib>

#include <Arduino.h>

#ifndef RECEIVER_H
#define RECEIVER_H

enum ChannelType {TWO_POS_CHANNEL, THREE_POS_CHANNEL, STICK_CHANNEL};

#define POWER_DEADZONE 0.05

typedef struct{
  ChannelType type;
  unsigned int minPulse;
  unsigned int maxPulse;
  unsigned int zeroPulse;
} ChannelConfig;

class Receiver {
private:
  int* pins;
  ChannelConfig* ch_configs;

  volatile uint8_t* prev_channel_states;
  volatile unsigned long* pulse_starts;
  volatile unsigned int* channel_pulses;
  volatile float* channel_powers;

  bool receiver_connected;
  bool failsafe_triggered;

  void getPower(int i);

public:
  Receiver(int _channel_count, int* _channel_pins, ChannelConfig* _channel_configs);
  void isrFunc();

  unsigned int getChannelPulseWidth(int _channel_index);
  float getChannelPower(int _channel_index);

  bool failsafeTriggered();

  void displayChannels();
  void displayChannelPowers();
};
#endif