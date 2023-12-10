#include "receiver.h"

Receiver::Receiver(int _channel_count, int* _channel_pins, ChannelConfig* _channel_configs) {
  //Allocate memory
  pins = (int*)malloc(sizeof(int) * _channel_count);
  ch_configs = (ChannelConfig*)malloc(sizeof(ChannelConfig) * _channel_count);

  for (int i = 0; i < _channel_count; i++) {
    pins[i] = _channel_pins[i];
    ch_configs[i] = _channel_configs[i];
  }

  prev_channel_states = (volatile uint8_t*)malloc(sizeof(uint8_t) * _channel_count);
  pulse_starts = (volatile unsigned long*)malloc(sizeof(unsigned long) * _channel_count);
  channel_pulses = (volatile unsigned int*)malloc(sizeof(unsigned int) * _channel_count);
  channel_powers = (volatile float*)malloc(sizeof(float) * _channel_count);

  failsafe_triggered = true;
}

void Receiver::isrFunc() {
  for (int i = 0; i < 4; i++) {
    uint8_t current_channel_state = digitalRead(pins[i]);
    if (current_channel_state != prev_channel_states[i]) {
      if (current_channel_state == 1) {
        pulse_starts[i] = micros();
      } else {
        channel_pulses[i] = (int)(micros() - pulse_starts[i]);

        if (channel_pulses[0] == 2011 && channel_pulses[1] == 1506 && channel_pulses[2] == 1500 && channel_pulses[3] == 1561 && !failsafe_triggered) {
          failsafe_triggered = true;
        } else {
          getPower(i);
        }
        if (channel_pulses[3] == 1000 && failsafe_triggered) {
          failsafe_triggered = false;
        }
      }
      prev_channel_states[i] = current_channel_state;
    }
  }
}

void Receiver::getPower(int i) {
  switch (ch_configs[i].type) {
    case STICK_CHANNEL:
      channel_powers[i] = 0.0f;
      if (channel_pulses[i] > ch_configs[i].zeroPulse) {
        if (channel_pulses[i] >= ch_configs[i].maxPulse) {
          channel_powers[i] = 1;
        } else {
          channel_powers[i] =
            (float)(channel_pulses[i] - ch_configs[i].zeroPulse) / (float)(ch_configs[i].maxPulse - ch_configs[i].zeroPulse);
        }
      } else if (channel_pulses[i] < ch_configs[i].zeroPulse) {
        if (channel_pulses[i] <= ch_configs[i].minPulse) {
          channel_powers[i] = -1.0f;
        } else {
          {
            channel_powers[i] =
              -(float)(ch_configs[i].zeroPulse - channel_pulses[i]) / (float)(ch_configs[i].zeroPulse - ch_configs[i].minPulse);
          }
        }
      }
      break;
    case THREE_POS_CHANNEL:
      if (channel_pulses[i] > (ch_configs[i].maxPulse + ch_configs[i].zeroPulse) / 2) {
        channel_powers[i] = 1.0f;
      } else if (channel_pulses[i] < (ch_configs[i].minPulse + ch_configs[i].zeroPulse) / 2) {
        channel_powers[i] = -1.0f;
      } else {
        channel_powers[i] = 0.0f;
      }
  }
}

unsigned int Receiver::getChannelPulseWidth(int channel_index) {
  return channel_pulses[channel_index];
}
float Receiver::getChannelPower(int channel_index) {
  if (abs(channel_powers[channel_index]) > POWER_DEADZONE) {
    return channel_powers[channel_index];
  } else {
    return 0;
  };
}

bool Receiver::failsafeTriggered() {
  return failsafe_triggered;
}

void Receiver::displayChannels() {
  for (int i = 0; i < 4; i++) {
    Serial.print(getChannelPulseWidth(i));
    Serial.print(", ");
  }

  if (failsafeTriggered()) {
    Serial.print("Failsafe has been triggered");
  }
  Serial.println();
}

void Receiver::displayChannelPowers() {
  for (int i = 0; i < 4; i++) {
    Serial.print(getChannelPower(i));
    Serial.print(", ");
  }

  if (failsafeTriggered()) {
    Serial.print("Failsafe has been triggered");
  }
  Serial.println();
}
