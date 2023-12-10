#include <SparkFun_LIS331.h>
#include <Wire.h>
#include <Arduino.h>
#include "ledDriver.h"
#include "receiver.h"

// Pinout
#define MOTOR_1_PIN 4
#define MOTOR_2_PIN 8
#define LED_PIN 11

#define CHANNEL_COUNT 4
#define CH1_PIN 2
#define CH2_PIN 3
#define CH3_PIN 5
#define CH4_PIN 6

#define ACCEL_RADIUS 1.19537 // Distance of accelerometer from centre in mm. (Value is set through calibration)

int channel_pins[CHANNEL_COUNT] = {CH1_PIN, CH2_PIN, CH3_PIN, CH4_PIN};

IntervalTimer meltyLockTimer;

ChannelConfig channel_configurations[4] = {
  {
    .type = THREE_POS_CHANNEL,
    .minPulse = 1010,
    .maxPulse = 2000,
    .zeroPulse = 1510
  },
  {
    .type = STICK_CHANNEL,
    .minPulse = 1118,
    .maxPulse = 1918,
    .zeroPulse = 1505,
  },
  {
    .type = STICK_CHANNEL,
    .minPulse = 1114,
    .maxPulse = 1950,
    .zeroPulse = 1526,
  },
  {
    .type = STICK_CHANNEL,
    .minPulse = 950,
    .maxPulse = 1910,
    .zeroPulse = 1000,
  }
};


float accel_calib[11][2] = {0};

Receiver rx(CHANNEL_COUNT, channel_pins, channel_configurations);

LedDriver led(LED_PIN);

unsigned long calibration_prev_time = 0;
unsigned long steer_prev_time = 0;

float accelerometer_offset = 0;

LIS331 xl;

struct Vector{
  float x;
  float y;
  float z;
};

Vector last_acceleration = {0, 0, 0}; // Last known value of acceleromter

volatile float last_angVelocity = 0; // Last known angular velocity
volatile float heading_error = 0; // Angular error of the led from the forward heading

volatile unsigned long heading_update_previous_time; // Value of the time of previous heading error update

float effective_radius = ACCEL_RADIUS; // Effective radius accounting for translational drift

void setup() {
  // put your setup code here, to run once:
  heading_update_previous_time = micros();
  calibration_prev_time = micros();

  Serial.begin(115200);
  Serial.println("Yeet");

  attachRxInterrupts();

  configureMotorPins();
  initialiseAccelerometer();

  led.setPattern(INITIALISING);
  meltyLockTimer.begin(meltyLockISR, 30);
  armMotors();
}

void loop() {
  if(rx.failsafeTriggered() || rx.getChannelPower(0) == -1){
    led.setPattern(FAILSAFE);

    analogWrite(MOTOR_1_PIN, 2005);
    analogWrite(MOTOR_2_PIN, 2005);

    for(int i=0;i<11;i++){
      Serial.print(i);
      Serial.print(": ");
      Serial.print(accel_calib[i][0], 20);
      Serial.print(", ");
      Serial.println(accel_calib[i][1], 20);
    }

    //Serial.println(last_angVelocity/6);
  }else{
    if(rx.getChannelPower(0) == 0){
      led.setPattern(WAKA_WAKA);
      rx.displayChannels();
      steerHeading(2);
      translationalDrift(3, 1);
    } else {
      //calibrationMode();
      led.setPattern(LOADING);
      rx.displayChannels();
      steerHeading(2);
      translationalDrift(3, 1);
    }

    readAccelerometer();
  }
}

void updateHeadingError(){
  // Updates heading error based on the previous angular velocity and time difference
  shiftHeading(last_angVelocity*(micros()-heading_update_previous_time)/1000000.0f);
  heading_update_previous_time = micros();
}

void shiftHeading(float angle){
  // Shifts the heading according to the angle (+ is CCW, - is CW)
  heading_error -= angle;
  if(heading_error > 180){
    heading_error = 360 - heading_error;
  }

  if(heading_error < -180){
    heading_error = 360 + heading_error;
  }
}

void initialiseAccelerometer(){
  Wire.begin();
  xl.setI2CAddr(0x19);
  xl.setFullScale(LIS331::HIGH_RANGE);
  xl.setODR(LIS331::DR_1000HZ);
  xl.begin(LIS331::USE_I2C);
  xl.axesEnable(true);
  xl.setHighPassCoeff(LIS331::HPC_16);

  for(int count = 0; count < 1000; count++){
    int16_t x, y, z;
    xl.readAxes(x, y, z);
    Serial.println(xl.convertToG(400, x));
    accelerometer_offset += xl.convertToG(400, x) * 9.81;
  }

  accelerometer_offset /= 1000;
}

void readAccelerometer(){
  int16_t x, y, z;
  xl.readAxes(x, y, z);
  last_acceleration.x = xl.convertToG(400, x) * 9.81 - accelerometer_offset;
  last_acceleration.y = xl.convertToG(400, y) * 9.81 - 5.5;
  last_acceleration.z = xl.convertToG(400, z) * 9.81 - 5.5;
}

void calculateAngularVelocity(){
  // Find the angular velocity in degrees;
  last_angVelocity =  sqrt(getCentripitalAcceleration() * RAD_TO_DEG * 1000 / abs(effective_radius));
}

float getCentripitalAcceleration(){
  return sqrt(last_acceleration.x*last_acceleration.x + last_acceleration.y*last_acceleration.y);
}
void configureMotorPins(){
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);

  analogWriteFrequency(MOTOR_1_PIN, 32000);
  analogWriteFrequency(MOTOR_2_PIN, 32000);

  analogWriteResolution(12);
}

void attachRxInterrupts(){
  for(int i = 0; i < CHANNEL_COUNT; i++){
    attachInterrupt(channel_pins[i], rxISR, CHANGE);
  }
}

void rxISR(){
  rx.isrFunc();
}

void armMotors(){
  delay(4500);
  Serial.println("Sending Middle");
  digitalWrite(LED_PIN, HIGH);
  analogWrite(MOTOR_1_PIN, 2005);
  analogWrite(MOTOR_2_PIN, 2005);
  
  delay(1000);
}

void radiusCalibration(int channel_index){
  if(abs(rx.getChannelPower(channel_index)) > 0.2){
    effective_radius += rx.getChannelPower(channel_index) * 0.5 * (micros() - calibration_prev_time) * 0.000001;
  }
  calibration_prev_time = micros();
}

void steerHeading(int channel_index){
  if(abs(rx.getChannelPower(channel_index)) > 0.1){
    heading_error += rx.getChannelPower(channel_index) * 360 * (micros() - steer_prev_time) * 0.000001;
  }
  steer_prev_time = micros();
}

void directSpin(int spin_channel){
  float motor_1_speed = 2005 - rx.getChannelPower(spin_channel) * 1900;
  float motor_2_speed = 2005 - rx.getChannelPower(spin_channel) * 1900;

  analogWrite(MOTOR_1_PIN, motor_1_speed);
  analogWrite(MOTOR_2_PIN, motor_2_speed);
}

void discreteSpin(int spin_level){
  float motor_1_speed = 2005 - (float)spin_level/10.0f * 1900;
  float motor_2_speed = 2005 - (float)spin_level/10.0f * 1900;

  analogWrite(MOTOR_1_PIN, motor_1_speed);
  analogWrite(MOTOR_2_PIN, motor_2_speed);
}

void translationalDrift(int spin_channel, int move_channel){
  float motor_1_speed = 2005 - rx.getChannelPower(spin_channel) * 1900;
  float motor_2_speed = 2005 - rx.getChannelPower(spin_channel) * 1900;

  float translational_heading = heading_error + 45;

  if(translational_heading < -180){
    translational_heading += 360;
  }

  if(translational_heading > 180){
    translational_heading -= 360;
  }

  Serial.println(translational_heading);

  if(translational_heading < 0){
    motor_1_speed -= rx.getChannelPower(move_channel) * 1400;
    motor_2_speed += rx.getChannelPower(move_channel) * 700;
  } else {
    motor_2_speed -= rx.getChannelPower(move_channel) * 700;
    motor_1_speed += rx.getChannelPower(move_channel) * 1400;
  }

  analogWrite(MOTOR_1_PIN, motor_1_speed);
  analogWrite(MOTOR_2_PIN, motor_2_speed);
}

void calibrationMode(){
  led.setPattern(SIMPLE_ARC);
  radiusCalibration(2);
  discreteSpin((int)(rx.getChannelPower(3)*10));

  accel_calib[(int)(rx.getChannelPower(3)*10)][0] = getCentripitalAcceleration();
  accel_calib[(int)(rx.getChannelPower(3)*10)][1] = effective_radius;
}

void meltyLockISR(){
  calculateAngularVelocity();
  updateHeadingError();
  led.updateLed(heading_error);
}