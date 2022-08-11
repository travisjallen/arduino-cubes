/*
 * Velocity controller for equivalent shapes experiments
 * 
 * Author: Travis Allen
 * 07.22
 * 
 */ 

#include <Encoder.h>
#include "DualVNH5019MotorShield.h"
#include <movingAvg.h>

// instantiate objects
Encoder enc(2,3);
DualVNH5019MotorShield md;
movingAvg velocity_filter(16);

// buttons
const int no_button = 0;
const int stop_reset = 1;
const int start_experiment = 2;
const int change_frequency = 4;
const int start_motor = 5;
int key_pressed = no_button;
int switch_counts = 0;

// time
double t = 0.0;
double prev_t = 0.0;
double dt = 0.0;
  
// position
long pos = 0;
double prev_pos_s = 0.0;

// velocity
double vel = 0.0;
double vel_d = -8.0;
double prev_vel = 0.0;
double filter_constant = 0.78;
int intvel = 0;

// controller
double error = 0.0;
double error_int = 0.0;
double u = 0.0;
double kp = 800.0; // 810.0 works if no power interruptions
double ki = 0.0; // 0.000031 works if no power interruptions

// motor
double countsPerShaftRev = 1200.0;


void setup() {
  Serial.begin(115200);
  md.init();  
  velocity_filter.begin();
}


void loop() {

  // read the buttons and do something based on the button pressed
  key_pressed = readButtons();
  
  switch (key_pressed) 
  {
    case stop_reset:
    {
      // set the motor speed to 0, reset the integral error, and set switch counts to 1
      md.setM1Speed(0);
      error_int = 0.0;
      switch_counts = 0;

      // delay for switch debounce
      delay(500);
      break;
    }
    case start_experiment:
    {
      // set switch counts to 1, reset integral error
      switch_counts = 1;
      error_int = 0.0;

      // delay for switch debounce
      delay(500);
      break;
    }
    case change_frequency:
    {
      // change desired velocity
      // delay for switch debounce
      delay(500);
      break;
    }
    case start_motor:
    {
      // set switch counts to 2, reset integral error
      switch_counts = 2;
      error_int = 0;
      // delay for switch debounce
      
      delay(500);
      break;
    }
  }


  
  // read the current position
  pos = enc.read();
  double pos_s = ((double)pos) * 2.0 * PI / countsPerShaftRev;;


  // calculate the sampling time
  t = (double) micros();
  dt = t - prev_t;

  // if the sampling time is nonzero, calculate the velocity in rad/s
  if (dt != 0){
    vel = (pos_s - prev_pos_s) / dt * 1000000;
  }
  else {
    Serial.println("it happened");
  }

  // convert velocity to hz
  vel = vel/(2*PI);

  // multiply vel by 100 and cast to an int to work with movingAvg library
  intvel = (int)(100 * vel);

  // get the moving average
  intvel = velocity_filter.reading(intvel);

  // get the averaged velocity as a double
  vel = ((double)intvel) / 100;

  // Plot the average velocity on the serial monitor
  Serial.println(vel);
  
  // compute the error and the integral of the error
  error = vel - vel_d;
  error_int = error_int + error*dt;

  // compute a control signal and constrain it
  u = kp*error + ki*error_int;
  u = constrain(u,-399,399);
  
  int u_i = (int) u;
  
  md.setM1Speed(u_i);

  // update the time and position variables
  prev_t = t;
  prev_pos_s = pos_s;
  prev_vel = vel;

}

int readButtons(){
  int adcKeyIn = analogRead(A7);

  if (adcKeyIn >= 1000) return no_button;
  if (adcKeyIn < 30) return stop_reset;
  if (adcKeyIn < 200) return start_experiment;
  if (adcKeyIn < 500) return no_button;
  if (adcKeyIn < 700) return change_frequency;
  if (adcKeyIn < 990) return start_motor;

  return er0;
}
