/*
 * Velocity controller for equivalent shapes experiments
 * 
 * Author: Travis Allen
 * 07.22
 * 
 */ 

#include <Encoder.h>
#include "DualVNH5019MotorShield.h"

// instantiate encoder and motor driver
Encoder enc(2,3);
DualVNH5019MotorShield md;

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

// controller
double error = 0.0;
double error_int = 0.0;
double u = 0.0;
double kp = 2500.0; // 810.0 works if no power interruptions
double ki = 0.0; // 0.000031 works if no power interruptions

// motor
double countsPerShaftRev = 1200.0;


void setup() {
  Serial.begin(115200);
  md.init();  
}


void loop() {
  // read the current position
  
  pos = enc.read();
  double pos_s = ((double)pos) * 2.0 * PI / countsPerShaftRev;;
  Serial.print(pos_s);

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

  vel = 0.78*vel + 0.22 * prev_vel;
  
  // print the velocity to the serial monitor
  Serial.print("     Velocity (hz): ");
  Serial.print(vel);

  // compute the error and the integral of the error
  error = vel - vel_d;
  error_int = error_int + error*dt;

  // print the error to the serial monitor
  Serial.print("     Error: ");
  Serial.print(error);

  // compute a control signal and constrain it
  u = kp*error + ki*error_int;
  u = constrain(u,-399,399);
  Serial.print("    u = ");
  Serial.println(u);
  int u_i = (int) u;
  md.setM1Speed(u_i);
  

  // update the time and position variables
  prev_t = t;
  prev_pos_s = pos_s;
  prev_vel = vel;

}
