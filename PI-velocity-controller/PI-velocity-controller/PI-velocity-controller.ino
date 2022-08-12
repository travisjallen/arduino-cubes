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
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// instantiate objects
Encoder enc(2,3);
DualVNH5019MotorShield md;
movingAvg velocity_filter(32);
LiquidCrystal_I2C lcd(0x27,16,2);

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
unsigned long current_time = 0;
unsigned long experiment_run_time = 25000; 
unsigned long DC_bias_time = 8000;
  
// position
long pos = 0;
double prev_pos_s = 0.0;

// velocity
double vel = 0.0;
double prev_vel = 0.0;
double filter_constant = 0.78;
int intvel = 0;
int velocity_index = 2;
double desired_velocities[3] = {-4.0, -6.0, -8.0};
double vel_d = desired_velocities[velocity_index];

// controller
double error = 0.0;
double error_int = 0.0;
double u = 0.0;
double proportional_gains[3] = {800.0, 800.0, 800.0};
double integral_gains[3] = {0.0000065, 0.0000065, 0.0000065};
double kp = proportional_gains[velocity_index];
double ki = integral_gains[velocity_index]; 

// motor
double countsPerShaftRev = 1200.0;

// lcd
int lcd_counts = 0;
int lcd_fast = 250;
int lcd_slow = 4000;


void setup() {
  Serial.begin(115200);
  md.init();  
  velocity_filter.begin();
  lcd.init();
  lcd.backlight();
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
      lcd_counts = 0;
            
      // delay for switch debounce
      delay(1000);
      break;
    }
    case start_experiment:
    {
      // set switch counts to 1, reset integral error
      switch_counts = 1;
      error_int = 0.0;
      lcd_counts = 0;
           
      // delay for switch debounce
      delay(1000);
      break;
    }
    case change_frequency:
    {
      // change desired velocity
      velocity_index++;
      if (velocity_index > 2){
        velocity_index = 0;
      }
      vel_d = desired_velocities[velocity_index];
      
      kp = proportional_gains[velocity_index];
      ki = integral_gains[velocity_index];
      
      lcd_counts = 10000;
      
      // delay for switch debounce
      delay(250);
      break;
    }
    case start_motor:
    {
      // set switch counts to 2, reset integral error
      switch_counts = 2;
      error_int = 0.0;
      lcd_counts = 0;
      
      // delay for switch debounce
      delay(1000);
      break;
    }
  }

  //----------------------------------------------------
  // stop/reset
  //----------------------------------------------------
  if (switch_counts == 0){
    // set the motor speed to 0;
    md.setM1Speed(0);
    error_int = 0.0;
    if (lcd_counts >= lcd_slow * 2){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Target: ");
      lcd.setCursor(9,0);
      lcd.print(vel_d);
      lcd.setCursor(0,1);
      lcd.print("Stop");
      lcd_counts = 0;
    }
    lcd_counts++;
  }

  //----------------------------------------------------
  // start experiment
  //----------------------------------------------------
  if (switch_counts == 1){
    
    lcd_counts = 0;
    
    // set up timer
    unsigned long experiment_start_time = millis();
    current_time = millis();

    // DC bias regime
    while ((current_time - experiment_start_time) < DC_bias_time){
      md.setM1Speed(0);
      if (lcd_counts >= lcd_slow){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Target: ");
        lcd.setCursor(9,0);
        lcd.print(vel_d);
        lcd_counts = 0;
      }
      current_time = millis();
      lcd_counts++;
    }

    lcd_counts = 0;
    
    // speed control regime
    while ((current_time - experiment_start_time) < experiment_run_time){
      // check to see if button has been pressed
      readButtons();

      // otherwise control speed
      speedControl();
      current_time = millis();
    }

    // Stop the motor afterward by changing switch counts
    switch_counts = 0;
  }

  //----------------------------------------------------
  // start motor
  //----------------------------------------------------
  if (switch_counts == 2){
    readButtons();
    speedControl();
  }
}

//----------------------------------------------------//----------------------------------------------------//

int readButtons(){
  int adcKeyIn = analogRead(A7);

  if (adcKeyIn >= 1000) return no_button;
  if (adcKeyIn < 30) return stop_reset;
  if (adcKeyIn < 200) return start_experiment;
  if (adcKeyIn < 500) return no_button;
  if (adcKeyIn < 700) return change_frequency;
  if (adcKeyIn < 990) return start_motor;

  return no_button;
}

void speedControl(){
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

  // print to the lcd
  if (lcd_counts >= lcd_fast){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Target: ");
    lcd.setCursor(9,0);
    lcd.print(vel_d);
    lcd.setCursor(0,1);
    lcd.print("Actual: ");
    lcd.setCursor(9,1);
    lcd.print(vel);
    lcd_counts = 0;
  }
  
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

  // update the lcd counts
  lcd_counts++;
}
