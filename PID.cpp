#include "PID.hpp"
#include "Arduino.h"

PID::PID(double* input, double* output, double* setpoint,
         double Kp, double Kd, double Ki, double sampling_time, int array_length){
  this->input = input;
  this->output = output;
  this->setpoint = setpoint;
  this->sampling_time = sampling_time;
  this->array_length = array_length;
  last_time = millis();
  last_error = 0;
  PID::setParameters(Kp, Kd, Ki);
  sum_error = new double[array_length];
  for(int i = 0; i < array_length; i++){
    sum_error[i] = 0;
  }
  curr_sum = 0;
}

void PID::setParameters(double Kp, double Kd, double Ki){
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
}

void PID::calculateOutput(){
  current_time = millis();
  double time_delta = (double)(current_time - last_time);

  double error = *setpoint - *input;
  
  curr_sum -= sum_error[counter%array_length]/(double)array_length;
  curr_sum += error/(double)array_length;
  sum_error[counter%array_length] = error;
  
  *output = Kp * error;
  *output += Kd * (error - last_error) / time_delta;
  *output += Ki * curr_sum * time_delta; // time delta is always the same in this case
  
  last_error = error;
  last_time = current_time;
  counter++;
}

void PID::clearSumError(){
  for(int i = 0; i < array_length; i++){
    sum_error[i] = 0;
  }
  curr_sum = 0;
}
