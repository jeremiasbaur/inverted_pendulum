#include "PVA.hpp"
#include "Arduino.h"

#define NULL -1000000000

PVA::PVA(volatile int* position_pointer) {
  this->position_pointer = position_pointer;
  this->last_position = NULL;
  this->time_now = NULL;
  this->time_last = NULL;
  this->current_velocity = NULL;
  this->last_velocity = NULL;
  this->current_accleration = NULL;
}

void PVA::newPosition() {
  time_now = micros();
  setVelocity();
  setAccleration();
  last_position = *position_pointer;
  time_last = time_now;
}

int PVA::getPosition(){
  return *position_pointer;
}

double PVA::getVelocity(){
  return this->current_velocity;
}

void PVA::setVelocity(){
  if (this->last_position != NULL && this->time_last != NULL){
    this->last_velocity = this->current_velocity;
    //this->current_velocity = ((double)(this->getPosition()-this->last_position)*1)/(double)(this->time_now-this->time_last);
    this->current_velocity = ((double)(this->getPosition()-this->last_position)*1)/(double)(10);
  }
}

double PVA::getAccleration(){
  return this->current_accleration;
}

void PVA::setAccleration(){
  if (this->current_velocity != NULL && this->last_velocity != NULL){
    //this->current_accleration = ((this->getVelocity()-this->last_velocity)*1)/(double)(this->time_now-this->time_last);
    this->current_accleration = ((this->getVelocity()-this->last_velocity)*1);
  }
}
