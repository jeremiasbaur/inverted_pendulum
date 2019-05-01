#include "PVA.hpp"
#include "Arduino.h"

#define NULL -1000000000

PVA::PVA(volatile int* position_pointer, const int array_length) {
  this->position_pointer = position_pointer;
  this->last_position = NULL;
  this->time_now = NULL;
  this->time_last = NULL;
  this->current_velocity = NULL;
  this->last_velocity = NULL;
  this->current_accleration = NULL;

  this->array_length = array_length;
  this->position_array = new int[this->array_length];
  this->time_array = new int[this->array_length];
  
  for(int i = 0; i < this->array_length; i++){
    position_array[i] = 0;
    time_array[i] = 0;
  }
}

void PVA::newPosition() {
  time_now = millis();
  //last_position = *position_pointer;
  //time_last = time_now;
  
  for (int i = array_length; i > 1; i--){
    position_array[i-1] = position_array[i-2];
    time_array[i-1] = time_array[i-2];
  }
  
  position_array[0] = *position_pointer;
  time_array[0] = time_now;

  setVelocity();
  setAccleration();
}

int PVA::getPosition(){
  return *position_pointer;
}

int PVA::getModPosition(int mod){
  if (*position_pointer < 0) return -(*position_pointer % 1200);
  return *position_pointer % 1200;
}

double PVA::getVelocity(){
  return this->current_velocity;
}

double PVA::getAbsVelocity(){
  return (this->current_velocity < 0 ? -current_velocity : current_velocity);
}

void PVA::setVelocity(){
  if (this->position_array[array_length-1] != NULL ){
    this->last_velocity = this->current_velocity;
    this->current_velocity = (double)(position_array[0] - position_array[array_length-1]) / (double)(time_array[0] - time_array[array_length-1]);
    }
}

double PVA::getAccleration(){
  return this->current_accleration;
}

void PVA::setAccleration(){
  if (this->current_velocity != NULL && this->last_velocity != NULL){
    //this->current_accleration = ((this->getVelocity()-this->last_velocity)*1)/(double)(this->time_now-this->time_last);
    this->current_accleration = (double)(getVelocity()- last_velocity) / (double)(time_array[array_length-2] - time_array[array_length-1]);
  }
}

bool PVA::noMovementWaiter(){
	int counter = 0;
	int last_value = NULL;
	int still = false;
	
	while (!still){
    while(last_value != *position_pointer){
			last_value = *position_pointer;
			delay(1);
    }
		while(last_value == *position_pointer && counter < 3000){
			last_value = *position_pointer;
			delay(1);
			counter++;
    }
		if (counter >= 3000) still = true;
		counter = 0;
	}
	return true;
}
