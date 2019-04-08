//#include "DegreeCounter.h"

//DegreeCounter cart = DegreeCounter(13, 12);
//DegreeCounter endFeet = DegreeCounter(14, 27);

#define cartA 27
#define cartB 14
#define endA 12
#define endB 13

//volatile int cart_counter = 0;
//volatile int end_counter = 0;

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define motor_backward 23
#define motor_forward 22
#define motor_pwm 1

/*void IRAM_ATTR handleInterruptCart(){
  //portENTER_CRITICAL_ISR(&mux);
  cart_counter += digitalRead(cartA) == digitalRead(cartB) ? -1 : 1;
  //portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR handleInterruptEnd(){
  //portENTER_CRITICAL_ISR(&mux);
  end_counter += digitalRead(endA) == digitalRead(endB) ? -1 : 1;
  //portEXIT_CRITICAL_ISR(&mux);
}*/

/*void setup() { 
 pinMode (cartA,INPUT);
 pinMode (cartB,INPUT);
 pinMode (endA,INPUT);
 pinMode (endB,INPUT);

 attachInterrupt(digitalPinToInterrupt(cartA), handleInterruptCart, CHANGE);
 attachInterrupt(digitalPinToInterrupt(endA), handleInterruptEnd, CHANGE);

 pinMode(motor_backward,OUTPUT);
 pinMode(motor_forward,OUTPUT);
 ledcSetup(0, 10000, 8);
 ledcAttachPin(motor_pwm, 0);
  
 Serial.begin (115200);
 Serial.println("Setup complete");   
}


void loop() {
  Serial.print("Cart encoder value: ");
  Serial.print(cart_counter);
  Serial.print(" End encoder value: ");
  Serial.println(end_counter);
}*/
