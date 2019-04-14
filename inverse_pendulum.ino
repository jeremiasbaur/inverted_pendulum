//#include "DegreeCounter.h"
#include "math.h"
#include "PVA.hpp"
#include <PID_v1.h>

//DegreeCounter cart = DegreeCounter(13, 12);
//DegreeCounter endFeet = DegreeCounter(14, 27);

#define cartA 27
#define cartB 14
#define endA 12
#define endB 13

volatile int cart_counter = 0;
volatile int end_counter = 0;

int range_negative = -8000;
int range_positive = 8000;

const int len_last_values = 2;
int last_values[len_last_values];
long long time_counter = 0;

const int sum_len =  100;
const int deriv_len = 0;
int Sums[sum_len];
int sum_error = 0;

volatile int* ptr_cart_counter = &cart_counter;
PVA pva(&cart_counter);

int corr = 0;

// PID parameters
double prop_k = -22.5;
double prop_pos_k = -0.1;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define motor_backward 23
#define motor_forward 22
#define motor_pwm 21

long long t_step = 0;
//gains
double propo = 1;
double deriv = 1;
double integ = 1;

//used to filter noise
//and make integral term not last too long
double filter_deriv = 20;
double filter_integ = 100;
double avg_error [20];
//double sum_error [100];
//vector<double> avg_error (filter_deriv);
//vector<double> sum_error (filter_integ);
double curr_avg = 0;
double curr_sum = 0;

void IRAM_ATTR handleInterruptCart(){
  //portENTER_CRITICAL_ISR(&mux);
  cart_counter += digitalRead(cartA) == digitalRead(cartB) ? -1 : 1;
  //portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR handleInterruptEnd(){
  //portENTER_CRITICAL_ISR(&mux);
  end_counter += digitalRead(endA) == digitalRead(endB) ? -1 : 1;
  //portEXIT_CRITICAL_ISR(&mux);
}

void setup() { 
 pinMode (cartA,INPUT);
 pinMode (cartB,INPUT);
 pinMode (endA,INPUT);
 pinMode (endB,INPUT);

 attachInterrupt(digitalPinToInterrupt(cartA), handleInterruptCart, CHANGE);
 attachInterrupt(digitalPinToInterrupt(endA), handleInterruptEnd, CHANGE);

 pinMode(motor_backward,OUTPUT);
 pinMode(motor_forward,OUTPUT);
 ledcSetup(1, 2000, 8);
 ledcAttachPin(motor_pwm, 0);
  
 Serial.begin (115200);
 
 delay(1000);
 pinMode(0, INPUT);
 while (digitalRead(0)){
     Serial.println("Waiting to reset position and rotation value...");
 }
 
 cart_counter = 0;
 end_counter = 0;

  for(int i = 0; i<len_last_values; i++){
    last_values[i] = 0;
  }

  for(int i = 0; i<sum_len; i++){
    Sums[i] = 0;
  }

 //pva = PVA(cart_counter);

 Serial.println("Setup complete");
 delay(1000);
}

bool change = false;
int motor_counter = 0;
bool break_it = true;
bool forward = true;

void loop() {
  //Serial.print("Cart encoder value: ");
  //Serial.print(cart_counter);
  //Serial.print(" End encoder value: ");
  //Serial.println(end_counter);

  //PID(cart_counter);

  mapMover(cart_counter);
  
  time_counter++;
  //pwmTester();
  /*pva.newPosition();
  Serial.print(cart_counter);
  Serial.print("\tPosition: ");
  Serial.print(pva.getPosition());
  //Serial.print("\tTime now: ");
  //Serial.print(pva.time_now);
  Serial.print("\tVelocity: ");
  Serial.print(pva.getVelocity(), 5);
  Serial.print("\tAccleration: ");
  Serial.println(pva.getAccleration(), 5);*/

  /*Serial.print("& of cart counter: ");
  Serial.print(&cart_counter);
  Serial.print("\t pointer of pva: ");
  Serial.println(pva.position_pointer);*/
  //delay(10);

  if (!digitalRead(0)){
    Serial.println("New parameter input mode:");
    Serial.println("prop_k = ");
    while (Serial.available() == 0){
      delay(1);
    }
    prop_k = (double) Serial.parseFloat();
    Serial.println(prop_k);
    Serial.read();
    Serial.flush();
    delay(1000);
    
    Serial.println("prop_pos_k = ");
    while (Serial.available() == 0){
      delay(1);
    }
    prop_pos_k = (double) Serial.parseFloat();
    Serial.println(prop_pos_k);
    Serial.read();
    Serial.flush();
    delay(1000);
  
    Serial.println("corr = ");
    while (Serial.available() == 0){
      delay(1);
    }
    corr = (int) Serial.parseFloat();
    Serial.println(corr);
    Serial.read();
    Serial.flush();
    delay(1000);
  }
}

int niceAngle(int rotation_value){
  rotation_value = (int)rotation_value % 1200;
  rotation_value += 1200;
  rotation_value = (int)rotation_value % 1200;

  int ans;
  ans = ((rotation_value-600) * 1000) / 600;
  return ans;
}

void mapMover(int angle){
  angle = angle % 1200;
  angle += 1200;
  angle = (angle%1200) -600 + corr ;
  
  //prop_k = -22.5;

  int current_end_counter;
  
  int move_value;
  if (end_counter < 0){
    current_end_counter = -sqrt(-end_counter);
    //current_end_counter = -90;
  } else{
    //current_end_counter = 90;
    current_end_counter = sqrt(end_counter);
  }
  
  move_value = prop_k*angle + current_end_counter * prop_pos_k;
  
  if (motorRangeChecker(true) && (angle > -51 && angle < 51)){
    motorMover(map(move_value, -50, 50, -255, 255));
  }
}

void pMover(int cart_counter){
  int prop_angle = niceAngle(cart_counter)-1;
  
  ledcWrite(0, 0);
  
  double prop_k = 120;
  double prop_pos_k = 0;
  //Serial.println(prop_angle);
  //jSerial.println((prop_angle*255)/1000);
 
  if (motorRangeChecker(true)){
    int calc_val = (prop_angle*255)/1000 * prop_k + (end_counter * prop_pos_k);
    int move_value;
    
    if (calc_val > 0){
      move_value = (calc_val > 255) ? 255 : calc_val; 
    } else {
      move_value = (calc_val < -255) ? -255 : calc_val; 
    }
    if (move_value == 0){
      move_value = last_values[(time_counter+1)%len_last_values];  
    }
    
    last_values[time_counter%len_last_values] = move_value;
    motorMover(-1*move_value);
  }

  /*double k_corr = end_counter/3750;
  double winkel = 0.1;
  
  if (ans>winkel+k_corr){
    forward = true;
  } else if(ans <= -winkel + k_corr){
    forward = false;
  }
  if (forward){
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, HIGH);
    ledcWrite(0, 255);
  } else{
    digitalWrite(motor_backward, HIGH);
    digitalWrite(motor_forward, LOW);
    ledcWrite(0, 255);
  }
  
  if (end_counter>10000 || end_counter<-10000){
    break_it = false;
    ledcWrite(0, 0);
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, LOW);
  }*/
}

void PID(int angle){
  angle = niceAngle(angle);
  ledcWrite(0, 0);
  
  int prop_k = 120;
  double integ_k = 1;
  int deriv_k = 0;
  
  sum_error -= Sums[time_counter%sum_len];
  Sums[time_counter%sum_len] = angle;
  sum_error += angle;
  
  int prop_term = ((angle*255)/1000) * prop_k;
  int integ_term = ((sum_error*255)/1000) * integ_k;
  int ans = prop_term + integ_term;

  Serial.println(sum_error);
  
  if (motorRangeChecker(true)){
    if (ans > 0){
      ans = (ans > 255) ? 255 : ans; 
    } else {
      ans = (ans < -255) ? -255 : ans; 
    }
    
    motorMover(-ans); 
  }
}

void motorMover(int speed_motor){
  if (speed_motor >= 0){ // Forward
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, HIGH);
    ledcWrite(0, speed_motor);
    Serial.print("Forward with speed: ");
    Serial.println(speed_motor);
    
  } else { // Backward
    digitalWrite(motor_backward, HIGH);
    digitalWrite(motor_forward, LOW);
    ledcWrite(0, -1*speed_motor + 19);
    Serial.print("Backward with speed: ");
    Serial.println(-1*speed_motor);
  }
}

bool motorRangeChecker(bool center_after_break){
  if (end_counter < range_negative || end_counter > range_positive){
    Serial.println("Break Break");
    Serial.println(end_counter);
    
    if (center_after_break && end_counter < range_negative){
      while (end_counter < 0){
        motorMover(-150);
        Serial.println("Centering backward");
      }
      motorMover(100);
      delay(100);
    } else if (center_after_break) {
      while (end_counter > 0){
        motorMover(150);
        Serial.println("Centering forward");
      }
      motorMover(-100);
      delay(100);
    }
    motorMover(0);
    return false;
  }
  return true;
}

void pwmTester(){
  for(int i = 8; i<25; i++){
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, HIGH);
    ledcWrite(0, i*10);
    
    Serial.println("Forward");
    Serial.println(i*10);
    
    while(end_counter>-8000){
      Serial.print("Cart encoder value: ");
      Serial.print(cart_counter);
      Serial.print(" End encoder value: ");
      Serial.println(end_counter);
    }
    //delay(1000);
    
    digitalWrite(motor_backward, HIGH);
    digitalWrite(motor_forward, LOW);
    ledcWrite(0, i*10);
    Serial.println("Backward");
    //delay(1000);
    while(end_counter<8000){
      Serial.print("Cart encoder value: ");
      Serial.print(cart_counter);
      Serial.print(" End encoder value: ");
      Serial.println(end_counter);
    }
  }
}

void Mover5000(){
  if (change){
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, HIGH);
    ledcWrite(0, 255);
    Serial.println("Forward");
    change = !change;
    delay(100);
    while(end_counter>-5000){
      Serial.print("Cart encoder value: ");
      Serial.print(cart_counter);
      Serial.print(" End encoder value: ");
      Serial.println(end_counter);
      ledcWrite(0, 255);
      digitalWrite(motor_backward, LOW);
      digitalWrite(motor_forward, HIGH);
    }
  } else{
    digitalWrite(motor_backward, HIGH);
    digitalWrite(motor_forward, LOW);
    ledcWrite(0, 255);
    Serial.println("Backward");
    change = !change;
    delay(100);
    while(end_counter<5000){
      Serial.print("Cart encoder value: ");
      Serial.print(cart_counter);
      Serial.print(" End encoder value: ");
      Serial.println(end_counter);
      ledcWrite(0, 255);
      digitalWrite(motor_backward, HIGH);
      digitalWrite(motor_forward, LOW);
    }
  }
}
