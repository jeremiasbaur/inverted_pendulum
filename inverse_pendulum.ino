//#include "DegreeCounter.h"

//DegreeCounter cart = DegreeCounter(13, 12);
//DegreeCounter endFeet = DegreeCounter(14, 27);


#define cartA 27
#define cartB 14
#define endA 12
#define endB 13

volatile int cart_counter = 0;
volatile int end_counter = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define motor_backward 23
#define motor_forward 22
#define motor_pwm 1

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
double sum_error [100];
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
 ledcSetup(0, 10000, 8);
 ledcAttachPin(motor_pwm, 0);
  
 Serial.begin (115200);
 Serial.println("Setup complete");   
}

bool change = false;
int motor_counter = 0;
bool break_it = true;
bool forward = true;

void loop() {
  Serial.print("Cart encoder value: ");
  Serial.print(cart_counter);
  Serial.print(" End encoder value: ");
  Serial.println(end_counter);

  // cart_counter 360 degrees result in 1200 steps
  double angle = (int)cart_counter%1200;
  angle += 1200;
  angle = (int)angle % 1200;
  double cart_counter_degrees = (360/1200) * angle;
  //Serial.print("PID:\t");
  //Serial.println(PID(double(cart_counter_degrees)));
  Serial.print("Easy:\t");
  Serial.println(Easy(cart_counter));
  EasyMover(cart_counter);
}

double Easy(double angle){
  angle = (int)angle % 1200;
  angle += 1200;
  angle = (int)angle % 1200;
  double ans;
  double prop_k = 0.1;
  ans = (angle-600) * prop_k * 1.5;
  return ans;
}
//input angle -90 to 90
//output number -1 to 1
double PID(double angle){
    angle /= 90;
    double ans;

    curr_avg -= (avg_error[t_step % (int)filter_deriv]/ filter_deriv);
    curr_avg += (angle/filter_deriv);
    avg_error[t_step % (int)filter_deriv] = angle;

    curr_sum -= (sum_error[t_step % (int)filter_integ]/ filter_integ);
    curr_sum += (angle/filter_integ);
    sum_error[t_step % (int)filter_integ] = angle;

    ans = (angle * propo) + ((angle - curr_avg) * deriv) + ((angle - curr_sum) * integ);
    Serial.print(angle * propo);
    Serial.print("\t");
    Serial.print((angle - curr_avg) * deriv);
    Serial.print("\t");
    Serial.print((angle + curr_sum) * integ);
    Serial.println("");
    //cout << (angle * propo) << ' ' << ((angle - curr_avg) * deriv) << ' ' << ((angle + curr_sum) * integ) << '\n';
    ans /= (propo + deriv + integ);
    t_step += 1;
    return ans;
}

void EasyMover(double cart_counter){
  double ans = Easy(cart_counter);
  ledcWrite(0, 0);
  digitalWrite(motor_backward, LOW);
  digitalWrite(motor_forward, LOW);
  Serial.println(ans);

  double k_corr = end_counter/10000;

  if (ans>0.35+k_corr){
    forward = true;
    /*digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, HIGH);
    ledcWrite(0, 255);*/
    Serial.println("Forward");
  } else if(ans <= -0.35 + k_corr){
    forward = false;
    /*digitalWrite(motor_backward, HIGH);
    digitalWrite(motor_forward, LOW);
    ledcWrite(0, 255);*/
    Serial.println("Backward");
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
  
  if (end_counter>5000 || end_counter<-5000){
    break_it = false;
    ledcWrite(0, 0);
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, LOW);
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
