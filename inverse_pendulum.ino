// Libraries import
#include "math.h"
#include "PVA.hpp"
//#include "PID.hpp"
#include <PID_v1.h>

// Rotary encoders pinout
#define cartA 27
#define cartB 14
#define endA 12
#define endB 13

// Interrupt stuff of esp32
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Motor driver pinout
#define motor_backward 23
#define motor_forward 22
#define motor_pwm 21

// Rotation and position value of rotary encoders, used in Interrupt function
volatile int cart_counter = 0;
volatile int end_counter = 0;

// max left and max right on linear rail in value of position rotary encoder
int range_negative = -10000;
int range_positive = 10000;

// Proportional experiment stuff
const int len_last_values = 2;
int last_values[len_last_values];
long long time_counter = 0;

// PID integral experiment stuff
const int sum_len =  100;
const int deriv_len = 0;
int Sums[sum_len];
int sum_error = 0;

// PVA setup
int array_length_pva = 50;
volatile int* ptr_cart_counter = &cart_counter;
PVA pva(&cart_counter, array_length_pva);

bool swung_up = false;
int speed_swing_up = 230;

double angle;
double rail_position;
double error_angle;
double last_error_angle;
double timer;
double rail_velocity;
double angle_velocity;
double last_time = 0;
int sampling_time = 10;

const int array_length = 10;
double rail_positions[array_length];
double angle_values[array_length];
double timer_values[array_length];

// parameters for Proportional
double prop_k = -22.5;
double prop_pos_k = -0.1;

long long t_step = 0;

// PID parameters
double output, output_pos, desired_angle, desired_position = 0;
double angle_kp = 0, angle_ki = 0, angle_kd = 0;
double pos_kp = 0, pos_ki = 0, pos_kd = 0;
// Use this to correct left/right leaning
double corr = 0.7;

// PID initialization
PID angle_controller(&angle, &output, &desired_angle, angle_kp, angle_ki, angle_kd, DIRECT);
PID pos_controller(&rail_position, &desired_angle, &desired_position, pos_kp, pos_ki, pos_kd, DIRECT);

//PID angle_controller(&angle, &output, &desired_angle, angle_kp, angle_ki, angle_kd, sampling_time, array_length);
//PID pos_controller(&rail_position, &output_pos, &desired_position, pos_kp, pos_ki, pos_kd, sampling_time, array_length);

bool swing_up_activated = false;

// Debug output on/off
bool motor_mover_debug = false;
bool pid_debug = false;
bool pva_angel_debug = false;
bool round_done_debug = false;
bool motor_range_debug = false;
bool centering_debug = false;
bool swing_up_debug = true;

bool log_cvs_format = true;
// log settings
int log_time_length[] = {500000, 0}; // in milliseconds
bool columns_called = false;

// interrupt function called by change of rotary encoder
// cart encoder for angle
void IRAM_ATTR handleInterruptCart() {
  //portENTER_CRITICAL_ISR(&mux);
  cart_counter += digitalRead(cartA) == digitalRead(cartB) ? -1 : 1;
  //portEXIT_CRITICAL_ISR(&mux);
}
// end rotary encoder for position
void IRAM_ATTR handleInterruptEnd() {
  //portENTER_CRITICAL_ISR(&mux);
  end_counter += digitalRead(endA) == digitalRead(endB) ? -1 : 1;
  //portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  pinMode (cartA, INPUT);
  pinMode (cartB, INPUT);
  pinMode (endA, INPUT);
  pinMode (endB, INPUT);

  attachInterrupt(digitalPinToInterrupt(cartA), handleInterruptCart, CHANGE);
  attachInterrupt(digitalPinToInterrupt(endA), handleInterruptEnd, CHANGE);

  pinMode(motor_backward, OUTPUT);
  pinMode(motor_forward, OUTPUT);
  ledcSetup(1, 2000, 8);
  ledcAttachPin(motor_pwm, 0);

  Serial.begin (115200);

  delay(1000);
  pinMode(0, INPUT);
  while (digitalRead(0)) {
    Serial.println("Waiting to reset position and rotation value...");
  }

  cart_counter = 0;
  end_counter = 0;

  pva.noMovementWaiter();
  cart_counter = 0;
  for (int i = 0; i < array_length; i++) {
    pva.newPosition();
    delay(1);
  }
  
  for (int i = 0; i < len_last_values; i++) {
    last_values[i] = 0;
  }

  for (int i = 0; i < sum_len; i++) {
    Sums[i] = 0;
  }

  //pva = PVA(cart_counter);

  angle_controller.SetMode(AUTOMATIC);
  angle_controller.SetOutputLimits(-255, 255);
  angle_controller.SetSampleTime(sampling_time);
  pos_controller.SetMode(AUTOMATIC);
  pos_controller.SetOutputLimits(-300, 300);
  pos_controller.SetSampleTime(sampling_time);

  delay(1000);
  Serial.println("Setup complete");
}

void loop() {

  if (!digitalRead(0) || angle_kp == 0) {
    motorMover(0);
    Serial.println("New parameter input mode:");
    Serial.println("angle_kp = ");
    while (Serial.available() == 0) {
      delay(1);
    }
    angle_kp = (double) Serial.parseFloat();
    Serial.println(angle_kp);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("angle_kd = ");
    while (Serial.available() == 0) {
      delay(1);
    }
    angle_kd = (double) Serial.parseFloat();
    Serial.println(angle_kd);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("angle_ki = ");
    while (Serial.available() == 0) {
      delay(1);
    }
    angle_ki = (double) Serial.parseFloat();
    Serial.println(angle_ki);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("pos_kp = ");
    while (Serial.available() == 0) {
      delay(1);
    }
    pos_kp = (double) Serial.parseFloat();
    Serial.println(pos_kp, 5);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("pos_kd = ");
    while (Serial.available() == 0) {
      delay(1);
    }
    pos_kd = (double) Serial.parseFloat();
    Serial.println(pos_kd, 5);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("pos_ki = ");
    while (Serial.available() == 0) {
      delay(1);
    }
    pos_ki = (double) Serial.parseFloat();
    Serial.println(pos_ki, 5);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("corr = ");
    while (Serial.available() == 0) {
      delay(1);
    }

    corr = (double) Serial.parseFloat();
    Serial.println(corr);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("speed_swing_up = ");
    while (Serial.available() == 0) {
      delay(1);
    }

    speed_swing_up = (int) Serial.parseFloat();
    Serial.println(speed_swing_up);
    Serial.read();
    Serial.flush();
    delay(100);

    Serial.println("swing_up_activated = ");
    while (Serial.available() == 0) {
      delay(1);
    }

    swing_up_activated = (bool) Serial.parseFloat();
    Serial.println(swing_up_activated);
    Serial.read();
    Serial.flush();
    delay(100);
    //angle_controller.clearSumError();
    //pos_controller.clearSumError();
    columns_called = false;
  }

  if (!swung_up && swing_up_activated) swingUp();

  if (!pidTester(pid_debug) && swing_up_activated) {
    swung_up = false;
    centerCart();
    pva.noMovementWaiter();
    cart_counter = 0;
    for (int i = 0; i < array_length; i++) {
      pva.newPosition();
      delay(1);
    }
    Serial.println("Starting swingup");
    delay(1000);
  }

  //time_counter++; // used in old stuff

  //pwmTester();
  pva.newPosition();
  if (pva_angel_debug) {
    Serial.print("Position: ");
    Serial.print(pva.getPosition());
    Serial.print("\tVelocity: ");
    Serial.print(pva.getVelocity(), 5);
    Serial.print("\tAccleration: ");
    Serial.println(pva.getAccleration(), 5);
  }

  if (millis() - last_time < sampling_time) {
    delay(sampling_time - (millis() - last_time));
  }

  if (round_done_debug) {
    Serial.print("Round done ");
    Serial.println(millis());
  }
  last_time = millis();
}

bool pidTester(bool log_values) {
  for (int i = array_length; i > 1; i--) {
    //Pushing the linear position values one step down in the array to make place at element [0] for the present linear position
    rail_positions[i - 1] = rail_positions[i - 2];
    //Pushing the angle values one step down in the array to make place at element [0] for the present angle
    angle_values[i - 1] = angle_values[i - 2];
    //Pushing the timer values one step down in the array to make place at element [0] for the present time in milliseconds
    timer_values[i - 1] = timer_values[i - 2];
  }

  // Converting value of cart rotary encoder to an value which is 0 at instable position
  int holder = cart_counter % 1200;
  holder += 1200;
  angle = (double)((holder % 1200) - 600);
  rail_position = (double)end_counter;

  timer = millis();
  timer_values[0] = timer;
  angle_values[0] = angle;
  rail_positions[0] = rail_position;

  // Average over last array_length cycles
  rail_velocity = (rail_positions[0] - rail_positions[array_length - 1]) / (timer_values[0] - timer_values[array_length - 1]);
  angle_velocity = (angle_values[0] - angle_values[array_length - 1]) / (timer_values[0] - timer_values[array_length - 1]);

  if (motorRangeChecker(true) && (angle > -46 && angle < 46)) {
    // PID computation for pos
    pos_controller.SetTunings(pos_kp, pos_ki, pos_kd);
    pos_controller.Compute();

    desired_angle += corr;
    // PID computation for angle
    error_angle = desired_angle - angle;

    angle_controller.SetTunings(angle_kp, angle_ki, angle_kd);
    angle_controller.Compute();
    //angle_controller.setParameters(angle_kp, angle_kd, angle_ki);
    //angle_controller.calculateOutput();
    
    if (output > 0) {
      output = map(abs(output), 0, 255, 95, 255);
      motorMover(output);
    } else {
      output = map(abs(output), 0, 255, 95, 255);
      motorMover(-output);
      output = -output;
    }
	
	if (!columns_called){
		log_time_length[1] = millis() + log_time_length[0];
		Serial.println("Desired angle\tError angle\tAngle\tPosition\tAverage angle velocity\tAverage rail velocity\tMotor output\tTime");
		columns_called = true;
		Serial.print(angle_kp,10);
		Serial.print("\t");
    Serial.print(angle_kd,10);
    Serial.print("\t");
		Serial.print(angle_ki,10);
    Serial.print("\t");
		Serial.print(pos_kp,10);
    Serial.print("\t");
		Serial.print(pos_kd,10);
    Serial.print("\t");
		Serial.print(pos_ki,10);
    Serial.print("\t");
		Serial.print(corr);
    Serial.print("\t");
		Serial.println(speed_swing_up);
  }
	
	if (log_cvs_format && millis() < log_time_length[1]){
		Serial.print(desired_angle);
		Serial.print("\t");
		Serial.print(error_angle);
		Serial.print("\t");
		Serial.print(angle);
		Serial.print("\t");
		Serial.print(end_counter);
		Serial.print("\t");
		Serial.print(angle_velocity,10);
		Serial.print("\t");
		Serial.print(rail_velocity,10);
		Serial.print("\t");
		Serial.print(output);
		Serial.print("\t");
		Serial.println(log_time_length[0] - (log_time_length[1]-millis()));
	}
	
    if (log_values) {
      Serial.print("Desired angle: ");
      Serial.print(desired_angle);
      Serial.print("\tError angle: ");
      Serial.print(error_angle);
      Serial.print("\tAngle: ");
      Serial.print(angle);
	  Serial.print("\tPosition: ");
      Serial.print(end_counter);
      Serial.print("\tAverage angle velocity: ");
      Serial.print(angle_velocity);
      Serial.print("\tAverage rail velocity: ");
      Serial.print(rail_velocity);
      Serial.print("\tMotor output: ");
      Serial.print(output);
	  Serial.print("\tTime: ");
      Serial.println(millis());
    }

    return true;
  }
  return false;
}

void swingUp() {
  motorMover(255);
  delay(450);
  for (int i = 0; i <= 50; i++) {
    pva.newPosition();
    delay(1);
  }
  motorMover(-255);
  delay(500);
  motorMover(20);

  int angle = pva.getPosition() % 1200;
  angle += 1200;
  angle = ((angle % 1200) - 600);

  int last_position = pva.getPosition();
  int dir = 1;

  while (!(angle > -31 && angle < 31)) {
    // check if pendulum went through 0
    while (pva.getPosition() == 0 || last_position / pva.getPosition() > 0.00) {
      last_position = pva.getPosition();
      pva.newPosition();
    }

    motorMover(0);
    angle = pva.getPosition() % 1200;
    angle += 1200;
    angle = ((angle % 1200) - 600);

    // start moving cart in other direction of pendulum movement until pendulum has almost 0 speed and reached current amplitude or reached balancing position
    while (pva.getAbsVelocity() > 0.01 && !(angle > -31 && angle < 31)) {
      if (swing_up_debug) Serial.println(end_counter);
      pva.newPosition();
      if (pva.getVelocity() / pva.getAbsVelocity() > 0) dir = 1;
      else dir = -1;

      motorMover(-speed_swing_up * dir);
      if (dir == -1 && end_counter < range_negative + 7000) {
        motorMover(-20);
        if (swing_up_debug) Serial.println("no movement because in danger zone");
      } else if (dir == 1 &&  end_counter > range_positive - 7000) {
        motorMover(20);
        if (swing_up_debug) Serial.println("no movement because in danger zone");
      }
      angle = pva.getPosition() % 1200;
      angle += 1200;
      angle = ((angle % 1200) - 600);
    }
    motorMover(0);
    pva.newPosition();
    angle = pva.getPosition() % 1200;
    angle += 1200;
    angle = ((angle % 1200) - 600);
    last_position = pva.getPosition();
  }
  swung_up = true;
}

// Forward/Backward motor movement with negative/positive integer as arg
void motorMover(int speed_motor) {
  if (speed_motor >= 0) { // Forward
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, HIGH);
    ledcWrite(0, speed_motor);
    if (motor_mover_debug) {
      Serial.print("Forward with speed: ");
      Serial.println(speed_motor);
    }

  } else { // Backward
    digitalWrite(motor_backward, HIGH);
    digitalWrite(motor_forward, LOW);
    ledcWrite(0, -1 * speed_motor); // +19
    if (motor_mover_debug) {
      Serial.print("Backward with speed: ");
      Serial.println(-1 * speed_motor);
    }
  }
}

bool motorRangeChecker(bool center_after_break) {
  if (end_counter < range_negative || end_counter > range_positive) {
    if (motor_mover_debug) {
      Serial.print("Break Break: ");
      Serial.println(end_counter);
    }
    if (center_after_break) {
      centerCart();
    }
    return false;
  }
  return true;
}

void centerCart() {
  if (end_counter < 0) {
    if (centering_debug) Serial.println("Centering backward");
    while (end_counter < 0) {
      motorMover(-150);
    }
    motorMover(100);
    delay(100);
  } else {
    if (centering_debug) Serial.println("Centering forward");
    while (end_counter > 0) {
      motorMover(150);
    }
    motorMover(-100);
    delay(100);
  }
  motorMover(0);
  delay(300);
}

void pwmTester() {
  for (int i = 90; i < 256; i++) {

    motorMover(i);

    int last_value = end_counter;
    delay(100);

    while (end_counter > -6000) {
      Serial.print("Cart encoder value: ");
      Serial.print(cart_counter);
      Serial.print(" End encoder value: ");
      Serial.println(end_counter);
      if (end_counter == last_value) {
        break;
      }
      last_value = end_counter;
      delay(100);
    }

    motorMover(-i);

    last_value = end_counter;
    delay(100);

    while (end_counter < 6000) {
      Serial.print("Cart encoder value: ");
      Serial.print(cart_counter);
      Serial.print(" End encoder value: ");
      Serial.println(end_counter);

      if (end_counter == last_value) {
        break;
      }
      last_value = end_counter;
      delay(100);
    }
  }
}

// old stuff
int niceAngle(int rotation_value) {
  rotation_value = (int)rotation_value % 1200;
  rotation_value += 1200;
  rotation_value = (int)rotation_value % 1200;

  int ans;
  ans = ((rotation_value - 600) * 1000) / 600;
  return ans;
}

void mapMover(int angle) {
  angle = angle % 1200;
  angle += 1200;
  angle = (angle % 1200) - 600 + corr ;

  //prop_k = -22.5;

  int current_end_counter;

  int move_value;
  if (end_counter < 0) {
    current_end_counter = -sqrt(-end_counter);
    //current_end_counter = -90;
  } else {
    //current_end_counter = 90;
    current_end_counter = sqrt(end_counter);
  }

  move_value = angle_kp * angle * abs(angle) + current_end_counter * pos_kp;

  if (motorRangeChecker(true) && (angle > -51 && angle < 51)) {
    motorMover(map(move_value, -50, 50, -255, 255));
  }
}

bool change = false;
int motor_counter = 0;
bool break_it = true;
bool forward = true;

void pMover(int cart_counter) {
  int prop_angle = niceAngle(cart_counter) - 1;

  ledcWrite(0, 0);

  double prop_k = 120;
  double prop_pos_k = 0;
  //Serial.println(prop_angle);
  //jSerial.println((prop_angle*255)/1000);

  if (motorRangeChecker(true)) {
    int calc_val = (prop_angle * 255) / 1000 * prop_k + (end_counter * prop_pos_k);
    int move_value;

    if (calc_val > 0) {
      move_value = (calc_val > 255) ? 255 : calc_val;
    } else {
      move_value = (calc_val < -255) ? -255 : calc_val;
    }
    if (move_value == 0) {
      move_value = last_values[(time_counter + 1) % len_last_values];
    }

    last_values[time_counter % len_last_values] = move_value;
    motorMover(-1 * move_value);
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

// old PI test function
void PID(int angle) {
  angle = niceAngle(angle);
  ledcWrite(0, 0);

  int prop_k = 120;
  double integ_k = 1;
  int deriv_k = 0;

  sum_error -= Sums[time_counter % sum_len];
  Sums[time_counter % sum_len] = angle;
  sum_error += angle;

  int prop_term = ((angle * 255) / 1000) * prop_k;
  int integ_term = ((sum_error * 255) / 1000) * integ_k;
  int ans = prop_term + integ_term;

  Serial.println(sum_error);

  if (motorRangeChecker(true)) {
    if (ans > 0) {
      ans = (ans > 255) ? 255 : ans;
    } else {
      ans = (ans < -255) ? -255 : ans;
    }

    motorMover(-ans);
  }
}

void Mover5000() {
  if (change) {
    digitalWrite(motor_backward, LOW);
    digitalWrite(motor_forward, HIGH);
    ledcWrite(0, 255);
    Serial.println("Forward");
    change = !change;
    delay(100);
    while (end_counter > -5000) {
      Serial.print("Cart encoder value: ");
      Serial.print(cart_counter);
      Serial.print(" End encoder value: ");
      Serial.println(end_counter);
      ledcWrite(0, 255);
      digitalWrite(motor_backward, LOW);
      digitalWrite(motor_forward, HIGH);
    }
  } else {
    digitalWrite(motor_backward, HIGH);
    digitalWrite(motor_forward, LOW);
    ledcWrite(0, 255);
    Serial.println("Backward");
    change = !change;
    delay(100);
    while (end_counter < 5000) {
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
