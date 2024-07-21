/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */


#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3

  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined ARDUINO_MY_COUNTER
  #define LEFT_A 21
  #define LEFT_B 20
  #define RIGHT_A 18
  #define RIGHT_B 19
  void initEncoders();
  void leftEncoderEventA();
  void leftEncoderEventB();
  void rightEncoderEventA();
  void rightEncoderEventB();
#endif

#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) / need for condition*/ {
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined ARDUINO_MY_COUNTER
  #include<PID_v1.h> //调用PID包
  volatile long left_count = 0L;
  volatile long right_count = 0L;

  void initEncoders(){
    pinMode(LEFT_A,INPUT); // 21  --- 2
    pinMode(LEFT_B,INPUT); // 20  --- 3
    pinMode(RIGHT_A,INPUT);// 18  --- 5
    pinMode(RIGHT_B,INPUT);// 19  --- 4

    attachInterrupt(2,leftEncoderEventA,CHANGE);
    attachInterrupt(3,leftEncoderEventB,CHANGE);
    attachInterrupt(5,rightEncoderEventA,CHANGE);
    attachInterrupt(4,rightEncoderEventB,CHANGE);

    pid_a.SetMode(AUTOMATIC);
    pid_b.SetMode(AUTOMATIC);
  }
  //四倍频计算  转数
  void leftEncoderEventA(){
    if(digitalRead(LEFT_A) == HIGH){
      if(digitalRead(LEFT_B) == HIGH){
        left_count++;
      } else {
        left_count--;
      }
    } else {
      if(digitalRead(LEFT_B) == LOW){
        left_count++;
      } else {
        left_count--;
      }
    }
  }
  void leftEncoderEventB(){
    if(digitalRead(LEFT_B) == HIGH){
      if(digitalRead(LEFT_A) == LOW){
        left_count++;
      } else {
        left_count--;
      }
    } else {
      if(digitalRead(LEFT_A) == HIGH){
        left_count++;
      } else {
        left_count--;
      }
    }
  }
  void rightEncoderEventA(){
    if(digitalRead(RIGHT_A) == HIGH){
      if(digitalRead(RIGHT_B) == HIGH){
        right_count++;
      } else {
        right_count--;
      }
    } else {
      if(digitalRead(RIGHT_B) == LOW){
        right_count++;
      } else {
        right_count--;
      }
    }  
  }
  void rightEncoderEventB(){
     if(digitalRead(RIGHT_B) == HIGH){
      if(digitalRead(RIGHT_A) == LOW){
        right_count++;
      } else {
        right_count--;
      }
    } else {
      if(digitalRead(RIGHT_A) == HIGH){
        right_count++;
      } else {
        right_count--;
      }
    }  
  }

  long readEncoder(int i) {
    if (i == LEFT) return left_count;
    else return right_count;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_count=0L;
      return;
    } else { 
      right_count=0L;
      return;
    }
  }
  void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
  }
  // long start_time = millis();
  // int interval_time = 50;
  // int per_round = 160*4;
  
  void myresetPID() //double *target_a,double *target_b,double *kp,double *ki,double *kd
  {
    target_a=0;
    target_b=0;
    kp = 0;
    ki = 0;
    kd = 0;
    pwma = 0;
    pwmb = 0;
  }
void  get_current_vel()
{
  long right_now = millis();
  long past_time = right_now - start_time;
  if(past_time>=interval_time)
  {
    noInterrupts();
    vela = (double)left_count/per_round/past_time*60;
    velb = (double)right_count/per_round/past_time*60;
    // Serial.println(left_count);
    // Serial.println(right_count);
    left_count = 0;
    right_count = 0;
    start_time = right_now;
    interrupts();
    //Serial.println(vel);
  }
}
  /*这是加上自己的pid */

void update_vel()
{

  get_current_vel();
  //调用 Compute函数生成PWM值
  
  if(moving)
  {
    pid_a.Compute();
    pid_b.Compute();
    setMotorSpeeds(pwma,pwmb);
  }
  Serial.println("-----vel----");
  Serial.println(vela);
  Serial.println(velb);
  Serial.println("-----pwm----");
  Serial.println(pwma);
  Serial.println(pwmb);
  Serial.println("------------");


  
}
  #else
  //#error A motor driver must be selected!
#endif