/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/




#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* Encoders directly attached to Arduino board */
   //#define ARDUINO_ENC_COUNTER

    /* 使用自定义的编码器驱动 */
   #define ARDUINO_MY_COUNTER

   /* L298 Motor driver*/
   //#define L298_MOTOR_DRIVER

    #define L298P_MOTOR_DRIVER

#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE                            //调用我们编写的头文件头文件
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           1     // Hz PID调试频率

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE; // PID调试周期

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL; //PID调试的结束时刻标记

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 5000
  long lastMotorCommand = AUTO_STOP_INTERVAL;

  long start_time = millis();
  #define interval_time  50
  #define per_round  160*4
  #include<string.h>
  #include<math.h>
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str,*str1,*str2;
  double pid_args[4];
  arg1 = atoi(argv1); //atoi(NULL) 返回值是0
  arg2 = atoi(argv2); //这两行代码的意义是将输入的字符参数转化为对应的整数，如速度等
  
  switch(cmd) {
  case GET_BAUDRATE:    //输出当前设定的比特率 奇奇怪怪的东西 知道特定比特率我才能接受返回的信息啊
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:  //读取对应pwm引脚的大小
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ: //读取对应引脚的高低频
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE://修改对应pwm引脚arg1的大小,大小为arg2 
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE: //修改对应引脚的高低频 arg2为0/1
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:    //修改对应引脚的输入输出模式
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING: //输出对应引脚的输入输出模式
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS //我们没有使用舵机不管这个
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:  //读取左右轮的转数
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS: //重置转数
    resetEncoders();
    // resetPID(); 
    myresetPID(); //&target_a,&target_b,&kp,&ki,&kd
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS: //---------------------------------------------
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      myresetPID(); //&target_a,&target_b,&kp,&ki,&kd
      // resetPID();
      moving = 0;
      //myresetPID(&target_a,&target_b,&kp,&ki,&kd);
    }
    else moving = 1;
    //设置左右电机目标转速分别为参数1和参数2
    // leftPID.TargetTicksPerFrame = arg1;
    // rightPID.TargetTicksPerFrame = arg2;

    target_a = arg1;
    target_b = arg2;
    Serial.println(target_a); 
    Serial.println(target_b); 
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {    //strtok_r函数的作用按某个字符来分割字符串。p指向的是argv1栈 在这个命令下的参数argv1存储的是PID的三个参数
      
      str1 = strtok_r(str, ".", &str2);
      int len = strlen(str2);
      if(len)
      pid_args[i] = atoi(str1)+pow(10,-len)*atoi(str2);
      else
      pid_args[i] = atoi(str1);
      i++;
      //  pid_args[i] = atoi(str);
      //  i++;
    }
    kp = pid_args[0];
    kd = pid_args[1];
    ki = pid_args[2];
    Serial.println(kp);
    Serial.println(kd);
    Serial.println(ki);

    //Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
    #elif defined ARDUINO_MY_COUNTER  //我们用的是这个
    initEncoders();
    #endif
    initMotorController();
    //resetPID();   //需要加入我们自己写的PID 
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {

  /*
    chr 是用来从系统缓冲区中一个一个读取字符并存储到chr中
    cmd 是当第一个存储进来的chr就赋值给cmd对应命令的种类
    arg 是用来记录 后续读进来的非空格字符数 最大两个
    argv 是用来保存两个输入进来的参数值
  */
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {   //  13(CR)  对应回车键
      if (arg == 1) argv1[index] = NULL;  
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL; 
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  // if (millis() > nextPID) {
  //   //updatePID();
  //   nextPID += PID_INTERVAL;
  // }
  // if (millis() > nextPID) { //沿用了原本的达到一定时间就进行一次pid控速，相当于我们自己写的delay(1000)
  //   update_vel();
  //   nextPID += PID_INTERVAL;
  // }

  // // Check to see if we have exceeded the auto-stop interval
  // if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {; //超过5秒不进行速度设定操作就停下来
  //   setMotorSpeeds(0, 0);
  //   moving = 0;
  // }
  delay(1000);
  update_vel();
  // setMotorSpeeds(pwma,pwmb);
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}

