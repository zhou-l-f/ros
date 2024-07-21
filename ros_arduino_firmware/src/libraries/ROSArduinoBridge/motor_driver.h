/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#elif defined L298P_MOTOR_DRIVER
  #define MOTORPINA1 6
  #define MOTORPINA2 7
  #define PWMA 5
  #define MOTORPINB1 9
  #define MOTORPINB2 8
  #define PWMB 10
  // #define DIRA 4
  // #define PWMA 5
  // #define DIRB 7
  // #define PWMB 6
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(double leftSpeed, double rightSpeed);
