
#include <PS2X_lib.h>

PS2X ps2x; // create PS2 Controller Class
#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK
//#define PS2_DAT 2 //MISO  19
//#define PS2_CMD 3 //MOSI  23
//#define PS2_SEL 4 //SS     5
//#define PS2_CLK 5 //SLK   18

//#define MIN_PWM 0
//#define MAX_PWM 4095
//bool RUN = 0;
int tocdo=4000;
int s1,s2,s3,s4;
void setupPS2controller()
{
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
  ps2x.readType();
  //  ps2x.read_gamepad(false, 0); // disable vibration of the controller
}
bool PS2control()
{
  

  int nJoyX = ps2x.Analog(PSS_LX); // read x-joystick
  int nJoyY = ps2x.Analog(PSS_LY); // read y-joystick

  nJoyX = map(nJoyX, 0, 255, -2046, 2046);
  nJoyY = map(nJoyY, 0, 255, 2046, -2046);

  // OUTPUTS
  int nMotMixL; // Motor (left) mixed output
  int nMotMixR; // Motor (right) mixed output

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  float fPivYLimit = 2046.0;

  // TEMP VARIABLES
  float nMotPremixL; // Motor (left) premixed output
  float nMotPremixR; // Motor (right) premixed output
  int nPivSpeed;     // Pivot Speed
  float fPivScale;   // Balance scale between drive and pivot

  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0)
  {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 2046.0 : (2046.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (2046.0 - nJoyX) : 2046.0;
  }
  else
  {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (2046.0 - nJoyX) : 2046.0;
    nMotPremixR = (nJoyX >= 0) ? 2046.0 : (2046.0 + nJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY / 2046.0;
  nMotPremixR = nMotPremixR * nJoyY / 2046.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  Serial.print(nMotMixL);
  Serial.print("\t");
  Serial.println(nMotMixR);
  int c1 = 0, c2 = 0, c3 = 0, c4 = 0,c5=0,c6=0,c7=0,c8=0;

  if (nMotMixR > 50)
  {
    c3 = nMotMixR;
    // pwm.setPWM(PWM_CHANNEL1, motor_right_speed, MAX_PWM - motor_right_speed );
    // pwm.setPWM(PWM_CHANNEL2, 0, MAX_PWM );
  }

  else if (nMotMixR < -50)
  {
    // pwm.setPWM(PWM_CHANNEL2, motor_right_speed, MAX_PWM - motor_right_speed );
    // pwm.setPWM(PWM_CHANNEL1, 0, MAX_PWM );
    c4 = abs(nMotMixR);
  }

  if (nMotMixL > 50)
  {
    // pwm.setPWM(PWM_CHANNEL3, motor_left_speed, MAX_PWM - motor_left_speed );
    // pwm.setPWM(PWM_CHANNEL4, 0, MAX_PWM );
    c1 = nMotMixL;
  }
  else if (nMotMixL < -50)
  {
    // pwm.setPWM(PWM_CHANNEL4, motor_left_speed, MAX_PWM - motor_left_speed );
    // pwm.setPWM(PWM_CHANNEL3, 0, MAX_PWM );
    c2 = abs(nMotMixL);
  }
  if(ps2x.Button(PSB_GREEN)){
    c7=tocdo;
  }
  if(ps2x.Button(PSB_BLUE)){
    c8=tocdo;
  }
  if(ps2x.Button(PSB_RED)){
    c6=tocdo;
  }
  if(ps2x.Button(PSB_PINK)){
    c6=tocdo;
  }
  // SERVO
  if(ps2x.Button(PSB_L1)){
    s1=205;
  }
  if(ps2x.Button(PSB_L2)){
   s1=410;
  } 
  if(ps2x.Button(PSB_R1)){
    s2=410;
  }
   if(ps2x.Button(PSB_R2)){
    s2=205;
  } 
  if (c1 > 40 || c2 > 40 || c3 > 40 || c4 > 40) {
    setPWMMotors1(c1, c2, c3, c4);
  }else{setPWMMotors1(0, 0, 0, 0);}
  setPWMMotors2(c5,c6,c7,c8);
  servo(s1,s2,s3,s4);
  return 1;

}
