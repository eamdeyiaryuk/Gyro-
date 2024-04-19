#include "MotorControl.h"
#include "Localize.h"
#include "Read_Light.h"

// Define Program
#define MAIN_PROGRAM
//#define TEST
//#define TEST2
//#define READ

// Define Pin
#define SW_START 16
#define SW_RESET 17
#define DIST_FRONT 4
#define DIST_BACK 25
#define R A7


// Define Parameter
#define TimeStop 500
#define TimeTraffic 3000
#define TimeSkip 9000
#define whiteColor 550
#define curveColor 180

void setup() 
{  
  // Set Serial
  Serial.begin(9600);
  Serial3.begin(115200);
  // Set PinMode
  pinMode(SW_START, INPUT_PULLUP);
  pinMode(SW_RESET, INPUT_PULLUP); 
  pinMode(DIST_FRONT, INPUT_PULLUP);
  pinMode(DIST_BACK, INPUT_PULLUP);
  pinMode(R, INPUT); 
  pinMode(13, OUTPUT);
  dynamixel_init();
  encoder_init();
  light_init();
  // Wait Press Sw
  noInterrupts(); 
  while(digitalRead(SW_START) != 0);
  interrupts();
  
  #ifdef MAIN_PROGRAM 
  delay(100);
  tracking(0, 800, 0);
  check_r(10, 800, 0);
  tracking_low(0, 800, 0);
  tracking(0, 1430, 0);
  tracking(250, 1430, 0);
  check_back();  // Check Manual
  tracking(0, 1430, 0);
  tracking(0, 1700, 0);
  check_r(10, 1700, 0); // 
  tracking_low(0, 1700, 0);
  //check_front();  // Check Traffic 1
  tracking(0, 2000, 0);
  tracking(550, 2000, 0);
  check_r(10, 550, 0);
  tracking_low(0, 550, 0);
  tracking_medium(0, 650, 0);
  //check_traf(); // Camera Process 1
  tracking(0, 1350, 0);
  check_curve(0); // curve 1
  resetPos(0,0);
  check_r2(0, -10, 270);
  tracking_low(0, 0, 270);
  check2(0);
  resetPos(0,0);
  tracking(1100, 0, 270);
  rotate(180);
  resetPos(0, 0);
  tracking_medium(0, -180, 180);
  check_r(-12, -180, 180);
  tracking_low(0, -180, 180);
  tracking(0, -1125, 180);
  check_r(-10, -1125, 180);
  tracking_low(0, -1125, 180);
  //check_front(); // Check Traffic 2
  tracking(0, -1400, 180);
  tracking(-275, -1400, 180);
  check_f(-275, -10, 180);
  tracking_low(-275, 0, 180);
  tracking(-550, 0, 180);
  check_r(-10, 0, 180);
  tracking_low(0, 0, 180);
  tracking_medium(0, -100, 180); 
  //check_traf(); // Camera Process 2
  tracking(0, -750, 180);
  check_curve(180); // Curve 2
  resetPos(0, 0);
  check_r2(0, 10, 450);
  tracking_low(0, 0, 450);
  check2(180);
  resetPos(0, 0);
  tracking(-950, 0, 450);
  #endif


  #ifdef TEST
  #endif  

  #ifdef TEST2
  
  #endif
 
} 

void loop() 
{ 
  #ifdef READ
  Serial.println(analogRead(R));
  //serialEvent3_();
  #endif
}

void check_back()
{
  while(1)
  {
    noInterrupts();
    if(digitalRead(DIST_BACK) == 0)
    {
      delay(TimeStop);
      interrupts();
      break;
    }
  }
}

void check_front()
{
  while(1)
  {
    drive(0, 0, 0);
    if(digitalRead(DIST_FRONT) == 0)
    {
      delay(TimeTraffic);
      interrupts();
      break;
    }
  }
}

void check_r(float x, float y, float head_check)
{
  if(analogRead(R) > whiteColor) // white
  {
    while(1)
    {
      if(analogRead(R) < whiteColor)
      { 
        drive(0,0,0); 
        delay(100);
        resetPos(x, y); 
        break; 
      }
      heading(30, 0, head_check);
    }
  }
  else if(analogRead(R) < whiteColor) // not white
  {
    while(1)
    {
      if(analogRead(R) > whiteColor) 
      { 
        drive(0,0,0); 
        delay(100);
        resetPos(x, y);
        break; 
      }
      heading(30, 180, head_check);
    }
  }
}

void check_r2(float x, float y, float head_check)
{
  if(analogRead(R) > curveColor) // white
  {
    while(1)
    {
      if(analogRead(R) < curveColor)
      { 
        drive(0,0,0); 
        delay(100);
        resetPos(x, y); 
        break; 
      }
      heading(30, 0, head_check);
    }
  }
  else if(analogRead(R) < curveColor) // not white
  {
    while(1)
    {
      if(analogRead(R) > curveColor) 
      { 
        drive(0,0,0); 
        delay(100);
        resetPos(x, y);
        break; 
      }
      heading(30, 180, head_check);
    }
  }
}

void check_f(float x, float y, float head_check)
{
  if(analogRead(R) > whiteColor) // white
  {
    while(1)
    {
      if(analogRead(R) < whiteColor)
      { 
        drive(0,0,0); 
        delay(100);
        resetPos(x, y); 
        break; 
      }
      heading(30, 90, head_check);
    }
  }
  else if(analogRead(R) < whiteColor) // not white
  {
    while(1)
    {
      if(analogRead(R) > whiteColor) 
      { 
        drive(0,0,0); 
        delay(100);
        resetPos(x, y+5);
        break; 
      }
      heading(30, -90, head_check);
    }
  }
}

void check_curve(float set_head)
{
  float head_curve = set_head;
  int i = 0;
  while(1)
  {
    serialEvent3_();
    if(yaw_ >= (245+set_head))     
    { 
      head_curve = 270 + set_head;
      rotate(head_curve); 
      break; 
    }
    else if(i == 0 && roll_ >= 1)
    {
      if(set_head == 0)
      {
        resetPos(0, 0);
        check_r(12, 0, set_head);
        head_curve = 0;
      }
      else if(set_head == 180)
      {
        resetPos(0, 0);
        check_r(-12, 0, set_head);
        head_curve = 180;
      }
      tracking_low(0, 0, set_head);
      i = 1;  
    }
    if(analogRead(R) < curveColor && i >= 1)  
    {
      if(i == 1 && roll_ >= 1)
      {
        if(set_head == 0)
        {
          resetPos(0, 0);
          check_r(12.5, 0, set_head);
          head_curve = 0;
        }
        else if(set_head == 180)
        {
          resetPos(0, 0);
          check_r(-12.5, 0, set_head);
          head_curve = 180;
        }
        tracking_low(0, 0, set_head);
        i++;   
      }
      else if(yaw_ >= (45+set_head) && yaw_ <= (90+set_head))
      {
        head_curve += 15;
        rotate_curve(head_curve);
      }
      else if(yaw_ >= (135+set_head) && yaw_ <= (180+set_head))
      {
        head_curve += 15;
        rotate_curve(head_curve);
      }
      else
      {
        head_curve += 20;
        rotate_curve(head_curve);
      }
      i++;      
    }
    if(roll_ >= 1)
    {
      heading(50, 90, head_curve);
    }
    else
    {
      heading(80, 90, head_curve);
    }
  }
}

void check2(float set_head)
{
  serialEvent3_();
  if(roll_ >= -1)
  {
    while(1)
    {
      if(roll_ <= -1) { drive(0,0,0); delay(100); break; }
      heading(50,90, 270+set_head);
    }
  }
    
}

void check_traf()
{
  unsigned long startTime = millis();
  while(1)
  {
    if(is_light() == true || millis() - startTime >= TimeSkip) { break; }
    drive(0, 0, 0);
  }
}
