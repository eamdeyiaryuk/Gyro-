#include "Dynamixel_for_Arduino.h"
#include <SoftwareSerial.h>
// Math Constant
const float deg2Rad = M_PI/180.f;
float spd_x = 0, spd_y = 0;
float spd1 = 0, spd2 = 0, spd3 = 0; 

SoftwareSerial DX_SERIAL(27,24); // rx,tx  
// Object Define
DX DX_Motor1;
DX DX_Motor2;
DX DX_Motor3;

// Setting  
#define DX_SERIAL_BAUD 57600
#define DRIVE_SPD_RANGE 1000
// Set ID
#define DX_MOTOR1_ID 0
#define DX_MOTOR2_ID 1
#define DX_MOTOR3_ID 2

void dynamixel_init()
{
  DX_Motor1.begin(&DX_SERIAL, DX_SERIAL_BAUD);
  DX_Motor1.setMode("WHEEL");
  DX_Motor1.setID(DX_MOTOR1_ID);
  DX_Motor1.setSpeedRange(-DRIVE_SPD_RANGE, DRIVE_SPD_RANGE);
  DX_Motor2.begin(&DX_SERIAL, DX_SERIAL_BAUD);
  DX_Motor2.setMode("WHEEL");
  DX_Motor2.setID(DX_MOTOR2_ID);
  DX_Motor2.setSpeedRange(-DRIVE_SPD_RANGE, DRIVE_SPD_RANGE);
  DX_Motor3.begin(&DX_SERIAL, DX_SERIAL_BAUD);
  DX_Motor3.setMode("WHEEL");
  DX_Motor3.setID(DX_MOTOR3_ID);
  DX_Motor3.setSpeedRange(-DRIVE_SPD_RANGE, DRIVE_SPD_RANGE);
}

void three_drive(float spd1, float spd2, float spd3)
{
   DX_Motor1.rotate(spd1);
   DX_Motor2.rotate(spd2);
   DX_Motor3.rotate(spd3);  
}

void drive(float spd, float dir, float omega)
{
    spd_x = spd * cos(dir * deg2Rad);
    spd_y = spd * sin(dir * deg2Rad);
 
    spd1 = spd_x + omega;
    spd2 = -0.5*spd_x + 0.866*spd_y + omega;
    spd3 = -0.5*spd_x - 0.866*spd_y + omega;
  
   three_drive(spd1, spd2, spd3);
}
void motor_stop()
{
  unsigned long timer1 = millis();
  three_drive(0,0,0); while(millis() - timer1 < 500);
 
}
