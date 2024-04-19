#include "Encoder_Interrupts.h"
#include "Read_Gyro.h"

// PID Coef
#define kp_head 1.55f  // 1.55
#define ki_head 0.0001f // 0.0001 
#define kd_head 0.65f //  0.315

#define kp_track 1.77f // 2.11
#define ki_track 0.0001f // 0.001
#define kd_track 0.04895f // 0.0135 

#define kp_rotate 1.85f
#define ki_rotate 0.0001f
#define kd_rotate 0.93f

#define kp_curve 1.55f  // 1.55
#define ki_curve 0.0001f // 0.0001 
#define kd_curve 0.215f //  0.315

// Constant Parameter
#define rad2Deg   180.f/M_PI
#define Deg2Rad   M_PI/180.f
#define pulse2Linear 1.1938052f

// Setting Loop
#define maxSpd 140.f
#define maxSpdLow 25.f
#define maxSpdMedium 75.f
#define spdBegin 65.f
#define maxHeading 35.f
#define maxHeadingCurve 20.f
#define maxRotate 50.f
#define tl_head 1.f
#define tl_track 2.f
#define tl_rotate 0.5f
#define tl_rotateCurve 1.25f

// Global Variable
float pos_x, pos_y, head;

float error_head, prev_error_head, p_head, i_head, d_head;
float error_track, prev_error_track, p_track, i_track, d_track;
int32_t last_odom_1 = 0, last_odom_2 = 0, last_odom_3 = 0;

// Constant Variable Time
const float periodLoopUpdate = 15;
float prevTimeUpdate, currTimeUpdate, diffTimeUpdate;

const float periodLoop = 15;
float prevTime, currTime, diffTime;


void updatePos()
{  
  serialEvent3_();
  int32_t odom1 = pulse1 - last_odom_1;
  int32_t odom2 = pulse2 - last_odom_2;
  int32_t odom3 = pulse3 - last_odom_3;
  
  last_odom_1 = pulse1;
  last_odom_2 = pulse2;
  last_odom_3 = pulse3;
  
  float dx = -odom2 * pulse2Linear;
  float dy = ((odom1-odom3)/2.f) * pulse2Linear;
  head = yaw_; 
  pos_x += (dx * cos(head * Deg2Rad)) - (dy * sin(head * Deg2Rad));
  pos_y += (dx * sin(head * Deg2Rad)) + (dy * cos(head * Deg2Rad));
}

void tracking(float tx, float ty, float th)
{
  int i = 0;
  float error_start;
  prevTimeUpdate = millis();
  while(1)
  {
    currTimeUpdate = millis();
    diffTimeUpdate = currTimeUpdate - prevTimeUpdate;
    if (diffTimeUpdate >= periodLoopUpdate)
      {
        prevTimeUpdate = currTimeUpdate;
        updatePos();
      
        float dx = tx - pos_x;
        float dy = ty - pos_y;
        float dist = sqrt(dx*dx + dy*dy);
        float dir = atan2(dy, dx) * rad2Deg;
        float comp_dir = dir - head;

        if(i == 0) { error_start = dist; }
        
        error_head = th - head;
        //if (error_head < 1 && error_head > -1) { error_head = 0; }
        p_head = error_head;
        i_head += error_head;
        d_head = prev_error_head - error_head;
        
        float output_heading = kp_head*p_head + ki_head*i_head + kd_head*d_head;
        output_heading = constrain(output_heading, -maxHeading, maxHeading);
        prev_error_head = error_head;
        
        error_track = dist;
        //if (error_track < 1 && error_track > -1) { error_track = 0; }
        p_track = error_track;
        i_track += error_track;
        i_track = constrain(i_track, -15, 15);
        d_track = prev_error_track - error_track;
        
        float output_track = kp_track*p_track + ki_track*i_track + kd_track*d_track;
        output_track = constrain(output_track, -maxSpd, maxSpd);
        prev_error_track = error_track;
        
        if(error_track < tl_track && error_track > -tl_track) { drive(0,0,0); delay(100); break; }
        if(error_start - error_track <= error_start/10.f) { output_track = constrain(output_track, -spdBegin, spdBegin);}
        drive(output_track, comp_dir, output_heading);  
        i++;    
      }
  }
}


void tracking_medium(float tx, float ty, float th)
{
  float error_start;
  prevTimeUpdate = millis();
  while(1)
  {
    currTimeUpdate = millis();
    diffTimeUpdate = currTimeUpdate - prevTimeUpdate;
    if (diffTimeUpdate >= periodLoopUpdate)
      {
        prevTimeUpdate = currTimeUpdate;
        updatePos();
      
        float dx = tx - pos_x;
        float dy = ty - pos_y;
        float dist = sqrt(dx*dx + dy*dy);
        float dir = atan2(dy, dx) * rad2Deg;
        float comp_dir = dir - head;
        
        error_head = th - head;
        //if (error_head < 1 && error_head > -1) { error_head = 0; }
        p_head = error_head;
        i_head += error_head;
        d_head = prev_error_head - error_head;
        
        float output_heading = kp_head*p_head + ki_head*i_head + kd_head*d_head;
        output_heading = constrain(output_heading, -maxHeading, maxHeading);
        prev_error_head = error_head;
        
        error_track = dist;
        //if (error_track < 1 && error_track > -1) { error_track = 0; }
        p_track = error_track;
        i_track += error_track;
        i_track = constrain(i_track, -15, 15);
        d_track = prev_error_track - error_track;
        
        float output_track = kp_track*p_track + ki_track*i_track + kd_track*d_track;
        output_track = constrain(output_track, -maxSpdMedium, maxSpdMedium);
        prev_error_track = error_track;
        
        if(error_track < tl_track && error_track > -tl_track) { drive(0,0,0); delay(100); break; }
        drive(output_track, comp_dir, output_heading);    
      }
  }
}

void tracking_low(float tx, float ty, float th)
{
  float error_start;
  prevTimeUpdate = millis();
  while(1)
  {
    currTimeUpdate = millis();
    diffTimeUpdate = currTimeUpdate - prevTimeUpdate;
    if (diffTimeUpdate >= periodLoopUpdate)
      {
        prevTimeUpdate = currTimeUpdate;
        updatePos();
      
        float dx = tx - pos_x;
        float dy = ty - pos_y;
        float dist = sqrt(dx*dx + dy*dy);
        float dir = atan2(dy, dx) * rad2Deg;
        float comp_dir = dir - head;
        
        error_head = th - head;
        //if (error_head < 1 && error_head > -1) { error_head = 0; }
        p_head = error_head;
        i_head += error_head;
        d_head = prev_error_head - error_head;
        
        float output_heading = kp_head*p_head + ki_head*i_head + kd_head*d_head;
        output_heading = constrain(output_heading, -maxHeading, maxHeading);
        prev_error_head = error_head;
        
        error_track = dist;
        //if (error_track < 1 && error_track > -1) { error_track = 0; }
        p_track = error_track;
        i_track += error_track;
        i_track = constrain(i_track, -15, 15);
        d_track = prev_error_track - error_track;
        
        float output_track = kp_track*p_track + ki_track*i_track + kd_track*d_track;
        output_track = constrain(output_track, -maxSpdLow, maxSpdLow);
        prev_error_track = error_track;
        
        if(error_track < tl_track && error_track > -tl_track) { drive(0,0,0); delay(100); break; }
        drive(output_track, comp_dir, output_heading);    
      }
  }
}

void rotate(float th)
{
  prevTimeUpdate = millis();
  while(1)
  {
    currTimeUpdate = millis();
    diffTimeUpdate = currTimeUpdate - prevTimeUpdate;
    if (diffTimeUpdate >= periodLoopUpdate)
    {
      prevTimeUpdate = currTimeUpdate;
      updatePos();
    
      error_head = th - head;
      //if (error_head < 2 && error_head > -2) { error_head = 0; }
      p_head = error_head;
      i_head += error_head;
      d_head = prev_error_head - error_head;
      
      float output_heading = kp_rotate*p_head + ki_rotate*i_head + kd_rotate*d_head;
      output_heading = constrain(output_heading, -maxRotate, maxRotate);
      prev_error_head = error_head;

      if (error_head < tl_rotate && error_head > -tl_rotate) { drive(0,0,0); delay(100); break; }        
      drive(0, 0, output_heading);    
      
    }
  }
}

void rotate_curve(float th)
{
  prevTimeUpdate = millis();
  while(1)
  {
    currTimeUpdate = millis();
    diffTimeUpdate = currTimeUpdate - prevTimeUpdate;
    if (diffTimeUpdate >= periodLoopUpdate)
    {
      prevTimeUpdate = currTimeUpdate;
      updatePos();
    
      error_head = th - head;
      //if (error_head < 2 && error_head > -2) { error_head = 0; }
      p_head = error_head;
      i_head += error_head;
      d_head = prev_error_head - error_head;
      
      float output_heading = kp_rotate*p_head + ki_rotate*i_head + kd_rotate*d_head;
      output_heading = constrain(output_heading, -maxRotate, maxRotate);
      prev_error_head = error_head;

      if (error_head < tl_rotateCurve && error_head > -tl_rotateCurve) { /*drive(0,0,0);*/  break; }
      drive(0, 0, output_heading);    
    }
  }
}
void heading(float spd, float dir, float th)
{
  currTime = millis();
  diffTime = currTime - prevTime;
  if (diffTime >= periodLoop)
  {
    prevTime = currTime;
    updatePos();
    
    error_head = th - head;
    
    p_head = error_head;
    i_head += error_head;
    d_head = prev_error_head - error_head;
    
    float output_heading = kp_curve*p_head + ki_curve*i_head + kd_curve*d_head;
    output_heading = constrain(output_heading, -maxHeadingCurve, maxHeadingCurve);
    prev_error_head = error_head;  
    drive(spd, dir, output_heading);
  }
    
}
void resetPos(float x, float y)
{
    pos_x = x;
    pos_y = y;
}
