/*
 * shield.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Noah Schwartz
 */

#include "shield.h"
#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#include <math.h>
#include <stdlib.h>

bool boost_test(int ke, int x_vel, int y_vel, int x_vel_final, int y_vel_final, int mass) //tests update calculations for the platform position and platform velocity
{
  float angle = atan2(y_vel,x_vel);
  float newV = sqrt((2*ke)/mass);
  int x_vel_c = newV * sin(angle);
  int y_vel_c = newV * cos(angle);
  if(x_vel_c != x_vel_final)
    {
      return false;
    }
  if(y_vel_c != y_vel_final)
    {
      return false;
    }
  return true;
}
bool keUpd_test(float kinPercent, int x_vel, int y_vel, float final_ke, int mass)
{
  float ke = 0.5*mass * (pow(x_vel,2) + pow(y_vel,2));
  ke = ke + kinPercent * ke;
  if(ke != final_ke)
    {
      return false;
    }
  return true;
}
float x_ke_upd(float kinPercent, int x_vel, int y_vel, int mass)
{
  if((kinPercent < 0) && (x_vel < 0))
    {
      kinPercent = -kinPercent;
    }
  float angle = atan2(y_vel,x_vel);
  float ke = 0.5*mass * (pow(x_vel,2) + pow(y_vel,2));
  ke = ke + kinPercent * ke;
  float newV = sqrt((2*ke)/mass);
  float x_vel_c = newV * sin(angle);
  if((x_vel < 0) && (x_vel_c >= 0))
    {
      x_vel_c = -x_vel_c;
    }
  else if((x_vel >= 0) && (x_vel_c < 0))
    {
      x_vel_c = -x_vel_c;
    }
  return x_vel_c;
}
float y_ke_upd(float kinPercent, int x_vel, int y_vel, int mass)
{
  if((kinPercent < 0) && (y_vel < 0))
    {
      kinPercent = -kinPercent;
    }
  float angle = atan2(y_vel,x_vel);
  float ke = 0.5*mass * (pow(x_vel,2) + pow(y_vel,2));
  ke = ke + kinPercent * ke;
  float newV = sqrt((2*ke)/mass);
  float y_vel_c = newV  * ((float) sin(angle));
  if((y_vel < 0) && (y_vel_c >= 0))
    {
      y_vel_c = -y_vel_c;
    }
  else if((y_vel >- 0) && (y_vel_c < 0))
    {
      y_vel_c = -y_vel_c;
    }
  return y_vel_c;
}







