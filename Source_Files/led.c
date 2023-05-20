/*
 * led.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Noah Schwartz
 */

#include "led.h"
#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#include <math.h>
#include <stdlib.h>

bool leftLED_test(int final_pwm, int max_force, int cur_force, int period) //unit test that tests that the pwm was correctly updated
{
  float new_pwm_percent = (float) ((float) cur_force /(float) max_force); //turn these two lines into a function
  int new_time = period * new_pwm_percent;
  if(final_pwm != new_time)
    {
      return false;
    }
  return true;
}

float calc_lled_pwm(float max_force, float cur_force, float period)
{
  float new_pwm = cur_force / max_force;
  return (float) period * new_pwm;
}
float calc_rled_pwm(float mass, float max_force, float period, float hm_x_init, float hm_y_init, float hm_x_vel, float hm_y_vel, float plat_x_init, float plat_x_vel, float fin_y, float gravity)
{
  float time = calc_time((fin_y - hm_y_init), hm_y_vel, gravity);
  float x_final = hm_x_vel*time + hm_x_init;
  float del_x = x_final - plat_x_init;
  float accel = (-2*(time*plat_x_vel-del_x)) / (time * time);
  float new_pwm = (mass*accel)*10;
  new_pwm = new_pwm / max_force;
  new_pwm = new_pwm * period;
  return abs(new_pwm);
}

float calc_time(float del_y, float vel, float gravity)
{
  float num = sqrt((vel * vel + 2*gravity*del_y)) - vel;
  float time_to_impact =  num / gravity;
  return time_to_impact/10;
}


bool rightLED_test(int accel, int mass, int max_force, int final_pwm, int period)
{
  float new_pwm = (((float) mass* (float) accel)/ (float) max_force) * (float) period;
  new_pwm = abs(new_pwm);
  if(final_pwm != new_pwm)
    {
      return false;
    }
  return true;
}

bool time_to_impact_test(double del_y, double vel, double time, double gravity)
{
  double num = abs(sqrt(abs(vel * vel + 2*gravity*del_y)) - vel);
  double time_to_impact =  num / gravity;
  if(time != time_to_impact)
    {
      return false;
    }
  return true;
}
bool accel_to_impact_test(int hm_x_vel, double time, int hm_x_init, int plat_x_init, int plat_x_vel, int final_a)
{
  int x_final = hm_x_vel*time + hm_x_init;
  int del_x = x_final - plat_x_init;
  int accel = abs((-2*(time*plat_x_vel-del_x)) / (time * time));
  if(accel != final_a)
    {
      return false;
    }
  return true;
}
