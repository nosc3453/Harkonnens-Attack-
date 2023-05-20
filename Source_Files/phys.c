/*
 * phys.c
 *
 *  Created on: Mar 31, 2022
 *      Author: Noah Schwartz
 */

#include "phys.h"
#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#include <math.h>
#include <stdlib.h>

bool platPos_test(int initVel, int time, int acceleration, int initPos, int finalPos) //tests update calculations for the platform position and platform velocity
{
  int pCalc = initPos + initVel * time + 0.5 * acceleration * time * time;
  if(pCalc != finalPos)
    {
      return false;
    }
  return true;
}
bool platVel_test(int initVel, int finalVel, int time, int acceleration)
{
  int vCalc = initVel + acceleration * time;
  if(vCalc != finalVel)
    {
      return false;
    }
  return true;
}

bool leftBounceTest(int x_vel, int final_pos, int time, int left_wall_pos) //
{ //before this unit test is called, I will be checking if the position of the ball = position of the left wall
  int xvel_new = -x_vel;
  int calc_pos = left_wall_pos + xvel_new*time;
  if(final_pos == calc_pos)
    {
      return true;
    }
  return false;
}

bool rightBounceTest(int x_vel, int final_pos, int time, int right_wall_pos) //
{ //before this unit test is called, I will be checking if the position of the ball = position of the left wall
  int xvel_new = -x_vel;
  int calc_pos = right_wall_pos + xvel_new*time;
  if(final_pos == calc_pos)
    {
      return true;
    }
  return false;
}

//bounce;//cur_vel, something that rep wall, kinetic energy, returns new_velocity
//{
//}

//update ball position and velocity in y direction test
bool hm_fall_test(int gravity, int y_vel, int init_pos, int time, int final_pos, int final_vel)
{
  int calc_pos = init_pos + y_vel*time + 0.5 * gravity * time * time;
  int calc_vel = y_vel + gravity * time;

  if(calc_pos != final_pos)
    {
      return false;
    }
  if(calc_vel != final_vel)
    {
      return false;
    }
  return true;
}

bool score_test(int hm_pos, int plat_pos, int cur_score, int final_score)//tests the scoring system.
{
  //scoring system adds points based on how close the hm is to the center of the platform. The score adds as a percentage of how close the hm is to the center of the platform.
  int add_pt = (abs(hm_pos-plat_pos) / plat_pos) * 100;
  int new_score = cur_score + add_pt;
  if(final_score != new_score)
    {
      return false;
    }
  return true;
}

float updVel(float init_vel, float acceleration, float time)
{
  return init_vel + acceleration * time;
}

float updVelKE(int mass, int velocity, int percent) //percent can be negative
{
  float ke = 0.5*mass*velocity*velocity;
  ke = ke + percent*ke;
  return sqrt((2*ke) / mass);
}

bool hm_plat_impact(int hm_pos, int plat_pos, int range)
{
  int diff = abs(hm_pos - plat_pos);
  if(diff <= range)
    {
      return true;
    }
  else
    {
      return false;
    }
}
