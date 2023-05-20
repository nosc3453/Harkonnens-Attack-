/*
 * tests.c
 *
 *  Created on: Apr 11, 2022
 *      Author: Noah Schwartz
 */
#include "tests.h"
#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#include <math.h>
#include "phys.h"
#include "shield.h"
#include "led.h"


void main_test()
{
  if(!phy_tests())
    {
      while(1);
    }
  if(!shield_tests())
    {
      while(1);
    }
  if(!led_tests())
    {
      while(1);
    }
}

bool phy_tests()
{
  //Position test 1 - No Acceleration
    if(!platPos_test(6, 3, 0, 5, 23))
      {
        return false;
      }
  //Position test 2 - No initial velocity
    if(!platPos_test(0, 3, 10, 5, 50))
      {
        return false;
      }
  //Position test 3 - Random Values
    if(!platPos_test(15, 9, 3, 0, 256))
      {
        return false;
      }

  //Velocity test 1 - No Acceleration
    if(!platVel_test(5,5,12,0))
      {
        return false;
      }
  //Velocity test 2 - No initial velocity
    if(!platVel_test(0,60,12,5))
      {
        return false;
      }
  //Velocity test 3 - Random Values
    if(!platVel_test(4,49,5,9))
      {
        return false;
      }
  //Velocity test 4 - More Random Values
    if(!platVel_test(-6,-46,8,-5))
      {
        return false;
      }
    //Left Bounce Test 1 - No X velocity
    if(!leftBounceTest(0,3,5,3)) //can keep wall position same through the tests
      {
        return false;
      }
    //Left Bounce Test 2 - Random values
    if(!leftBounceTest(-8,43,5,3)) //can keep wall position same through the tests
      {
        return false;
      }
    //Left Bounce Test 3 - More random values
    if(!leftBounceTest(-8,35,4,3)) //can keep wall position same through the tests
      {
        return false;
      }
    //Right Bounce Test 1 - No X velocity
    if(!rightBounceTest(0,50,5,50)) //can keep wall position same through the tests
      {
        return false;
      }
    //Right Bounce Test 2 - Random values
    if(!rightBounceTest(8,10,5,50)) //can keep wall position same through the tests
       {
         return false;
       }
    //Right Bounce Test 3 - More random values
    if(!rightBounceTest(3,38,4,50)) //can keep wall position same through the tests
       {
         return false;
       }
    //Fall Test 1 - No gravity
    if(!hm_fall_test(0,8,6,5,46,8))
       {
         return false;
       }
    //Fall Test 2 - Random Numbers
    if(!hm_fall_test(-10,-8,7,4,-105,-48))
       {
         return false;
       }
    //Fall Test 3 - More Random Numbers
    if(!hm_fall_test(-9,-2,3,5,-119,-47))
       {
         return false;
       }
    //HM-Platform impact test 1 - In Range
    if(!hm_plat_impact(10,15,30))
       {
         return false;
       }
    //HM-Platform impact test 2 - Another In Range
    if(!hm_plat_impact(20,10,30))
       {
         return false;
       }
    //HM-Platform impact test 3 - Not in Range
    if(hm_plat_impact(10,16,5))
       {
         return false;
       }
    //HM-Platform impact test 4 - Also Not in Range
    if(hm_plat_impact(20,10,9))
       {
         return false;
       }
    return true;
}

bool shield_tests()
{
  //KE Update Test 1 - Random Values with KE increase
  if(!keUpd_test(0.05,6,7,669.375,15))
     {
       return false;
     }
  //KE Update Test 2 - Random Values with KE decrease
  if(!keUpd_test(-0.05,3,5,242.25,15))
     {
       return false;
     }
  if(!boost_test(669,6,7,7,6,15))
     {
       return false;
     }
  if(!boost_test(242,3,5,4,2,15))
     {
       return false;
     }
  return true;
}
bool led_tests()
{
  //Left LED Test 1 - Random Values
  if(!leftLED_test(500,100,50,1000))
    {
      return false;
    }
  //Left LED Test 2 - Random Values
  if(!leftLED_test(73,100,49,150))
    {
      return false;
    }
  //Time Test 1 - No velocity
  if(!time_to_impact_test(5,0,1.0,10))
    {
      return false;
    }
  //Time Test 2 - Random Values
  if(!time_to_impact_test(3,6,0.375,8))
    {
      return false;
    }
  //Time Test 3 - More Random Values
  if(!time_to_impact_test(1,5,0.1,10))
    {
      return false;
    }
 // bool accel_to_impact_test(int hm_x_vel, double time, int hm_x_init, int plat_x_init, int plat_x_vel, int final_a)
//  {
//    int x_final = hm_x_vel*time + hm_x_init;
//    int del_x = x_final - plat_x_init;
//    int accel = (-2*(time*plat_x_vel-del_x)) / (time * time);
  if(!accel_to_impact_test(5,1.0,3,2,7,2))
    {
      return false;
    }
  if(!accel_to_impact_test(8,0.375,5,0,4,92))
    {
      return false;
    }
  if(!accel_to_impact_test(10,0.1,6,2,3,939))
    {
      return false;
    }
//  bool rightLED_test(int accel, int mass, int max_force, int final_pwm, int period)
//    float new_pwm = abs(((mass*accel)/ max_force) * period);
  if(!rightLED_test(2,15,100,300,1000))
    {
      return false;
    }
  if(!rightLED_test(92,15,2000,690,1000))
    {
      return false;
    }
  if(!rightLED_test(939,15,15000,939,1000))
    {
      return false;
    }
  return true;
}
