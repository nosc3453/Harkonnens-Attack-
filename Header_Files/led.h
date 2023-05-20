/*
 * led.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Noah Schwartz
 */
#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#ifndef SRC_HEADER_FILES_LED_H_
#define SRC_HEADER_FILES_LED_H_

bool leftLED_test(int final_pwm, int max_force, int cur_force, int period);
float calc_lled_pwm(float max_force, float cur_force, float period);
float calc_rled_pwm(float mass, float max_force, float period, float hm_x_init, float hm_y_init, float hm_x_vel, float hm_y_vel, float plat_x_init, float plat_x_vel, float fin_y, float gravity);
bool rightLED_test(int accel, int mass, int max_force, int final_pwm, int period);
bool time_to_impact_test(double del_y, double vel, double time, double gravity);
bool accel_to_impact_test(int hm_x_vel, double time, int hm_x_init, int plat_x_init, int plat_x_vel, int final_a);
float calc_time(float del_y, float vel, float gravity);
#endif /* SRC_HEADER_FILES_LED_H_ */
