/*
 * phys.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Noah Schwartz
 */
#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#ifndef SRC_HEADER_FILES_PHYS_H_
#define SRC_HEADER_FILES_PHYS_H_

bool platPos_test(int initVel, int time, int acceleration, int initPos, int finalPos);
bool platVel_test(int initVel, int finalVel, int time, int acceleration);
bool leftBounceTest(int x_vel, int final_pos, int time, int left_wall_pos);
bool rightBounceTest(int x_vel, int final_pos, int time, int right_wall_pos);
bool hm_fall_test(int gravity, int y_vel, int init_pos, int time, int final_pos, int final_vel);
bool score_test(int hm_pos, int plat_pos, int cur_score, int final_score);
float updVel(float init_vel, float acceleration, float time);
float updVelKE(int mass, int velocity, int percent);
bool hm_plat_impact(int hm_pos, int plat_pos, int range);
#endif /* SRC_HEADER_FILES_PHYS_H_ */
