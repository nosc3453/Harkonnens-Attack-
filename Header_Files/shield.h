/*
 * shield.h
 *
 *  Created on: Mar 31, 2022
 *      Author: Noah Schwartz
 */
#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#ifndef SRC_HEADER_FILES_SHIELD_H_
#define SRC_HEADER_FILES_SHIELD_H_

bool boost_test(int ke, int x_vel, int y_vel, int x_vel_final, int y_vel_final, int mass);
//bool boostFail_test(int kinReduct, int x_vel, int y_vel, int x_vel_final, int y_vel_final, int kinEn, int mass);
float x_ke_upd(float kinPercent, int x_vel, int y_vel, int mass);
float y_ke_upd(float kinPercent, int x_vel, int y_vel, int mass);
bool keUpd_test(float kinPercent, int x_vel, int y_vel, float final_ke, int mass);
#endif /* SRC_HEADER_FILES_SHIELD_H_ */
