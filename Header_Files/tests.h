/*
 * tests.h
 *
 *  Created on: Apr 11, 2022
 *      Author: Noah Schwartz
 */

#include <stdio.h>
#include "os.h"
#include "em_emu.h"
#include <math.h>
#include "phys.h"
#include "shield.h"
#include "led.h"

#ifndef SRC_HEADER_FILES_TESTS_H_
#define SRC_HEADER_FILES_TESTS_H_


void main_test();
bool phy_tests();
bool shield_tests();
bool led_tests();

#endif /* SRC_HEADER_FILES_TESTS_H_ */
