/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include "cmu.h"
#include "gpio.h"
#include "capsense.h"

#define PRIORITY 20
#define STACK_SIZE 1024
#define IDLE_PRIORITY 50
#define FRICTION_CONST 0.1
struct boost{
  float kin_inc; //percent
  int arm_window; //ms
  int recharge_after_disarm;
  bool recharged;
};
struct bounce{
  bool enabled;
  bool limited;
  int max_platbounce_speed; // cm/s
};
struct h_mass {
  int num; //number of balls in game
  int disp_diameter; //cm
  int init_cond; //fixed = 0.
  uint8_t user_input_mode;
  float x_vel;
  float y_vel;
  float x_pos;
  float y_pos;
  bool alrt;
  bool hm_bounce;
  bool ejected;
  float x_vel_init;
  float y_vel_init;
};
struct platform {
  int maxForce; //N
  int mass; //kg
  int length; //cm
  struct bounce b_can;
  float x_velocity;
  float pos; //cm
  int cur_force;
  bool prev_same;
  bool auto_ctrl;
};
struct h_shield{
  float bnc_reduct; //percent
  struct boost b_set;
  bool press_valid; //
  bool activated;
  uint32_t prev_bt;
  float minPerpSp;
};
struct laser{
  int numActivate;
  bool activated;
  bool auto_ctrl;

};
struct game_settings{
  bool game_over; //initialized to false, true only when the game should be over
  bool game_win;
  int canyon_size;
};
/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void);

void lled_init(void);
void rled_init(void);
void rled_task(void *arg);
void lled_task(void *arg);
void idle_init(void);
void idle_task(void *arg);
void read_capsense();
void tmr_cb(void * p_t,void * p_a);
void game_init();
void detect_init(void);
void phys_upd_init(void);
void lcd_init(void);
void phys_upd_task(void *arg);
void detect_task(void *arg);
void lcd_task(void *arg);
void tmr_cb_phys(void * p_t,void * p_a);
void tmr_cb_rled(void * p_t,void * p_a);
void tmr_cb_lled(void * p_t,void * p_a);
void cap_init(void);
void capsense_task(void * arg);
void tmr_cb_cap(void * p_t,void * p_a);
#endif  // APP_H
