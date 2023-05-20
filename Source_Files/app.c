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
#include "app.h"
#include "os.h"
#include "em_emu.h"
#include "glib.h"
#include "dmd.h"
#include "fifo.h"
#include "sl_board_control.h"
#include "sl_board_control_config.h"
#include <stdio.h>
#include "phys.h"
#include "shield.h"
#include <math.h>
#include "led.h"
#include "tests.h"
#include <stdlib.h>


static OS_TCB tcb1; //make a new tcb for each task
static OS_TCB tcb2; //make a new tcb for each task
static OS_TCB tcb3; //make a new tcb for each task
static OS_TCB tcb4; //make a new tcb for each task
static OS_TCB tcb5; //make a new tcb for each task
static OS_TCB tcb6; //make a new tcb for each task
static OS_TCB tcb7; //make a new tcb for each task


static CPU_STK stack1[STACK_SIZE]; //each task needs to have its own stack
static CPU_STK stack2[STACK_SIZE];
static CPU_STK stack3[STACK_SIZE]; //idle task = 5 priority
static CPU_STK stack4[STACK_SIZE];
static CPU_STK stack5[STACK_SIZE];
static CPU_STK stack6[STACK_SIZE];
static CPU_STK stack7[STACK_SIZE];

struct bt_node * b0_s = NULL;
struct bt_node * b1_s = NULL;
int plat_range;
#define DEF_NULL 0
CPU_BOOLEAN start;
int l_wall_pos;
int r_wall_pos;
int plat_y_pos;
int in_force = 0;
float gravity;
int hm_radius;
static int currentLine = 0;
float base_lled_period;
float base_rled_period;
OS_SEM sem_bt; //btn update, lcd, direction time
OS_SEM sem_phys;
OS_SEM sem_disp;
OS_SEM sem_lled;
OS_SEM sem_rled;
OS_SEM sem_cap;

OS_MUTEX mut_laser;
OS_MUTEX mut_shield;
OS_MUTEX mut_plat;
OS_MUTEX mut_hm;
OS_MUTEX mut_cap;
OS_TMR pd_tmr;
OS_TMR s_tmr;
OS_TMR r_tmr;
OS_TMR t_phy;
OS_TMR t_dis;
OS_TMR t_rled;
OS_TMR t_lled;
OS_TMR t_cap;


float phy_period;
float dis_period;
float cap_period;
float rled_period;
float lled_period;
static GLIB_Context_t glibContext;
struct h_mass h_settings;
struct h_shield shield_settings;
struct platform plat_settings;
struct laser l_settings;
struct game_settings g_set;
CPU_BOOLEAN start;
bool bt0_b = false;
bool bt1_b = false;
//can only unit test things where we put numbers in and get numbers out

/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  RTOS_ERR err;
  GPIO_IntClear(1 << BUTTON0_pin);
  int state = GPIO_PinInGet(BUTTON0_port, BUTTON0_pin);
  if(is_empty(b0_s) == 0)
    {
      b0_s = create_queue(state);
    }
  else
    {
      b0_s = push(b0_s, state);
    }
  if(state == 0)
    {
      bt0_b = true;
    }
  OSSemPost(&sem_bt, OS_OPT_POST_1, &err);

}
/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  RTOS_ERR err;
  GPIO_IntClear(1 << BUTTON1_pin);
  int state = GPIO_PinInGet(BUTTON1_port, BUTTON1_pin);
  if(is_empty(b1_s) == 0)
    {
      b1_s = create_queue(state);
    }
  else
    {
      b1_s = push(b1_s, state);
    }
  if(state == 0)
    {
      bt1_b = true;
    }
  OSSemPost(&sem_bt, OS_OPT_POST_1, &err);

}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  gpio_open();
  //Maybe call tests here?
  main_test();
 // while(1); //to stop and check that my unit tests work
  game_init();
  detect_init();
  phys_upd_init();
  lcd_init();
  lled_init();
  rled_init();
  cap_init();
  idle_init();
}
void game_init()
{
  RTOS_ERR err;
  plat_settings.maxForce = 2000000;
  plat_settings.b_can.enabled = true;
  plat_settings.b_can.limited = false;
  plat_settings.b_can.max_platbounce_speed = 20; //NEED TO COME BACK TO THIS 4/18/22
  plat_settings.mass = 100;
  plat_settings.x_velocity = 0.0;
  plat_settings.pos = 50;
  plat_settings.length = 20;
  plat_settings.cur_force = 0;
  plat_settings.prev_same = false;
  plat_range = 20;
  plat_settings.auto_ctrl = false;
  phy_period = 1;
  dis_period = 5;
  cap_period = 20;
  rled_period = 150;
  lled_period = 50;
  base_lled_period = lled_period;
  base_rled_period = rled_period;
  gravity = 0.098*2; //0.00098
  hm_radius = 2; //good
  g_set.canyon_size = 123;
  r_wall_pos = g_set.canyon_size+2;
  l_wall_pos = 2;

  g_set.game_win = false;
  l_settings.numActivate = 1;
  l_settings.auto_ctrl = false;

  g_set.game_over = false;

  plat_y_pos = 110;

  shield_settings.b_set.arm_window = 50;
  shield_settings.b_set.recharge_after_disarm = 100;
  shield_settings.press_valid = false;
  shield_settings.b_set.kin_inc = 0.40;
  shield_settings.b_set.recharged = true;
  shield_settings.activated = false;
  shield_settings.bnc_reduct = -0.25;
  shield_settings.prev_bt = 1;
  shield_settings.minPerpSp = 2;

  h_settings.num = 1;
  h_settings.alrt = false;
  h_settings.y_vel_init = 3;
  h_settings.x_vel_init = 5;
  h_settings.x_pos = rand() % 90 + 2;
  h_settings.y_pos = rand() % 15;
  h_settings.x_vel = h_settings.x_vel_init;
  h_settings.y_vel = h_settings.y_vel_init;
  h_settings.disp_diameter = hm_radius;
  h_settings.hm_bounce = false;
  h_settings.user_input_mode = 0;

  OSMutexCreate(&mut_shield, "mutex for shield properties", &err);
  OSMutexCreate(&mut_laser, "mutex for laser properties", &err);
  OSMutexCreate(&mut_plat, "mutex for platform properties", &err);
  OSMutexCreate(&mut_hm, "mutex for hm mass properties", &err);
  OSMutexCreate(&mut_cap, "mutex for hm mass properties", &err);

  OSSemCreate(&sem_bt, "semaphore for buttons", 0, &err);
  OSSemCreate(&sem_phys, "semaphore for physics update", 0, &err);
  OSSemCreate(&sem_disp, "semaphore for display", 0, &err);
  OSSemCreate(&sem_lled, "semaphore for left led", 0, &err);
  OSSemCreate(&sem_rled, "semaphore for right led", 0, &err);
  OSSemCreate(&sem_cap, "semaphore for capsense", 0, &err);


  OSTmrCreate(&t_dis, "display timer", 0,dis_period, OS_OPT_TMR_PERIODIC, &tmr_cb, NULL, &err);
  OSTmrCreate(&s_tmr, "shield threshold timer", shield_settings.b_set.arm_window, 0, OS_OPT_TMR_ONE_SHOT, NULL, NULL, &err);  //replace 100 with time shield is active
  OSTmrCreate(&r_tmr, "recharge threshold timer", shield_settings.b_set.recharge_after_disarm, 0, OS_OPT_TMR_ONE_SHOT, NULL, NULL, &err);  //replace 100 with time shield is active
  OSTmrCreate(&t_phy, "physics update timer", 0,phy_period, OS_OPT_TMR_PERIODIC, &tmr_cb_phys, NULL, &err);
  OSTmrCreate(&t_rled, "right led timer", 0, rled_period, OS_OPT_TMR_PERIODIC, &tmr_cb_rled, NULL, &err);  //replace 100 with time shield is active
  OSTmrCreate(&t_lled, "left led timer", 0,lled_period, OS_OPT_TMR_PERIODIC, &tmr_cb_lled, NULL, &err);
  OSTmrCreate(&t_cap, "capsense timer", 0,cap_period, OS_OPT_TMR_PERIODIC, &tmr_cb_cap, NULL, &err);
  phy_period = phy_period/10;
}
void read_capsense(void)
{
  CAPSENSE_Sense();
  bool p0 = CAPSENSE_getPressed(0);
  bool p1 = CAPSENSE_getPressed(1);
  bool p2 = CAPSENSE_getPressed(2);
  bool p3 = CAPSENSE_getPressed(3);
  bool f_left = false;
  bool n_left = false;
  bool f_right = false;
  bool n_right = false;
  if(p0)
    {
      f_left = true;
    }
  if(p1)
    {
      n_left = true;
    }
  if(p2)
    {
      n_right = true;
    }
  if(p3)
    {
      f_right = true;
    }

  if(f_left)
    {
      in_force = -plat_settings.maxForce;
      return;
    }
  if(n_left)
    {
      in_force = -0.5*plat_settings.maxForce;
      return;
    }
  if(n_right)
    {
      in_force = 0.5*plat_settings.maxForce;
      return;
    }
  if(f_right)
    {
      in_force = plat_settings.maxForce;
      return;
    }
  in_force = 0;
  return;
}
void detect_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&tcb1,
               "detection task",
               detect_task,
               DEF_NULL,
               PRIORITY,
               &stack1[0],
               (STACK_SIZE / 10u),
               STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void phys_upd_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&tcb6,
               "physics update task",
               phys_upd_task,
               DEF_NULL,
               PRIORITY,
               &stack6[0],
               (STACK_SIZE / 10u),
               STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void lcd_init(void)
{
  RTOS_ERR err;
  uint32_t status;  //start of code obtained from example!

  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);
//end of code obtained from given example
  OSTaskCreate(&tcb5,
               "LCD display task",
               lcd_task,
               DEF_NULL,
               PRIORITY,
               &stack5[0],
               (STACK_SIZE / 10u),
               STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void lled_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&tcb3,
               "left led task",
               lled_task,
               DEF_NULL,
               PRIORITY,
               &stack3[0],
               (STACK_SIZE / 10u),
               STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void rled_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&tcb2,
               "right led task",
               rled_task,
               DEF_NULL,
               PRIORITY,
               &stack2[0],
               (STACK_SIZE / 10u),
               STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void cap_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&tcb7,
               "capsense task",
               capsense_task,
               DEF_NULL,
               19,
               &stack7[0],
               (STACK_SIZE / 10u),
               STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void capsense_task(void * arg)
{
  PP_UNUSED_PARAM(arg);
  RTOS_ERR err;
  CAPSENSE_Init();
  OSTmrStart(&t_cap, &err);
  while (1)
    {
      OSSemPend(&sem_cap, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      OSMutexPend(&mut_plat, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      read_capsense();
      if(in_force == 0)
        {
          GPIO_PinOutClear(LED1_port, LED1_pin);
        }
      OSMutexPost(&mut_plat, OS_OPT_POST_NONE, &err);
    }
}
void phys_upd_task(void *arg) //testing for bounce on one wall, platform.
{
  PP_UNUSED_PARAM(arg);
  RTOS_ERR err;
  OSTmrStart(&t_phy, &err);
  while (1)
  {
      OSSemPend(&sem_phys, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      OSMutexPend(&mut_shield, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      //checking shield and recharge timer
      if((OSTmrRemainGet(&r_tmr, &err) == 0) && (!shield_settings.b_set.recharged))
        {
          shield_settings.b_set.recharged = true;
          OSTmrStop(&r_tmr,OS_OPT_TMR_NONE, NULL, &err);
        }
      if((GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) == 1) && (OSTmrRemainGet(&s_tmr, &err) != 0) && ((OSTmrStateGet(&s_tmr, &err) == 2)))
        {
          shield_settings.press_valid = false;
          OSTmrStop(&s_tmr, OS_OPT_TMR_NONE, NULL, &err);
          shield_settings.activated = true;
        }
      else if((OSTmrRemainGet(&s_tmr, &err) == 0))
        {
          shield_settings.press_valid = false;
        }

      if((GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) == 1) && (shield_settings.prev_bt != 1) && (OSTmrStateGet(&r_tmr, &err) == 1) && (OSTmrRemainGet(&s_tmr, &err) != 0))
        {
          shield_settings.activated = true;
          shield_settings.prev_bt = 1;
        }
      else if((OSTmrRemainGet(&s_tmr, &err) == 0) && (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) == 1) && (shield_settings.prev_bt != 1) && (OSTmrRemainGet(&r_tmr, &err) == 0))
        {
          shield_settings.activated = true;
          shield_settings.prev_bt = 1;
        }
      else if(shield_settings.prev_bt != GPIO_PinInGet(BUTTON1_port, BUTTON1_pin))
        {
          shield_settings.prev_bt = GPIO_PinInGet(BUTTON1_port, BUTTON1_pin);
        }

      OSMutexPost(&mut_shield, OS_OPT_POST_NONE, &err);

      OSMutexPend(&mut_plat, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      if(plat_settings.cur_force != in_force) //checks to see if the force has been changed
        {
          plat_settings.cur_force = in_force;
          plat_settings.prev_same = false;
        }
      else
        {
          plat_settings.prev_same = true;
        }
      float a = in_force / plat_settings.mass;
      a = a*0.0001;
      if(in_force == 0)
        {
          plat_settings.x_velocity = 0;
        }
      plat_settings.x_velocity = (plat_settings.x_velocity + a*phy_period);
      if(plat_settings.x_velocity > 0)
        {
          plat_settings.pos = plat_settings.pos + plat_settings.x_velocity*phy_period + 0.5*a*phy_period*phy_period - FRICTION_CONST;
        }
      else if(plat_settings.x_velocity < 0)
        {
          plat_settings.pos = plat_settings.pos + plat_settings.x_velocity*phy_period + 0.5*a*phy_period*phy_period + FRICTION_CONST;
        }
      OSMutexPost(&mut_plat, OS_OPT_POST_NONE, &err);

      OSMutexPend(&mut_hm, 0, OS_OPT_PEND_BLOCKING, NULL, &err);

      //y velocity and position
      bool in_range = false;
      if(abs(h_settings.y_pos-plat_y_pos) <= 2.5)
        {
          if(hm_plat_impact(h_settings.x_pos, plat_settings.pos, plat_range))
            {
              if(h_settings.hm_bounce == false)
                {
                  in_range = true;
                  h_settings.hm_bounce = true;
                }
              if(h_settings.y_vel < shield_settings.minPerpSp)
                {
                  in_range = false;
                }
            }
        }
      if((in_range) && (h_settings.hm_bounce)) //func checks if HM's x-pos is in the range of the platform
          {
            if((OSTmrRemainGet(&s_tmr, &err) != 0) && (shield_settings.press_valid == true) && (shield_settings.b_set.recharged == true)) //checking for bouncing
              {
                OSTmrStop(&s_tmr,OS_OPT_TMR_NONE, NULL, &err);
                //float x_new = x_ke_upd(shield_settings.b_set.kin_inc, h_settings.x_vel, h_settings.y_vel, plat_settings.mass);
                float y_new = y_ke_upd(shield_settings.b_set.kin_inc, h_settings.x_vel, h_settings.y_vel, plat_settings.mass);
                //h_settings.x_vel = x_new;
                h_settings.y_vel = y_new;
                OSMutexPend(&mut_shield, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
                shield_settings.press_valid = false;
                shield_settings.b_set.recharged = false;
                shield_settings.activated = true;
                OSMutexPost(&mut_shield, OS_OPT_POST_NONE, &err);
                if(plat_settings.b_can.limited == true)
                  {
                    if(h_settings.y_vel > plat_settings.b_can.max_platbounce_speed)
                      {
                        h_settings.y_vel = plat_settings.b_can.max_platbounce_speed;
                      }
//                    if(h_settings.x_vel > plat_settings.b_can.max_platbounce_speed)
//                      {
//                        h_settings.x_vel = plat_settings.b_can.max_platbounce_speed;
//                      }
                  }
              }
            else
              {
                //float x_new = x_ke_upd(shield_settings.bnc_reduct, h_settings.x_vel, h_settings.y_vel, plat_settings.mass);
                float y_new = y_ke_upd(shield_settings.bnc_reduct, h_settings.x_vel, h_settings.y_vel, plat_settings.mass);
                //h_settings.x_vel = x_new;
                h_settings.y_vel = y_new; //gotta fix x dir after bounce off of platform

              }
            h_settings.y_vel = updVel(-h_settings.y_vel, gravity, phy_period); //need to update velocity before position, otherwise the hm will
            h_settings.y_pos = h_settings.y_pos + h_settings.y_vel*phy_period + 0.5*gravity*phy_period*phy_period;
            in_range = false;
            if(plat_settings.b_can.limited == true)
              {
                if(h_settings.y_vel > plat_settings.b_can.max_platbounce_speed)
                  {
                    h_settings.y_vel = plat_settings.b_can.max_platbounce_speed;
                  }
//                if(h_settings.x_vel > plat_settings.b_can.max_platbounce_speed)
//                  {
//                    h_settings.x_vel = plat_settings.b_can.max_platbounce_speed;
//                  }
              }
          }
      else
        {
          h_settings.y_pos = h_settings.y_pos + h_settings.y_vel*phy_period + 0.5*gravity*phy_period*phy_period;
          h_settings.y_vel = updVel(h_settings.y_vel, gravity, phy_period);
        }
      //here I will be setting the right led to blink on and off at 1Hz
      if((h_settings.y_pos > plat_y_pos) && (h_settings.alrt == false) && (abs(h_settings.y_pos - plat_y_pos) > 3.5))
        {
          h_settings.alrt = true;
          rled_period = 100; //need to double check if this value is right
          OSTmrSet(&t_rled, 0, rled_period, &tmr_cb_rled, NULL, &err);
          if((OSTmrStateGet(&t_rled, &err) == 1))
            {
              OSTmrStart(&t_rled, &err);
            }
        }
      //x position and velocity
      if(((abs(h_settings.x_pos - r_wall_pos) <= 1) || (abs(h_settings.x_pos - l_wall_pos) <= 1)) && ((plat_settings.b_can.enabled == true)))
        {
          h_settings.x_vel = -h_settings.x_vel;
          h_settings.x_pos = h_settings.x_pos + h_settings.x_vel*phy_period;
        }
      else
        {
          h_settings.x_pos = h_settings.x_pos + h_settings.x_vel*phy_period;
        }
      if(abs(h_settings.y_pos-plat_y_pos) >= 2.5)
        {
          h_settings.hm_bounce = false;
        }
      OSMutexPost(&mut_hm, OS_OPT_POST_NONE, &err);

      //Set Right LED PWM
      if(h_settings.alrt != true)
        {
          rled_period = calc_rled_pwm(plat_settings.mass, plat_settings.maxForce, base_rled_period, h_settings.x_pos, h_settings.y_pos, h_settings.x_vel, h_settings.y_vel, plat_settings.pos, plat_settings.x_velocity, plat_y_pos, gravity)*5;
          if(rled_period == 0 && (OSTmrStateGet(&t_rled, &err) == 2))
            {
              OSTmrStop(&t_rled,OS_OPT_TMR_NONE, NULL, &err);
              GPIO_PinOutClear(LED0_port, LED0_pin);
            }
          else if((rled_period != 0) && (OSTmrStateGet(&t_rled, &err) == 1))
            {
              OSTmrSet(&t_rled, 0, rled_period, &tmr_cb_rled, NULL, &err);
              OSTmrStart(&t_rled, &err);
            }
          else
            {
              if(rled_period != 0)
                {
                  OSTmrSet(&t_rled, 0, rled_period, &tmr_cb_rled, NULL, &err);
                }
            }
        }
      //Set Left LED PWM
      if(plat_settings.prev_same == false)
        {
          lled_period = abs(calc_lled_pwm(plat_settings.maxForce, plat_settings.cur_force, base_lled_period));
          if(lled_period == 0 && (OSTmrStateGet(&t_lled, &err) == 2))
            {
              OSTmrStop(&t_lled,OS_OPT_TMR_NONE, NULL, &err);
              GPIO_PinOutClear(LED1_port, LED1_pin);
            }
          else if((lled_period != 0) && (OSTmrStateGet(&t_lled, &err) == 1))
            {
              OSTmrSet(&t_lled, 0, lled_period, &tmr_cb_lled, NULL, &err);
              OSTmrStart(&t_lled, &err);
            }
          else
            {
              if(lled_period != 0)
                {
                  OSTmrSet(&t_lled, 0, lled_period, &tmr_cb_lled, NULL, &err);
                }
            }
        }

      //clamping (depending on functionality)
      if(plat_settings.pos+10 >= r_wall_pos)
        {
          plat_settings.pos = r_wall_pos - 10;
          plat_settings.x_velocity = 0;
        }
      else if(plat_settings.pos-10 <= l_wall_pos)
        {
          plat_settings.pos = l_wall_pos +10;
          plat_settings.x_velocity = 0;
        }
      if(h_settings.x_pos > r_wall_pos)
        {
          h_settings.x_pos = r_wall_pos;
        }
      else if(h_settings.x_pos < l_wall_pos)
        {
          h_settings.x_pos = l_wall_pos;
        }
      if(h_settings.y_pos < -5)
        {
          h_settings.ejected = true;
        }
      if((h_settings.y_pos > 150) && (l_settings.auto_ctrl == false))
        {
          g_set.game_over = true;
        }
      else if((h_settings.y_pos > 150) && (l_settings.auto_ctrl == true))
        {
          if(l_settings.numActivate > 0)
            {
              l_settings.activated = true;
            }
          else
            {
              g_set.game_over = true;
            }
        }
      if((OSTmrStateGet(&t_rled, &err) == 1) && (GPIO_PinInGet(LED0_port, LED0_pin) == 1))
        {
          GPIO_PinOutClear(LED0_port, LED0_pin);
        }
      if(plat_settings.auto_ctrl == true)
        {
          plat_settings.pos = h_settings.x_pos;
        }
  }
}
void detect_task(void *arg)
{
  PP_UNUSED_PARAM(arg);

  RTOS_ERR err;
  while (1)
  {
      OSSemPend(&sem_bt, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
      OSMutexPend(&mut_shield, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      OSMutexPend(&mut_laser, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
      int st_rt0 = pop(&(*b0_s));
      int st_rt1 = pop(&(*b1_s));
      if((GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) == 0) && (GPIO_PinInGet(BUTTON0_port, BUTTON0_pin) == 0))
        {
          bt0_b = false;
          bt1_b = false;
          g_set.game_over = true; //maybe need a mutex for this?, will check
        }
      if((st_rt0 == 0) && (bt0_b == true))
        {
          bt0_b = false;
          if(l_settings.numActivate > 0)
            {
              l_settings.activated = true;
            }
          l_settings.numActivate--;
          //OSFlagPost(&f_gp1, lsr_upd, OS_OPT_POST_FLAG_SET, &err);
          EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        }
      if((st_rt1 == 0) && (bt1_b == true))
        {
          bt1_b = false;
          if((OSTmrStateGet(&r_tmr, &err) != 2)) //2 == running
            {
              OSTmrStart(&s_tmr, &err);
              shield_settings.press_valid = true;
            }
          //OSFlagPost(&f_gp1, sh_upd, OS_OPT_POST_FLAG_SET, &err); //will need to look at these flags
          EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        }
      OSMutexPost(&mut_shield, OS_OPT_POST_NONE, &err);
      OSMutexPost(&mut_laser, OS_OPT_POST_NONE, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }

}

void lcd_task(void *arg)
{
  PP_UNUSED_PARAM(arg);

  RTOS_ERR err;
  OSTmrStart(&t_dis, &err);

  while (1)
  {
      OSSemPend(&sem_disp, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
      //draws hm
      GLIB_clear(&glibContext);
      if(g_set.game_over == true)
        {
          char string[100];
          sprintf(string, "You Lose!");
          const char * str1 = string;
          GLIB_drawStringOnLine(&glibContext, str1, currentLine, GLIB_ALIGN_LEFT, 45, 60, true);
        }
      else if(g_set.game_win == true)
        {
          char string[100];
          sprintf(string, "You Win!");
          const char * str1 = string;
          GLIB_drawStringOnLine(&glibContext, str1, currentLine, GLIB_ALIGN_LEFT, 45, 60, true);
        }
      else
        {
          //OSMutexPend(&mut_laser, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
          //OSMutexPend(&mut_hm, 0, OS_OPT_PEND_BLOCKING, NULL, &err);

          if(l_settings.activated == true)
          {
            h_settings.num--;
          }
          if(h_settings.ejected == true)
            {
              h_settings.num--;
            }
          if(h_settings.num <= 0)
            {
              g_set.game_win = true;
            }
          if(h_settings.num >= 0)
            {
              //reset x + y position and velocity
              if(l_settings.activated == true)
                {
                  l_settings.activated = false;
                  h_settings.x_pos = rand() % 90+2;
                  h_settings.y_pos = 0;
                  h_settings.x_vel = rand() % 10 - rand() % 15;
                  h_settings.y_vel = rand() % 10;
                }
              else if(h_settings.ejected == true)
                {
                  h_settings.ejected = false;
                  h_settings.x_pos = rand() % 90+2;
                  h_settings.y_pos = 0;
                  h_settings.x_vel = rand() % 10;
                  h_settings.y_vel = rand() % 10;
                }
              //OSMutexPost(&mut_shield, OS_OPT_POST_NONE, &err);
              //OSMutexPost(&mut_hm, OS_OPT_POST_NONE, &err);
            }
          if(shield_settings.activated == true && ((OSTmrRemainGet(&r_tmr, &err) == 0) || ((OSTmrStateGet(&r_tmr, &err) != 2))))
            {
              shield_settings.activated = false;
              OSTmrStart(&r_tmr, &err);
            }
          char string[100];
          int timegot = OSTmrRemainGet(&s_tmr, &err);
          sprintf(string, "S=%d", timegot);
          const char * str1 = string;
          GLIB_drawStringOnLine(&glibContext, str1, currentLine, GLIB_ALIGN_LEFT, 10, 120, true);
          char string1[100];
          int timegot1 = OSTmrRemainGet(&r_tmr, &err);
          sprintf(string1, "R=%d", timegot1);
          const char * str2 = string1;
          GLIB_drawStringOnLine(&glibContext, str2, currentLine, GLIB_ALIGN_LEFT, 85, 120, true);
          GLIB_drawCircleFilled(&glibContext,h_settings.x_pos, h_settings.y_pos,h_settings.disp_diameter);
          //draws platform
          GLIB_drawLineH(&glibContext, (plat_settings.pos-10),plat_y_pos,(plat_settings.pos+10));
          //draws the canyons
          GLIB_drawLineV(&glibContext, 2, 0, 130); //left wall
          GLIB_drawLineV(&glibContext, 125, 0, 130); //right wall

        }
      DMD_updateDisplay();
  }

}
void lled_task(void *arg) //LED1
{
  PP_UNUSED_PARAM(arg);

  RTOS_ERR err;
  //OSTmrStart(&t_lled, &err);

  while (1)
  {
      OSSemPend(&sem_lled, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      if(GPIO_PinInGet(LED1_port, LED1_pin) == 0)
        {
          GPIO_PinOutSet(LED1_port, LED1_pin);
        }
      else
        {
          GPIO_PinOutClear(LED1_port, LED1_pin);
        }
  }

}
void rled_task(void *arg) //LED0
{
  PP_UNUSED_PARAM(arg);

  RTOS_ERR err;
  OSTmrStart(&t_rled, &err);
  while (1)
  {
      OSSemPend(&sem_rled, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      if(GPIO_PinInGet(LED0_port, LED0_pin) == 0)
        {
          GPIO_PinOutSet(LED0_port, LED0_pin);
        }
      else
        {
          GPIO_PinOutClear(LED0_port, LED0_pin);
        }
  }

}
void idle_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&tcb4,
               "idle task",
               idle_task,
               DEF_NULL,
               IDLE_PRIORITY,
               &stack4[0],
               (STACK_SIZE / 10u),
               STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void idle_task(void * arg)
{
  PP_UNUSED_PARAM(arg);

  while (1)
  {
      EMU_EnterEM1();
  }
}

void tmr_cb(void * p_t,void * p_a)
{
  PP_UNUSED_PARAM(p_t);
  PP_UNUSED_PARAM(p_a);
  RTOS_ERR err;
  OSSemPost(&sem_disp, OS_OPT_POST_1, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  return;
}
void tmr_cb_phys(void * p_t,void * p_a)
{
  PP_UNUSED_PARAM(p_t);
  PP_UNUSED_PARAM(p_a);
  RTOS_ERR err;
  OSSemPost(&sem_phys, OS_OPT_POST_1, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  return;
}

void tmr_cb_rled(void * p_t,void * p_a)
{
  PP_UNUSED_PARAM(p_t);
  PP_UNUSED_PARAM(p_a);
  RTOS_ERR err;
  OSSemPost(&sem_rled, OS_OPT_POST_1, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  return;
}
void tmr_cb_lled(void * p_t,void * p_a)
{
  PP_UNUSED_PARAM(p_t);
  PP_UNUSED_PARAM(p_a);
  RTOS_ERR err;
  OSSemPost(&sem_lled, OS_OPT_POST_1, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  return;
}
void tmr_cb_cap(void * p_t,void * p_a)
{
  PP_UNUSED_PARAM(p_t);
  PP_UNUSED_PARAM(p_a);
  RTOS_ERR err;
  OSSemPost(&sem_cap, OS_OPT_POST_1, &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  return;
}

