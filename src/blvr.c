// Copyright (c) 2010-2016 The YP-Spur Authors, except where otherwise indicated.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <errno.h>
#include <pthread.h>
#include <math.h>

#include <blvr.h>
#include <param.h>
#include <yprintf.h>
#include <odometry.h>
#include <command.h>
#include <control.h>
#include <ssm_spur_handler.h>
#include <utility.h>

modbus_t *g_ctx;

BlvrDrivePacket g_blvr_drive_packet;
BlvrOdometryPacket g_blvr_odometry_packet;

/*Modbus RTU ID Share Write 
  45: Direct Drive Mode
  62: R-IN
  47: Speed [step/s]
  51: Trigger
  48: Acc [step/s^2]
  49: Dec [step/s^2]
  192: Alarm Reset
*/

/*Modbus RTU ID Share Read 
  77: Position [step]
  80: Speed [step/s]
  107: Torqe [0.1%]
  164: Power Sorce Voltage [0.1V]
  63: R-OUT
  64: Alarm
*/

void blbr_decode_odometry_data(BlvrOdometryPacket *src){
  src->leftMotorPosition = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 0);
  src->leftMotorSpeed = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 2);
  src->leftMotorTorque = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 4);
  src->leftMotorVolt = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 6);
  src->leftStatus = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 8);
  src->rightMotorPosition = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 11);
  src->rightMotorSpeed = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 13);
  src->rightMotorTorque = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 15);
  src->rightMotorVolt = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 17);
  src->rightStatus = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 19);
}

void blbr_decode_alarm_data(BlvrOdometryPacket *src){
  src->leftAlarm = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 0);
  src->rightAlarm = MODBUS_GET_INT32_FROM_INT16(src->tab_uint16, 3);
}

void blbr_encode_drive_data(BlvrDrivePacket *src){
  src->tab_uint16[0] = src->tab_uint16[6] = 0x0000;
  src->tab_uint16[1] = src->tab_uint16[7] = BLVR_SON;
  if(src->motorFree){
    src->tab_uint16[1] |= BLVR_FREE;
    src->tab_uint16[7] |= BLVR_FREE;
  }
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 2, src->leftMotorSpeed);
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 4, src->leftMotorTrigger);
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 8, src->rightMotorSpeed);
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 10, src->rightMotorTrigger);
}

void blbr_encode_acc_and_dec(BlvrDrivePacket *src){
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 0, src->leftMotorAcc);
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 2, src->leftMotorDec);
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 4, src->rightMotorAcc);
  MODBUS_SET_INT32_TO_INT16(src->tab_uint16, 6, src->rightMotorDec);
}

void blbr_set_rs485_timeout(uint16_t timeout_msec){
  g_blvr_drive_packet.tab_uint16[4] = g_blvr_drive_packet.tab_uint16[10] = timeout_msec;
}

void blbr_get_alarm(void){
  modbus_read_registers(g_ctx, 0x000A, 6, g_blvr_odometry_packet.tab_uint16);
  blbr_decode_alarm_data(&g_blvr_odometry_packet);
}

int blbr_isfree(void){
  if( g_blvr_odometry_packet.leftStatus & BLVR_FREE || g_blvr_odometry_packet.rightStatus & BLVR_FREE)
    return 1;
  else
    return 0;
}

void blbr_reser_alarm(uint16_t error_id){
  g_blvr_drive_packet.tab_uint16[0] = g_blvr_drive_packet.tab_uint16[2] = 0x0000;
  g_blvr_drive_packet.tab_uint16[1] = g_blvr_drive_packet.tab_uint16[3] = error_id;
  modbus_write_registers(g_ctx, 0x000C, 4, g_blvr_drive_packet.tab_uint16);
}

int blvr_connect(char *device_name)
{
  g_ctx = modbus_new_rtu(device_name, BLVR_DEFAULT_BAUDRATE, 'E', 8, 1);
  if (g_ctx == NULL) {
      yprintf(OUTPUT_LV_ERROR, "Unable to create the libmodbus context\n");
      return -1;
  }

  if (modbus_set_response_timeout(g_ctx, 0, 200000)== -1) {
      yprintf(OUTPUT_LV_ERROR, "Set timeout failed: %s\n", modbus_strerror(errno));
      modbus_free(g_ctx);
      return -1;
  }

  if (modbus_connect(g_ctx) == -1) {
      yprintf(OUTPUT_LV_ERROR, "Connection failed: %s\n", modbus_strerror(errno));
      modbus_free(g_ctx);
      return -1;
  }

  modbus_set_slave(g_ctx, 5);

  return 1;
}

int blvr_close(void)
{
  modbus_close(g_ctx);
  modbus_free(g_ctx);
}

int blvr_write_and_read(void)
{
  int ret;
  blbr_set_rs485_timeout(1000);
  ret = modbus_write_and_read_registers(g_ctx, 0x0002, 12, g_blvr_drive_packet.tab_uint16, 0x0000, 22, g_blvr_odometry_packet.tab_uint16);
  if(ret < 0) fprintf(stderr, "Error: %s\n", modbus_strerror(errno));
  return ret;
}

void blvr_init_odomety_data_packet(void)
{
  g_blvr_odometry_packet.leftMotorPosition =
  g_blvr_odometry_packet.prev_leftMotorPosition =
  g_blvr_odometry_packet.rightMotorPosition =
  g_blvr_odometry_packet.prev_rightMotorPosition = 0;
  g_blvr_odometry_packet.flagPosInit = 0;
}

void blvr_init_drive_data_packet(void)
{
  g_blvr_drive_packet.leftMotorSpeed =
  g_blvr_drive_packet.rightMotorSpeed =
  g_blvr_drive_packet.motorFree = 0;
}

/* オドメトリ計算 */
void blvr_odometry(OdometryPtr xp, double dt, double time)
{
  double v, w;
  double wvel[2], wvolt[2], mvel[2];
  double mtorque[2], wtorque[2];
  double torque_trans, torque_angular;
  int32_t cnt[2],enc[2];
  Parameters *param;
  param = get_param_ptr();

  cnt[0] = g_blvr_odometry_packet.leftMotorPosition;
  cnt[1] = g_blvr_odometry_packet.rightMotorPosition;
  enc[0] = g_blvr_odometry_packet.prev_leftMotorPosition;
  enc[1] = g_blvr_odometry_packet.prev_rightMotorPosition;
  mtorque[0] = (double)g_blvr_odometry_packet.leftMotorTorque * 0.1 *p(YP_PARAM_TORQUE_MAX, 0);
  mtorque[1] = (double)g_blvr_odometry_packet.rightMotorTorque * 0.1 *p(YP_PARAM_TORQUE_MAX, 0);
  wvolt[0] = (double)g_blvr_odometry_packet.leftMotorVolt * 0.1;
  wvolt[1] = (double)g_blvr_odometry_packet.rightMotorVolt * 0.1;
  int i;
  for (i = 0; i < 2; i++)
  {
    int32_t cnt_diff;
    {
      cnt_diff = cnt[i] - enc[i];
      if (!g_blvr_odometry_packet.flagPosInit)
      {
        cnt_diff = 0;
        dt = 1;
      }
    }
    /* 角速度計算 */
    mvel[i] = 2.0 * M_PI *
              ((double)cnt_diff) * pow(2, p(YP_PARAM_ENCODER_DIV, i)) /
              (p(YP_PARAM_COUNT_REV, i) * dt);
    wvel[i] = mvel[i] / p(YP_PARAM_GEAR, i);

    /* トルク推定 */
    /* 摩擦補償の補償 */
    if (wvel[i] > 0)
    {
      mtorque[i] -= p(YP_PARAM_TORQUE_NEWTON, i) + p(YP_PARAM_TORQUE_VISCOS, i) * fabs(mvel[i]);
    }
    else if (wvel[i] < 0)
    {
      mtorque[i] += p(YP_PARAM_TORQUE_NEWTON_NEG, i) + p(YP_PARAM_TORQUE_VISCOS_NEG, i) * fabs(mvel[i]);
    }
    wtorque[i] = mtorque[i] * p(YP_PARAM_GEAR, i);
  }
  if (!g_blvr_odometry_packet.flagPosInit)
  {
    g_blvr_odometry_packet.flagPosInit = 1;
  }
  g_blvr_odometry_packet.prev_leftMotorPosition = g_blvr_odometry_packet.leftMotorPosition;
  g_blvr_odometry_packet.prev_rightMotorPosition = g_blvr_odometry_packet.rightMotorPosition;

  /* キネマティクス計算 */
  v = p(YP_PARAM_RADIUS, MOTOR_RIGHT) * wvel[MOTOR_RIGHT] / 2.0 + p(YP_PARAM_RADIUS, MOTOR_LEFT) * wvel[MOTOR_LEFT] / 2.0;
  w = +p(YP_PARAM_RADIUS, MOTOR_RIGHT) * wvel[MOTOR_RIGHT] / p(YP_PARAM_TREAD, 0) - p(YP_PARAM_RADIUS, MOTOR_LEFT) * wvel[MOTOR_LEFT] / p(YP_PARAM_TREAD, 0);

  torque_trans = wtorque[MOTOR_RIGHT] / p(YP_PARAM_RADIUS, MOTOR_RIGHT) + wtorque[MOTOR_LEFT] / p(YP_PARAM_RADIUS, MOTOR_LEFT);
  torque_angular = (+wtorque[MOTOR_RIGHT] / p(YP_PARAM_RADIUS, MOTOR_RIGHT) - wtorque[MOTOR_LEFT] / p(YP_PARAM_RADIUS, MOTOR_LEFT)) * p(YP_PARAM_TREAD, 0) / 2;

  /* オドメトリ計算 */
  xp->x = xp->x + v * cos(xp->theta) * dt;
  xp->y = xp->y + v * sin(xp->theta) * dt;
  xp->theta = xp->theta + w * dt;
  xp->time = time;
  if (option(OPTION_SHOW_TIMESTAMP))
  {
    static int cnt = 0;
    if (++cnt % 100 == 0)
      printf("now - stamp: %0.3f ms\n", (get_time() - time) * 1000.0);
  }
  xp->v = v;
  xp->w = w;
  for (i = 0; i < YP_PARAM_MAX_MOTOR_NUM; i++)
  {
    if (!param->motor_enable[i])
      continue;
    xp->wvel[i] = wvel[i];
    xp->wvolt[i] = wvolt[i];
    xp->wang[i] = xp->wang[i] + xp->wvel[i] * dt;
    xp->wtorque[i] = wtorque[i];
  }
  xp->torque_trans = torque_trans;
  xp->torque_angular = torque_angular;

  /* FS座標系セット */
  CS_set(get_cs_pointer(CS_FS), xp->x, xp->y, xp->theta);

  // 数式指定のパラメータを評価
  param_calc();
}

void blvr_son(void)
{
  g_blvr_drive_packet.tab_uint16[0] = g_blvr_drive_packet.tab_uint16[4] = 0x0000;//Drive Mode: Direct Data Mode
  g_blvr_drive_packet.tab_uint16[1] = g_blvr_drive_packet.tab_uint16[5] = 0x0010;//Drive Mode: Direct Data Mode
  g_blvr_drive_packet.tab_uint16[2] = g_blvr_drive_packet.tab_uint16[6] = 0x0000;//R-OUT: None
  g_blvr_drive_packet.tab_uint16[3] = g_blvr_drive_packet.tab_uint16[7] = 0x0001;//R-OUT: S-ON
  // Wite 4 registers from address 0 of server.
  modbus_write_registers(g_ctx, 0x0000, 8, g_blvr_drive_packet.tab_uint16);
}

void blvr_set_wheelaccdec(void)
{
  SpurUserParamsPtr spur;
  spur = get_spur_user_param_ptr();
  int32_t d;
  
  d = (int32_t)(p(YP_PARAM_MAX_VEL, 0) * p(YP_PARAM_GEAR, 0) * p(YP_PARAM_COUNT_REV, 0) / (2 * M_PI * p(YP_PARAM_RADIUS, 0)));
  g_blvr_drive_packet.leftMotorAcc = g_blvr_drive_packet.leftMotorDec = g_blvr_drive_packet.rightMotorAcc = g_blvr_drive_packet.rightMotorDec = d;
  blbr_encode_acc_and_dec(&g_blvr_drive_packet);
  modbus_write_registers(g_ctx, 0x0008, 8, g_blvr_drive_packet.tab_uint16);
}

void blvr_set_wheelvel(void)
{
  SpurUserParamsPtr spur;
  spur = get_spur_user_param_ptr();

  pthread_mutex_lock(&spur->mutex);
  g_blvr_drive_packet.leftMotorSpeed = (int32_t)(spur->wheel_vel_smooth[0] * p(YP_PARAM_GEAR, 0) * p(YP_PARAM_COUNT_REV, 0) / (2 * M_PI));
  g_blvr_drive_packet.rightMotorSpeed = (int32_t)(spur->wheel_vel_smooth[1] * p(YP_PARAM_GEAR, 1) * p(YP_PARAM_COUNT_REV, 1) / (2 * M_PI));
  if(spur->run_mode == RUN_FREE)
    g_blvr_drive_packet.motorFree = 1; 
  else
    g_blvr_drive_packet.motorFree = 0;
  pthread_mutex_unlock(&spur->mutex);
  g_blvr_drive_packet.leftMotorTrigger = g_blvr_drive_packet.rightMotorTrigger = 1;
}

void blvr_control_loop_cleanup(void *data)
{
  yprintf(OUTPUT_LV_INFO, "Trajectory control loop stopped.\n");
}

/* 20msごとの割り込みで軌跡追従制御処理を呼び出す */
void blvr_control_loop(void)
{
  OdometryPtr odometry;
  SpurUserParamsPtr spur;

  odometry = get_odometry_ptr();
  spur = get_spur_user_param_ptr();

  yprintf(OUTPUT_LV_INFO, "Trajectory control loop started.\n");
  pthread_cleanup_push(blvr_control_loop_cleanup, NULL);
  int request;
  request = (p(YP_PARAM_CONTROL_CYCLE, 0) * 1000000);

  while (1)
  {
    yp_usleep(request);
    coordinate_synchronize(odometry, spur);
    run_control(*odometry, spur);
    blvr_set_wheelvel();
    // スレッドの停止要求チェック
    pthread_testcancel();
  }
  pthread_cleanup_pop(1);
}

void blvr_init_control_thread(pthread_t *thread)
{
  if (pthread_create(thread, NULL, (void *)blvr_control_loop, NULL) != 0)
  {
    yprintf(OUTPUT_LV_ERROR, "Can't create blvr_control_loop thread\n");
  }
}

int blvr_odometry_receive_loop(void)
{
  OdometryPtr odom;
  double time,prev_time,dt;
  int ret=1;

  odom = get_odometry_ptr();
  blvr_init_odomety_data_packet();
  blvr_init_drive_data_packet();
  blbr_get_alarm();
  if(g_blvr_odometry_packet.leftAlarm != 0 || g_blvr_odometry_packet.rightAlarm != 0)
  {
    ret = -1;
    if(g_blvr_odometry_packet.leftAlarm == BLVR_RS485_TIMEOUT && g_blvr_odometry_packet.rightAlarm == BLVR_RS485_TIMEOUT)
    {
      blbr_reser_alarm(BLVR_RS485_TIMEOUT);
      yprintf(OUTPUT_LV_WARNING, "Warning: Reset Blvr_alarm %0x.\n",BLVR_RS485_TIMEOUT);
      ret = 1;
    }
    else
    {
      yprintf(OUTPUT_LV_ERROR, "Error: Unknown blvr_alarm %0x %0x\n",g_blvr_odometry_packet.leftAlarm, g_blvr_odometry_packet.rightAlarm);
    }
  }
  blvr_son();
  blvr_set_wheelaccdec();
  yp_usleep(1000000);
  while (ret>0)
  {
    blbr_encode_drive_data(&g_blvr_drive_packet);
    time = get_time();
    ret = blvr_write_and_read();
    if(ret<0){
      yprintf(OUTPUT_LV_ERROR, "Error: blvr_write_and_read()\n");
      break;
    }
    blbr_decode_odometry_data(&g_blvr_odometry_packet);    
    dt = time - prev_time;
    blvr_odometry(odom,dt,time);
    prev_time = time;
    yp_usleep(5000);
  }

  return 1;
}
