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

#ifndef BLVR_H
#define BLVR_H

#include <modbus/modbus.h>

#define BLVR_DEFAULT_BAUDRATE 230400
#define BLVR_RS485_TIMEOUT 0x0085
#define BLVR_FREE 0x00000040
#define BLVR_SON 0x00000001

typedef struct _blbr_drive_packet {
  uint16_t tab_uint16[16];
  int32_t leftMotorSpeed;// [step/s]
  int32_t leftMotorTrigger;
  int32_t leftMotorAcc;// [step/s^2]
  int32_t leftMotorDec;// [step/s^2]
  int32_t rightMotorSpeed;// [step/s]
  int32_t rightMotorTrigger;
  int32_t rightMotorAcc;// [step/s^2]
  int32_t rightMotorDec;// [step/s^2]
  int motorFree;
} BlvrDrivePacket;

typedef struct _blbr_odometry_packet {
  uint16_t tab_uint16[22];// datanum +2
  int32_t leftMotorPosition;// [step]
  int32_t prev_leftMotorPosition;// [step]
  int32_t leftMotorSpeed;// [step/s]
  int32_t leftMotorTorque;// [1=0.1% Rated torque]
  int32_t leftMotorVolt;// [1=0.1 V]
  int32_t leftAlarm;
  int32_t leftStatus;
  int32_t rightMotorPosition;// [step]
  int32_t prev_rightMotorPosition;// [step]
  int32_t rightMotorSpeed;// [step/s]
  int32_t rightMotorTorque;// [1=0.1% Rated torque]
  int32_t rightMotorVolt;// [1=0.1 V]
  int32_t rightAlarm;
  int32_t rightStatus;
  int flagPosInit;
} BlvrOdometryPacket;

int blvr_connect(char *device_name);
int blvr_close(void);

void blvr_init_control_thread(pthread_t *thread);
int blvr_odometry_receive_loop(void);
#endif  // BLVR_H
