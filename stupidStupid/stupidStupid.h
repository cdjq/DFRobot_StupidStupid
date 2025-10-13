#ifndef __STUPID_STUPID_H
#define __STUPID_STUPID_H

#include "ss_bluetoothCtl.h"

typedef struct {

  int   speedMax = 0;
  float speedRate = 0.0;    // 速度比例
  int   angleValue = 0;     // 角度值

  float spinSpeedRate = 0.0;    // 旋转比例
} sMatDat_t;

extern sMatDat_t matDat;

#endif
