#ifndef __SS_BLUETOOTH_CTL_H
#define __SS_BLUETOOTH_CTL_H

#include "Arduino.h"
#include <Ps3Controller.h>
#include "stupidStupid.h"
#include <math.h>

#define SS_BT_DEBUG //调试开关


/*
左摇杆坐标矢量(leftRockerVecX,leftRockerkVecY),这俩模拟值范围为[-128,127]
原始摇杆数据
          |Y
          |
    (-,-) |    (+,-)   
          |
----------|---------->X
          |
    (-,+) |     (+,+)
          |\
已映射为正规坐标系
*/
typedef struct{
  int16_t lX = 0;//左摇杆数据
  int16_t lY = 0;
  int16_t lXdata = 0;//左摇杆原始数据
  int16_t lYdata = 0;

    int16_t rX = 0;//右摇杆数据
  int16_t rXdata = 0;//右摇杆原始数据

}sPs3Dat_t;

extern sPs3Dat_t ps3Dat;

/**
 * @brief 蓝牙&ps3初始化
 */
void initBluetooth();

/**
 * @brief 蓝牙&ps3循环
 * @n 需要在loop()中调用，并判断返回值
 * @return true 未连接
 * @return false 已连接
 */
bool loopBluetooth();

/**
 * @brief 连接回调
 */
void onConnect();

/**
 * @brief 手柄事件通知回调
*/
void notify();

/**
 * @brief 速度档位设置
*/
void setVelocity();

/**
 * @brief 打印matDat数据
*/
void pritnMatData(void);

#endif
