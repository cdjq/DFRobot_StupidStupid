#include "ss_bluetoothCtl.h"
void _driveMotor(uint16_t directionAngle, float speed);
const char *bluetoothPaireCode = "20:00:00:01:15:08";    // 配对码

uint8_t speedFlag = 0;
uint8_t vocStage = 3;    // 默认速度3档

sPs3Dat_t ps3Dat;

void initBluetooth()
{
  Serial.begin(115200);

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin(bluetoothPaireCode);

#ifdef DEBUG_MODE
  Serial.println("Ready.");
#endif
}

bool loopBluetooth()
{
  setVelocity();
  if(!Ps3.isConnected()) {
    return true;
  }
  else {
    return false;
  }
}

void setVelocity()    // 速度档位设置
{
  if(speedFlag == 1) {
    speedFlag = 0;
    if(++vocStage == 4)
      vocStage = 0;
  }
  else {
    return;
  }

  switch(vocStage) {
    case 0:
      // 速度0 停止
      matDat.speedMax = 0;
      break;
    case 1:
      // 速度1 低
      matDat.speedMax = 100;
      break;
    case 2:
      // 速度2 中
      matDat.speedMax = 200;
      break;
    case 3:
      // 速度3 高
      matDat.speedMax = 300;
      break;
    default:
      matDat.speedMax = 0;
      break;
  }
#ifdef SS_BT_DEBUG
  Serial.print("Current speed MAX:");
  Serial.println(matDat.speedMax);
#endif
}

void notify()
{
  static uint8_t rockerFlag = 0;
  uint8_t        matFlag = 0;

  // if( Ps3.event.button_down.square )
  //     Serial.println("Started pressing the square button");
  if(Ps3.event.button_up.triangle) {
    speedFlag = 1;
  }

  if(Ps3.event.button_up.square) {
#ifdef SS_BT_DEBUG
    Serial.println("===========Left rocker==========");
#endif
    rockerFlag = 0;
  }
  if(Ps3.event.button_up.circle) {
#ifdef SS_BT_DEBUG
    Serial.println("==========Right rocker==========");
#endif
    rockerFlag = 1;
  }

  //---------------- Analog stick value events ---------------
  if((abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2) && rockerFlag == 0) {

    ps3Dat.lXdata = Ps3.data.analog.stick.lx;
    ps3Dat.lYdata = Ps3.data.analog.stick.ly;
    ps3Dat.lX = ps3Dat.lXdata;
    ps3Dat.lY = -ps3Dat.lYdata;

#ifdef SS_BT_DEBUG
    Serial.print("left Rocker x:");
    Serial.println(ps3Dat.lX);
    Serial.print("left Rocker Y:");
    Serial.println(ps3Dat.lY);
#endif
    matFlag = 1;
  }
  else {
    ps3Dat.lX = 0;
    ps3Dat.lY = 0;
  }

  if((abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2) && rockerFlag == 1) {

    ps3Dat.rX = Ps3.data.analog.stick.rx;

#ifdef SS_BT_DEBUG
    Serial.print("rightr Rocker x:");
    Serial.println(ps3Dat.rX);
#endif
    matFlag = 1;
  }
  else {
    ps3Dat.rX = 0;
  }

  if(matFlag != 0) {
    if(rockerFlag == 0) {
      float angle = atan2((float)ps3Dat.lY, (float)ps3Dat.lX);    // 计算角度弧度值
      if(angle < 0)
        angle += 2 * PI;                       // 转换为0~2PI范围
      float angleTemp = angle * 180.0 / PI;    // 转换为角度值
      matDat.angleValue = (int)angleTemp;

      float length = sqrt((float)(ps3Dat.lX * ps3Dat.lX + ps3Dat.lY * ps3Dat.lY));    // 计算摇杆偏移量长度
      if(length > 128.0)
        length = 128.0;                     // 限制最大值为128
      matDat.speedRate = length / 128.0;    // 计算速度比例，范围0~1

      matDat.spinSpeedRate = 0;
    }
    else {
      matDat.spinSpeedRate = (float)ps3Dat.rX / 128.0;    // 计算旋转速度比例，范围-1~1
      matDat.angleValue = 0;

      matDat.speedRate = 0;
    }
    pritnMatData();
  }
}

void onConnect()
{
#ifdef SS_BT_DEBUG
  Serial.println("Connected.");
#endif
}

void pritnMatData(void)
{
  // #ifdef SS_BT_DEBUG
  Serial.print("speedMax:");
  Serial.println(matDat.speedMax);
  Serial.print("speedRate:");
  Serial.println(matDat.speedRate);
  Serial.print("angleValue:");
  Serial.println(matDat.angleValue);
  Serial.print("spinSpeedRate:");
  Serial.println(matDat.spinSpeedRate);
  _driveMotor(matDat.angleValue, matDat.speedRate);
  // #endif
}
