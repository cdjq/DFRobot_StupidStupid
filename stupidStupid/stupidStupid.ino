#include <stdint.h>

#include "stupidStupid.h"
/************************************************
                  ↑  前方（Forward）
                  │
                  │
            ┌───────────────┐
            │               │
            │   M1       M2 │
            │ (左前)   (右前) │
            │               │
            │   M3       M4 │
            │ (左后)   (右后) │
            └───────────────┘
                  │
                  ▼
                  后方（Backward）
**************************************************/

// CRC-8/MAXIM 多项式 (反射后为 0x8C)
#define CRC8_POLY 0x8C
#define CRC8_INIT 0x00
sMatDat_t matDat;
// 计算一个字节的 CRC
uint8_t crc8_update(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for(uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x01) ? (crc >> 1) ^ 0x8C : crc >> 1;
  }
  return crc;
}

// 计算整个数据缓冲区的 CRC
uint8_t calculate_crc(uint8_t data[], uint8_t length)
{
  uint8_t crc = 0x00;
  for(int i = 0; i < length; i++) {
    crc = crc8_update(crc, data[i]);
  }
  return crc;
}

// 配置速度命令的函数

void configureSpeed(uint8_t deviceAddress, int16_t targetSpeed)    // 速度配置函数
{
  // 速度值转换（2100对应210rpm）
  uint8_t deviceId = deviceAddress + 1;    //  地址从 1 开始
  uint8_t command[100] = { 0 };
  int16_t speed = targetSpeed;
  if((deviceId % 2 == 0)) {
    speed *= -1;
  }

  command[0] = deviceId;
  command[1] = 0x64;
  command[2] = speed >> 8;
  command[3] = speed & 0xFF;
  for(int i = 4; i < 9; i++) command[i] = 0x00;
  command[6] = 2;
  command[9] = calculate_crc(command, 9);
  for(int i = 0; i < 10; i++) Serial1.write(command[i]);
  delay(2);
}

void _rotary(eDirection_t direction, float speed)    // 旋转编码器驱动函数
{
  uint16_t realSpeed = speedBase[speedLevel] * speed;
  uint16_t driveSpeed[4] = { 0 };
  if(direction == d_left) {
    driveSpeed[LF] = -1 * realSpeed;
    driveSpeed[LR] = -1 * realSpeed;
    driveSpeed[RF] = realSpeed;
    driveSpeed[RR] = realSpeed;
  }
  else if(direction == d_right) {
    driveSpeed[LF] = realSpeed;
    driveSpeed[LR] = realSpeed;
    driveSpeed[RF] = -1 * realSpeed;
    driveSpeed[RR] = -1 * realSpeed;
  }
  configureSpeed(LF, driveSpeed[LF]);
  configureSpeed(RF, driveSpeed[RF]);
  configureSpeed(LR, driveSpeed[LR]);
  configureSpeed(RR, driveSpeed[RR]);
}

void _driveMotor(uint16_t directionAngle, float speed)    // 电机驱动函数
{
  uint16_t realSpeed = speedBase[speedLevel] * speed;
  uint16_t realAccelerated = acceleratedBase[speedLevel];
  uint16_t driveSpeed[4] = { 0 };
  // 右侧正转并且减速
  if(directionAngle >= 0 && directionAngle <= 90) {
    int16_t degree = (directionAngle - 90) * -1;
    float   subRight = 1.0f - directionAngle / 90.0;
    if(directionAngle == 90) {
      driveSpeed[RF] = driveSpeed[RR] = realSpeed * 0.1;
    }
    else {
      driveSpeed[RF] = driveSpeed[RR] = realSpeed * subRight;
    }
    driveSpeed[LF] = driveSpeed[LR] = realSpeed;
  }

  // 右侧反转并且减速
  if(directionAngle > 90 && directionAngle <= 180) {
    int16_t degree = (directionAngle - 180) * -1;
    float   subRight = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = -1 * realSpeed * subRight;
    driveSpeed[LF] = driveSpeed[LR] = -1 * realSpeed;
  }

  // 左侧正转并减速
  if(directionAngle > 270 && directionAngle <= 360) {
    int16_t degree = (directionAngle - 360) * -1;
    float   subLeft = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = realSpeed;
    driveSpeed[LF] = driveSpeed[LR] = realSpeed * subLeft;
  }

  // 左侧反转并减速
  if(directionAngle > 180 && directionAngle <= 270) {
    int16_t degree = (directionAngle - 270 + 90);
    float   subLeft = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = -1 * realSpeed;
    ;
    if(directionAngle == 270) {
      driveSpeed[LF] = driveSpeed[LR] = -1 * realSpeed * 0.1;
    }
    else {
      driveSpeed[LF] = driveSpeed[LR] = -1 * realSpeed * subLeft;
    }
  }

  // Serial.print("LF:");
  // Serial.print(driveSpeed[LF]);
  // Serial.print("RF:");
  // Serial.print(driveSpeed[RF]);
  // Serial.print("LR:");
  // Serial.print(driveSpeed[LR]);
  // Serial.print("RR:");
  // Serial.print(driveSpeed[RR]);

  configureSpeed(LF, driveSpeed[LF]);
  configureSpeed(RF, driveSpeed[RF]);
  configureSpeed(LR, driveSpeed[LR]);
  configureSpeed(RR, driveSpeed[RR]);
}

void setup()
{
  // 启动串口 0，用于调试
  Serial.begin(115200);

  // 启动串口 1，设置波特率为 9600
  Serial1.begin(115200, SERIAL_8N1, 16, 17);    // 参数：波特率、数据格式、RX 引脚、TX 引脚
  delay(100);                                   // 等待串口初始化完成
  memset(testCmd, 0, CMD_MAX_LEN);
  // 打印调试信息到串口 0
  Serial.println("Sending 'Hello' via UART0...");
  Serial1.println("Sending 'Hello' via UART1...");

  initBluetooth();
  Ps3.setPlayer(speedLevel);
}

void loop()
{
  if(loopBluetooth()) {
    return;
  }
}
