#include <float.h>
#include <math.h>
#include <stdint.h>

#include "stupidStupid.h"

#define PI               3.1415926
#define WHEEL_DIAMETER_M 0.05    // 轮胎直径 5 cm = 0.05 m

// 由线速度 (m/s) 转换为转速 (RPM)
int speed_to_rpm(double v_m_per_s)
{
  double circumference = PI * WHEEL_DIAMETER_M;    // 轮胎周长
  double rpm           = (v_m_per_s * 60.0) / circumference;
  return (int)rpm;
}

// c: 轮距[m]；rpm_c: 基准轮速；R: 期望转弯半径[m]（∞ 直行）
// 会自动把同侧前后轮设为相同
void mixer_from_radius(float c, float R, float rpm_c, uint8_t flag)
{
  float alpha = (isfinite(R) && fabsf(R) > 1e-6f) ? (c / (2.0f * R)) : 0.0f;
  float rpmL  = rpm_c * (1.0f - alpha);
  float rpmR  = rpm_c * (1.0f + alpha);

  int llll = speed_to_rpm(rpmL);
  int rrrr = speed_to_rpm(rpmR);
  // Serial.print("rpml = ");
  // Serial.println(rpmL);
  // Serial.print("rpmr = ");
  // Serial.println(rpmR);
  // Serial.print("llllll = ");
  // Serial.println(llll);
  // Serial.print("rrrr = ");
  // Serial.println(rrrr);
  if (flag == 0) {
    configureSpeed(LF, llll);
    configureSpeed(RF, rrrr);
    configureSpeed(LR, llll);
    configureSpeed(RR, rrrr);
  } else {
    configureSpeed(LF, rrrr);
    configureSpeed(RF, llll);
    configureSpeed(LR, rrrr);
    configureSpeed(RR, llll);
  }
}

// CRC-8/MAXIM 多项式 (反射后为 0x8C)
#define CRC8_POLY 0x8C
#define CRC8_INIT 0x00
sMatDat_t matDat;
// 计算一个字节的 CRC
uint8_t crc8_maxim_update(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x01)
      crc = (crc >> 1) ^ CRC8_POLY;
    else
      crc >>= 1;
  }
  return crc;
}
// 计算整个数据缓冲区的 CRC
uint8_t crc8_maxim_compute(const uint8_t *data, uint32_t len)
{
  uint8_t crc = CRC8_INIT;
  for (uint32_t i = 0; i < len; i++) {
    crc = crc8_maxim_update(crc, data[i]);
  }
  return crc;
}

void motorInit(void)
{
  uint8_t sendBuffer[10] = { 0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
  for (uint8_t j = 1; j <= 4; j++) {
    sendBuffer[0] = j;
    for (uint8_t i = 0; i < 10; i++) {
      Serial1.write(sendBuffer[i]);
    }
    delay(5);
  }
}

// 配置速度命令的函数
void configureSpeed(uint8_t deviceAddress, int16_t targetSpeed)
{
  uint8_t deviceId = deviceAddress + 1;    //  地址从 1 开始
  int16_t speed    = targetSpeed;
  if ((deviceId % 2 == 0)) {
    speed *= -1;
  }
  uint8_t sendPackage[20] = { 0 };
  sendPackage[0]          = deviceId;
  sendPackage[1]          = 0x64;
  sendPackage[2]          = speed >> 8;
  sendPackage[3]          = speed;
  sendPackage[4]          = 0;
  sendPackage[5]          = 0;
  sendPackage[6]          = 0;
  sendPackage[7]          = 0;
  sendPackage[8]          = 0;
  sendPackage[9]          = crc8_maxim_compute(sendPackage, 9);
  for (uint8_t i = 0; i < 10; i++) {
    Serial1.write(sendPackage[i]);
  }
  delay(5);
}

void _rotary(eDirection_t direction, float speed)
{
  uint16_t realSpeed     = speedBase[speedLevel] * speed;
  uint16_t driveSpeed[4] = { 0 };
  if (direction == d_left) {
    driveSpeed[LF] = -1 * realSpeed;
    driveSpeed[LR] = -1 * realSpeed;
    driveSpeed[RF] = realSpeed;
    driveSpeed[RR] = realSpeed;
  } else if (direction == d_right) {
    driveSpeed[LF] = realSpeed;
    driveSpeed[LR] = realSpeed;
    driveSpeed[RF] = -1 * realSpeed;
    driveSpeed[RR] = -1 * realSpeed;
  }
  configureSpeed(LF, driveSpeed[LF]);
  configureSpeed(RF, driveSpeed[RF]);
  configureSpeed(LR, driveSpeed[LR]);
  configureSpeed(RR, driveSpeed[RR]);
#if 0
  uint16_t realSpeed     = speedBase[speedLevel] * speed;
  uint16_t driveSpeed[4] = { 0 };
  if (speedLevel == 0) {
    mixer_from_radius(0.38, 0.3, 0.1, (uint8_t)direction);
  } else if (speedLevel == 1) {
    mixer_from_radius(0.38, 0.3, 0.2, (uint8_t)direction);
  } else if (speedLevel == 2) {
    mixer_from_radius(0.38, 0.3, 0.3, (uint8_t)direction);
  } else if (speedLevel == 3) {
    mixer_from_radius(0.38, 0.3, 0.4, (uint8_t)direction);
  }
  if (speed > -0.00001 && speed < 0.00001) {
    configureSpeed(LF, 0);
    configureSpeed(RF, 0);
    configureSpeed(LR, 0);
    configureSpeed(RR, 0);
  }
#endif
}

void _driveMotor(uint16_t directionAngle, float speed)    // 电机驱动函数
{
  uint16_t realSpeed       = speedBase[speedLevel] * speed;
  uint16_t realAccelerated = acceleratedBase[speedLevel];
  uint16_t driveSpeed[4]   = { 0 };
  // 右侧正转并且减速
  if (directionAngle >= 0 && directionAngle <= 90) {
    int16_t degree   = (directionAngle - 90) * -1;
    float   subRight = 1.0f - directionAngle / 90.0;
    if (directionAngle == 90) {
      driveSpeed[RF] = driveSpeed[RR] = realSpeed * 0.1;
    } else {
      driveSpeed[RF] = driveSpeed[RR] = realSpeed * subRight;
    }
    driveSpeed[LF] = driveSpeed[LR] = realSpeed;
  }

  // 右侧反转并且减速
  if (directionAngle > 90 && directionAngle <= 180) {
    int16_t degree   = (directionAngle - 180) * -1;
    float   subRight = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = -1 * realSpeed * subRight;
    driveSpeed[LF] = driveSpeed[LR] = -1 * realSpeed;
  }

  // 左侧正转并减速
  if (directionAngle > 270 && directionAngle <= 360) {
    int16_t degree  = (directionAngle - 360) * -1;
    float   subLeft = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = realSpeed;
    driveSpeed[LF] = driveSpeed[LR] = realSpeed * subLeft;
  }

  // 左侧反转并减速
  if (directionAngle > 180 && directionAngle <= 270) {
    int16_t degree  = (directionAngle - 270 + 90);
    float   subLeft = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = -1 * realSpeed;
    ;
    if (directionAngle == 270) {
      driveSpeed[LF] = driveSpeed[LR] = -1 * realSpeed * 0.1;
    } else {
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
  Serial1.begin(115200, SERIAL_8N1, 16, 17);    // 参数：波特率、数据格式、RX 引脚、TX 引脚
  delay(100);
  motorInit();

  // 等待串口初始化完成
  delay(1000);
  memset(testCmd, 0, CMD_MAX_LEN);
  // 打印调试信息到串口 0
  Serial.println("Sending 'Hello' via UART0...");
  Serial1.println("Sending 'Hello' via UART1...");
  initBluetooth();
  Ps3.setPlayer(speedLevel);
}

void loop()
{
  if (loopBluetooth()) {
    return;
  }
}
