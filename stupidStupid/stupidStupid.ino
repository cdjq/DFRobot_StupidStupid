#include "crc16_modbus.h"
#include "stupidStupid.h"

sMatDat_t matDat;

#define CMD_MAX_LEN 120
uint8_t  testCmd[CMD_MAX_LEN] = { 0 };
uint16_t cmdCount = 0;

// 配置速度命令的函数
void configureSpeed(uint8_t deviceAddress, uint8_t packetNumber, int32_t targetSpeed, int32_t acceleration)
{
  // 构建命令包
  uint8_t sendBuffer[20] = { 0 };

  // 协议头
  sendBuffer[0] = 0xAE;

  // 包序号
  sendBuffer[1] = packetNumber;

  // 设备地址
  sendBuffer[2] = deviceAddress;
  if((deviceAddress % 2) != 0) {
    targetSpeed *= -1;
  }
  // 命令码
  sendBuffer[3] = 0x21;

  // 数据包长度 (目标速度 + 加速度 = 8 字节)
  sendBuffer[4] = 0x08;

  // 目标速度 (4字节，单位为 0.01RPM)
  sendBuffer[8] = (targetSpeed >> 24) & 0xFF;
  sendBuffer[7] = (targetSpeed >> 16) & 0xFF;
  sendBuffer[6] = (targetSpeed >> 8) & 0xFF;
  sendBuffer[5] = targetSpeed & 0xFF;

  // 加速度 (4字节，单位为 0.01RPM/s)
  sendBuffer[12] = (acceleration >> 24) & 0xFF;
  sendBuffer[11] = (acceleration >> 16) & 0xFF;
  sendBuffer[10] = (acceleration >> 8) & 0xFF;
  sendBuffer[9] = acceleration & 0xFF;

  // CRC16 校验
  uint16_t crc = CalcCRC16_Modbus(sendBuffer, 13);    // CRC计算（从协议头到加速度的字段）

  // 校验码写入
  sendBuffer[13] = crc & 0xFF;           // CRC低字节
  sendBuffer[14] = (crc >> 8) & 0xFF;    // CRC高字节
  // 发送数据包
  for(uint8_t i = 0; i < 15; i++) {
    Serial1.write(sendBuffer[i]);
  }
}

// 电机失能 OFF 命令封装函数
void motorDisable(uint8_t deviceAddress, uint8_t packetNumber)
{
  // 构建命令包
  uint8_t sendBuffer[10] = { 0 };

  // 协议头
  sendBuffer[0] = 0xAE;

  // 包序号
  sendBuffer[1] = packetNumber;

  // 设备地址
  sendBuffer[2] = deviceAddress;

  // 命令码：0x2F 电机失能
  sendBuffer[3] = 0x2F;

  // 数据包长度：0x00 (没有额外数据)
  sendBuffer[4] = 0x00;

  // CRC16 校验（从协议头到数据包长度）
  uint16_t crc = CalcCRC16_Modbus(sendBuffer, 5);    // CRC计算（前5个字节）

  // 校验码写入（低字节在前，高字节在后）
  sendBuffer[5] = crc & 0xFF;           // CRC低字节
  sendBuffer[6] = (crc >> 8) & 0xFF;    // CRC高字节

  // 发送数据包
  for(uint8_t i = 0; i < 7; i++) {
    Serial1.write(sendBuffer[i]);
  }
}

void setup()
{
  // 启动串口 0，用于调试
  Serial.begin(115200);

  // 启动串口 1，设置波特率为 9600
  Serial1.begin(115200, SERIAL_8N1, 16, 17);    // 参数：波特率、数据格式、RX 引脚、TX 引脚

  // 等待串口初始化完成
  delay(1000);
  memset(testCmd, 0, CMD_MAX_LEN);
  // 打印调试信息到串口 0
  Serial.println("Sending 'Hello' via UART1...");
  //  testCmd[0] = 0xAE;
  //  testCmd[1] = 0x00;
  //  testCmd[2] = 0x01;
  //  testCmd[3] = 0x0b;
  //  testCmd[4] = 0x00;
  //  uint16_t crc16 = CalcCRC16_Modbus(testCmd, 5);
  //  testCmd[5] = crc16&0x00FF;
  //  testCmd[6] = (crc16>>8)&0x00FF;
  configureSpeed(1, cmdCount++, 5000, 5000);
  delay(5);
  configureSpeed(2, cmdCount++, 5000, 5000);
  delay(5);
  configureSpeed(3, cmdCount++, 5000, 5000);
  delay(5);
  // configureSpeed(4, cmdCount++, 5000, 5000);
  delay(8000);
  motorDisable(1, cmdCount++);
  delay(5);
  motorDisable(2, cmdCount++);
  delay(5);
  motorDisable(3, cmdCount++);
  delay(5);
  motorDisable(4, cmdCount++);
  delay(5);

  initBluetooth();
}

void loop()
{
  if(loopBluetooth()) {
    return;
  }
}
