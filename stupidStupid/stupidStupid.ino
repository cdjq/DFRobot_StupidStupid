#include "crc16_modbus.h"
#include "stupidStupid.h"
#include <stdint.h>

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
  uint8_t sendBuffer[10] = {0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
  for(uint8_t j = 1; j <= 4; j++){
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
  uint8_t deviceId = deviceAddress+1;  //  地址从 1 开始
  int16_t speed = targetSpeed;
  if((deviceId%2 == 0)){
    speed *= -1;
  }
  uint8_t sendPackage[20] = {0};
  sendPackage[0] = deviceId;
  sendPackage[1] = 0x64;
  sendPackage[2] = speed >> 8;
  sendPackage[3] = speed;
  sendPackage[4] = 0;
  sendPackage[5] = 0;
  sendPackage[6] = 0;
  sendPackage[7] = 0;
  sendPackage[8] = 0;
  sendPackage[9] = crc8_maxim_compute(sendPackage, 9);
  for (uint8_t i = 0; i < 10; i++) {
    Serial1.write(sendPackage[i]);
  }
  delay(4);
}

// 电机失能 OFF 命令封装函数
void motorDisable(uint8_t deviceAddress, uint8_t packetNumber) {
  // 构建命令包
  uint8_t sendBuffer[10] = {0};
  
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
  uint16_t crc = CalcCRC16_Modbus(sendBuffer, 5);  // CRC计算（前5个字节）
  
  // 校验码写入（低字节在前，高字节在后）
  sendBuffer[5] = crc & 0xFF;         // CRC低字节
  sendBuffer[6] = (crc >> 8) & 0xFF;  // CRC高字节
  
  // 发送数据包
  for (uint8_t i = 0; i < 7; i++) {
    Serial1.write(sendBuffer[i]);
  }
  delay(5);
}

void _rotary(eDirection_t direction, float speed)
{
  uint16_t realSpeed = speedBase[speedLevel]*speed;
  uint16_t driveSpeed[4] = {0};
  if(direction == d_left){
    driveSpeed[LF] = -1*realSpeed;
    driveSpeed[LR] = -1*realSpeed;
    driveSpeed[RF] = realSpeed;
    driveSpeed[RR] = realSpeed;
    
  }else if(direction == d_right){
    driveSpeed[LF] = realSpeed;
    driveSpeed[LR] = realSpeed;
    driveSpeed[RF] = -1*realSpeed;
    driveSpeed[RR] = -1*realSpeed;
  }
  configureSpeed(LF, driveSpeed[LF]);
  configureSpeed(RF, driveSpeed[RF]);
  configureSpeed(LR, driveSpeed[LR]);
  configureSpeed(RR, driveSpeed[RR]);
}
void _driveMotor(uint16_t directionAngle, float speed)
{
  uint16_t realSpeed = speedBase[speedLevel]*speed;
  uint16_t realAccelerated= acceleratedBase[speedLevel];
  uint16_t driveSpeed[4] = {0};
  // 右侧正转并且减速
  if(directionAngle >= 0 && directionAngle <= 90){
    int16_t degree = (directionAngle-90)*-1; 
    float subRight = 1.0f - directionAngle / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = realSpeed*subRight;
    driveSpeed[LF] = driveSpeed[LR] = realSpeed;
  }

  // 右侧反转并且减速
  if(directionAngle > 90 && directionAngle <= 180){
    int16_t degree = (directionAngle-180)*-1; 
    float subRight = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = -1 * realSpeed*subRight;
    driveSpeed[LF] = driveSpeed[LR] = -1 * realSpeed;
  }

  // 左侧正转并减速
  if(directionAngle > 270 && directionAngle <= 360){
    int16_t degree = (directionAngle-360)*-1; 
    float subLeft = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = realSpeed;
    driveSpeed[LF] = driveSpeed[LR] = realSpeed*subLeft;
  }

  // 左侧反转并减速
  if(directionAngle > 180 && directionAngle <= 270){
    int16_t degree = (directionAngle-270)*-1; 
    float subLeft = 1.0f - degree / 90.0;
    driveSpeed[RF] = driveSpeed[RR] = -1 * realSpeed;;
    driveSpeed[LF] = driveSpeed[LR] = -1 * realSpeed*subLeft;
  }

  
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
  if(loopBluetooth()) {
    return;
  }

}
