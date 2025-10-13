#include "crc16_modbus.h"
#include "stupidStupid.h"

sMatDat_t matDat;

#define CMD_MAX_LEN 120
// 轮子索引：LF, RF, LR, RR
enum { LF = 0, RF = 1, LR = 2, RR = 3 };

typedef enum{
  d_left = 0,
  d_right = 1,
}eDirection_t;
uint8_t testCmd[CMD_MAX_LEN]={0};
uint16_t cmdCount = 0;
uint8_t  speedLevel = 0;  // 配置速度等级
const uint16_t speedBase[5] = {5000, 10000, 15000, 20000, 25000};
const uint16_t acceleratedBase[5]= {5000, 5000, 7500, 10000, 10000};

void _driveMotor(uint16_t directionAngle, float speed);
void _rotary(eDirection_t direction, float speed);
void computeRawWheelSpeeds(float V, float theta_deg, float out[4]);
void normalizeWheelSpeeds(const float in[4], float out[4]);


// ----------------------------------------------------------------------------
// normalizeWheelSpeeds
// 将 four[] 按绝对值最大值归一化，使得 max(|four|) == 1（如果全为0则保持0）
// 输入 in[4]，输出写回到 out[4]（可以与 in 指向同一数组）
// ----------------------------------------------------------------------------
void normalizeWheelSpeeds(const float in[4], float out[4]) {
  float maxAbs = 0.0f;
  for (int i = 0; i < 4; ++i) {
    float a = fabsf(in[i]);
    if (a > maxAbs) maxAbs = a;
  }
  if (maxAbs <= 1e-6f) {
    // 全零或接近零，直接复制
    for (int i = 0; i < 4; ++i) out[i] = 0.0f;
    return;
  }
  // 归一化
  for (int i = 0; i < 4; ++i) out[i] = in[i] / maxAbs;
}

// ----------------------------------------------------------------------------
// computeRawWheelSpeeds
// 计算未归一化的四轮速度（浮点），基于公式：
// vFL = V*(cosθ + sinθ)
// vFR = V*(cosθ - sinθ)
// vLR = V*(cosθ - sinθ)
// vRR = V*(cosθ + sinθ)
// 约定：theta_deg 以“正前”为 0°，向右为正（右前为正角）
// 输出到 out[4]：顺序 LF, RF, LR, RR
// ----------------------------------------------------------------------------
void computeRawWheelSpeeds(float V, float theta_deg, float out[4]) {
  const float theta = theta_deg * (PI / 180.0f); // 转为弧度
  const float c = cosf(theta);
  const float s = sinf(theta);

  out[LF] = V * (c + s);
  out[RF] = V * (c - s);
  out[LR] = V * (c - s);
  out[RR] = V * (c + s);
}

// 配置速度命令的函数
void configureSpeed(uint8_t deviceAddress, uint8_t packetNumber, int32_t targetSpeed, int32_t acceleration) {
  // 构建命令包
  uint8_t sendBuffer[20] = {0};
  uint8_t _deviceAddress = deviceAddress+1;
  // 协议头
  sendBuffer[0] = 0xAE;
  
  // 包序号
  sendBuffer[1] = packetNumber;
  
  // 设备地址
  sendBuffer[2] = _deviceAddress;
  if((_deviceAddress % 2) != 0){
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
  sendBuffer[9]  = acceleration & 0xFF;
  
  // CRC16 校验
  uint16_t crc = CalcCRC16_Modbus(sendBuffer, 13); // CRC计算（从协议头到加速度的字段）
  
  // 校验码写入
  sendBuffer[13] = crc & 0xFF;         // CRC低字节
  sendBuffer[14] = (crc >> 8) & 0xFF;  // CRC高字节
  // 发送数据包
  for (uint8_t i = 0; i < 15; i++) {
    Serial1.write(sendBuffer[i]);
  }
  delay(5); // 不加延时会出现问题
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
  uint16_t realAccelerated= acceleratedBase[speedLevel];
  if(direction == d_left){
    configureSpeed(LF, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RF, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(LR, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed, realAccelerated);
  }else if(direction == d_right){
    configureSpeed(LF, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RF, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(LR, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed*-1, realAccelerated);
  }
}
void _driveMotor(uint16_t directionAngle, float speed)
{
  uint16_t realSpeed = speedBase[speedLevel]*speed;
  uint16_t realAccelerated= acceleratedBase[speedLevel];
  float raw[4];
  float norm[4];
  int16_t dirveSpeed[5]= {0};
  // 1) 计算未归一化的四轮比值
  computeRawWheelSpeeds(1.0f, directionAngle, raw); // V=1 -> 得到方向分布
  // 2) 归一化（使得最大绝对值 = 1）
  normalizeWheelSpeeds(raw, norm);
  Serial.println(norm[LF]);
  Serial.println(norm[RF]);
  Serial.println(norm[LR]);
  Serial.println(norm[RR]);

  dirveSpeed[LF] = realSpeed * norm[LF];
  dirveSpeed[RF] = realSpeed * norm[RF];
  dirveSpeed[LR] = realSpeed * norm[LR];
  dirveSpeed[RR] = realSpeed * norm[RR];
  
  configureSpeed(LF, cmdCount++, dirveSpeed[LF], realAccelerated);
  configureSpeed(RF, cmdCount++, dirveSpeed[RF], realAccelerated);
  configureSpeed(LR, cmdCount++, dirveSpeed[LR], realAccelerated);
  configureSpeed(RR, cmdCount++, dirveSpeed[RR], realAccelerated);

#if 0
  if(directionAngle==0 || directionAngle==360){
    configureSpeed(FL, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(FR, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RL, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed, realAccelerated);
  }else if(directionAngle > 0 && directionAngle < 45){

  }else if(directionAngle==45){
    configureSpeed(FL, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(FR, cmdCount++, 0, realAccelerated);
    configureSpeed(RL, cmdCount++, 0, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed, realAccelerated);
  }else if(directionAngle > 45 && directionAngle < 90){
  }else if(directionAngle==90){
    configureSpeed(FL, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(FR, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RL, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed, realAccelerated);
  }else if(directionAngle==135){
    configureSpeed(FL, cmdCount++, 0, realAccelerated);
    configureSpeed(FR, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RL, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RR, cmdCount++, 0, realAccelerated);
  }else if(directionAngle==180){
    configureSpeed(FL, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(FR, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RL, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed*-1, realAccelerated);
  }else if(directionAngle==225){
    configureSpeed(FL, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(FR, cmdCount++, 0, realAccelerated);
    configureSpeed(RL, cmdCount++, 0, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed*-1, realAccelerated);
  }else if(directionAngle==270){
    configureSpeed(FL, cmdCount++, realSpeed*-1, realAccelerated);
    configureSpeed(FR, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RL, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RR, cmdCount++, realSpeed*-1, realAccelerated);
  }else if(directionAngle==315){
    configureSpeed(FL, cmdCount++, 0, realAccelerated);
    configureSpeed(FR, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RL, cmdCount++, realSpeed, realAccelerated);
    configureSpeed(RR, cmdCount++, 0, realAccelerated);
  }
#endif

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
  initBluetooth();
}

void loop()
{
  if(loopBluetooth()) {
    return;
  }
}
