#include <HardwareSerial.h>

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

#define UART2_TX_PIN 17           // GPIO17作为TX
#define UART2_RX_PIN 16           // GPIO16作为RX
HardwareSerial MotorSerial(1);    // 使用Serial2

// =======================================================
// 发送设置ID命令
// =======================================================
void sendSetIDCommand(uint8_t newID)
{
  uint8_t rTest[100] = { 0 };
  int     count      = 0;

  // 此版本无CRC 校验
  uint8_t cmd[10] = { 0xAA, 0x55, 0x53, newID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  Serial.print("设置电机ID为: 0x");
  Serial.println(newID, HEX);

  // 连续发送5次
  for (int i = 0; i < 5; i++) {
    MotorSerial.write(cmd, sizeof(cmd));
    delay(500);
    Serial.print("已发送第 ");
    Serial.print(i + 1);
    Serial.println(" 次设置指令");
    if (MotorSerial.available()) {
      while (MotorSerial.available()) {
        rTest[count++] = MotorSerial.read();
      }
    }
  }
}

// =======================================================
// 电机查询函数：发送查询命令并返回电机ID
// 更稳健的读取策略：
//  1) 发送命令前清空串口输入缓冲。
//  2) 发送命令后等待首字节（主超时，例如 500ms）。
//  3) 收到首字节后，使用短的“帧内空闲超时”（例如 50ms）来判断帧结束。
//  返回：>=0 表示解析到的 ID；-1 表示超时/失败。
// =======================================================
int queryMotorID()
{
  const uint8_t queryCmd[10] = { 0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE };
  uint8_t       recvBuf[128];
  int           recvCount = 0;

  // 1) 发送前清空串口接收缓冲
  while (MotorSerial.available()) {
    MotorSerial.read();
  }

  Serial.println("发送电机查询命令...");
  MotorSerial.write(queryCmd, sizeof(queryCmd));
  MotorSerial.flush();    // 等待写入底层缓冲
  delay(1000);
  // 2) 等待首字节，主超时（等待设备应答的最大时间）
  unsigned long       startMillis       = millis();
  const unsigned long firstByteTimeout  = 500;    // ms, 等待首字节
  bool                firstByteReceived = false;
  if (MotorSerial.available()) {
    while (MotorSerial.available()) {
      recvBuf[recvCount++] = MotorSerial.read();
    }
  }
  if (recvCount == 0) {
    return -1;
  }
  return recvBuf[0];
}

// =======================================================
// 初始化
// =======================================================
void setup()
{
  Serial.begin(115200);
  MotorSerial.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  delay(1000);

  Serial.println("开始设置电机ID...");
  sendSetIDCommand(0x04);    // 示例：设置为ID 0x04
  delay(1000);

  Serial.println("开始查询电机ID...");
  int motorID = queryMotorID();
  if (motorID >= 0) {
    Serial.print("电机ID查询结果: 0x");
    Serial.println(motorID, HEX);
  } else {
    Serial.println("查询失败！");
  }

  Serial.println("程序结束。请断电重启以验证ID是否生效。");
  while (1);
}

void loop() {}
