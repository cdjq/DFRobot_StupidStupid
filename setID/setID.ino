#include <HardwareSerial.h>

#define UART2_TX_PIN 17           // GPIO17作为TX
#define UART2_RX_PIN 16           // GPIO16作为RX
HardwareSerial MotorSerial(2);    // 使用Serial2

// CRC-8/MAXIM 多项式 (反射后为 0x8C)
#define CRC8_POLY 0x8C
#define CRC8_INIT 0x00

// 计算一个字节的 CRC
uint8_t crc8_update(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for(uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x01) ? (crc >> 1) ^ CRC8_POLY : (crc >> 1);
  }
  return crc;
}

// 计算整个数据缓冲区的 CRC
uint8_t calculate_crc(uint8_t data[], uint8_t length)
{
  uint8_t crc = CRC8_INIT;
  for(uint8_t i = 0; i < length; i++) {
    crc = crc8_update(crc, data[i]);
  }
  return crc;
}

// 构造并发送设置ID的指令
void sendSetIDCommand(uint8_t newID)
{
  uint8_t cmd[10] = { 0xAA, 0x55, 0x53, newID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  cmd[9] = calculate_crc(cmd, 9);    // 最后一字节填CRC
  Serial.print("crc8 = ");
  Serial.println(cmd[9], HEX);
  Serial.print("设置电机ID为: 0x");
  Serial.println(newID, HEX);

  // 连续发送5次
  for(int i = 0; i < 5; i++) {
    MotorSerial.write(cmd, sizeof(cmd));
    delay(100);
    Serial.print("已发送第 ");
    Serial.print(i + 1);
    Serial.println(" 次设置指令");
  }
}

void setup()
{
  Serial.begin(115200);
  MotorSerial.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  delay(1000);

  Serial.println("开始设置电机ID...");

  // 示例：将电机ID设置为 0x01
  sendSetIDCommand(0x04);
  ///////////////////////////////////////////////
  //
  //
  // 设置id 后需要给电机断电，再次运行此程序查看后面的id打印，是否更改成功
  //
  //
  ///////////////////////////////////////////////
  delay(500);
}

void loop()
{
  uint8_t test[100] = { 0 };
  int     count1 = 0;
  if(MotorSerial.available()) {
    while(MotorSerial.available()) {
      test[count1++] = MotorSerial.read();
    }
  }
  for(uint8_t i = 0; i < 5; i++) {
    Serial.print("id = ");
    Serial.println(test[10 * i], HEX);
  }
  // 给电机断电，再次复位esp32 此时id打印和更改的id 相同更改成功
  Serial.println("plsese rst !!!");
  while(1);
}
