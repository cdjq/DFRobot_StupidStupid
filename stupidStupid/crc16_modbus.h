#ifndef __CRC16_MODBUS_H
#define __CRC16_MODBUS_H

#include "Arduino.h"
#define usMBCRC16 CalcCRC16_Modbus

uint16_t CalcCRC16_Modbus(uint8_t *Frame, uint16_t Len);

#endif /*_CRC16_MODBUS_H */
