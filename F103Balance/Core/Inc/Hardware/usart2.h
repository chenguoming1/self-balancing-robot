#ifndef __USART2_H
#define __USART2_H
#include "robot_hal.h"

extern u8 Fore, Back, Left, Right;

void uart2_init(u32 bound);
void BluetoothCMD(int cmd);
void Uart2SendByte(char byte);
void Uart2SendBuf(char *buf, u16 len);
void Uart2SendStr(char *str);

#endif
