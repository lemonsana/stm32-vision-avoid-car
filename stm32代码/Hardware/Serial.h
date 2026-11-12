#ifndef __SERIAL_H
#define __SERIAL_H
#include "stm32f10x.h"

#define K210_CLASS_MODEL 0x12u
#define K210_FRAME_SIZE 29u
typedef struct
{
    uint16_t x, y, w, h;
    uint8_t id;           /* label.txt index; 0 = z */
    uint8_t confidence;   /* 0..100 percent */
    uint8_t found;        /* 1 = target, 0 = no target */
} K210_MSG_TypeDef;

void Serial_Init(uint32_t baud);
void USART2_Send_ArrayU8(const uint8_t *buf, uint16_t len);
void Serial_SendString(const char *str);
/* Atomically copy the latest complete packet. Returns 1 once per new packet. */
uint8_t K210_ReadMessage(K210_MSG_TypeDef *message);
#endif
