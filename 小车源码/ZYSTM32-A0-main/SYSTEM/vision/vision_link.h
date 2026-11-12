#ifndef VISION_LINK_H
#define VISION_LINK_H
#include "stm32f10x.h"

#define K210_CLASS_MODEL 0x12u
#define K210_FRAME_SIZE 29u
typedef struct {
    uint16_t x, y, w, h;
    uint8_t id, confidence, found;
    uint32_t stamp;
} K210_MSG_TypeDef;

void VisionLink_Init(void);
/* Copy latest packet atomically; 1 only for a new packet. Includes no-target heartbeats. */
uint8_t VisionLink_Read(K210_MSG_TypeDef *message);
uint32_t VisionLink_GetRxBytes(void);
uint32_t VisionLink_GetValidFrames(void);
uint32_t VisionLink_GetBadFrames(void);
#endif
