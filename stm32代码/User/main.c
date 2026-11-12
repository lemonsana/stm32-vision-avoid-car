#include "stm32f10x.h"
#include "Serial.h"
#include <stdio.h>

/* Set to 1 only if an OLED is connected: PB6 SCL, PB7 SDA. */
#define USE_OLED 0
#if USE_OLED
#include "OLED.h"
#endif

int main(void)
{
    K210_MSG_TypeDef target;
    char buffer[128];
    const char *label;
    uint8_t previous_found = 2;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Serial_Init(115200);
    Serial_SendString("STM32 READY: waiting for K210, 115200 8N1\r\n");
#if USE_OLED
    OLED_Init();
    OLED_ShowString(1, 1, "K210 MODEL");
    OLED_ShowString(2, 1, "x=   y=");
    OLED_ShowString(3, 1, "w=   h=");
#endif
    while (1)
    {
        if (!K210_ReadMessage(&target)) continue;
        if (target.found)
        {
            label = (target.id == 0) ? "z" : "unknown";
            sprintf(buffer,
                    "DETECTED label=%s,id=%u,conf=%u%%,x=%u,y=%u,w=%u,h=%u,cx=%u,cy=%u\r\n",
                    label, (unsigned)target.id, (unsigned)target.confidence,
                    (unsigned)target.x, (unsigned)target.y,
                    (unsigned)target.w, (unsigned)target.h,
                    (unsigned)(target.x + target.w / 2),
                    (unsigned)(target.y + target.h / 2));
            Serial_SendString(buffer);
        }
        else if (previous_found != 0)
        {
            Serial_SendString("NO TARGET (K210 link OK)\r\n");
        }
        previous_found = target.found;
#if USE_OLED
        OLED_ShowString(1, 12, target.found ? "YES" : "NO ");
        OLED_ShowNum(2, 3, target.x, 3);
        OLED_ShowNum(2, 8, target.y, 3);
        OLED_ShowNum(3, 3, target.w, 3);
        OLED_ShowNum(3, 8, target.h, 3);
#endif
    }
}
