#include "Serial.h"

static volatile K210_MSG_TypeDef latest;
static volatile uint8_t message_ready;
static uint8_t frame[K210_FRAME_SIZE];
static uint8_t frame_count;

void Serial_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* PA2 TX -> USB-TTL RX. PA3 RX <- K210 TX. PA4 is not USART2 TX. */
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = baud;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart); /* 8 bits, no parity, 1 stop bit */
    frame_count = 0;
    message_ready = 0;
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void USART2_Send_ArrayU8(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; ++i)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART2, buf[i]);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}
}

void Serial_SendString(const char *str)
{
    while (*str)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART2, (uint8_t)*str++);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}
}

uint8_t K210_ReadMessage(K210_MSG_TypeDef *message)
{
    uint8_t ready;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ready = message_ready;
    if (ready)
    {
        *message = latest;
        message_ready = 0;
    }
    __set_PRIMASK(primask);
    return ready;
}

static uint16_t ReadU16(const uint8_t *raw)
{
    return (uint16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
}

/* Sliding window resynchronizes after noise, truncation or a bad checksum.
 * Data bytes equal to '$', '#' or ',' must remain intact. */
static void ReceiveByte(uint8_t data)
{
    uint8_t i, sum, raw[11];
    if (frame_count == 0 && data != 0x24u) return;
    frame[frame_count++] = data;
    if (frame_count < K210_FRAME_SIZE) return;
    sum = 0;
    for (i = 1; i < K210_FRAME_SIZE - 2; ++i) sum = (uint8_t)(sum + frame[i]);
    if (frame[0] == 0x24u && frame[1] == 27u &&
        frame[2] == K210_CLASS_MODEL && frame[3] == 0xBBu &&
        frame[4] == 22u && frame[27] == sum && frame[28] == 0x23u)
    {
        for (i = 0; i < 11; ++i)
        {
            if (frame[6 + 2 * i] != 0x2Cu) break;
            raw[i] = frame[5 + 2 * i];
        }
        if (i == 11 && raw[9] <= 100u && raw[10] <= 1u)
        {
            latest.x = ReadU16(&raw[0]);
            latest.y = ReadU16(&raw[2]);
            latest.w = ReadU16(&raw[4]);
            latest.h = ReadU16(&raw[6]);
            latest.id = raw[8];
            latest.confidence = raw[9];
            latest.found = raw[10];
            message_ready = 1;
            frame_count = 0;
            return;
        }
    }
    for (i = 1; i < K210_FRAME_SIZE; ++i) frame[i - 1] = frame[i];
    frame_count = K210_FRAME_SIZE - 1;
}

void USART2_IRQHandler(void)
{
    uint16_t status = USART2->SR;
    uint8_t data;
    /* Read SR then DR to clear RXNE and error flags, including overrun. */
    if (status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE |
                  USART_FLAG_FE | USART_FLAG_PE))
    {
        data = (uint8_t)USART2->DR;
        if (status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
        {
            frame_count = 0;
            return;
        }
        if (status & USART_FLAG_RXNE) ReceiveByte(data);
    }
}
