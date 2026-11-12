#include "vision_link.h"
#include "sched.h"

static volatile K210_MSG_TypeDef latest;
static volatile uint8_t message_ready;
static uint8_t frame[K210_FRAME_SIZE];
static uint8_t frame_count;
static uint32_t byte_stamp;
static volatile uint32_t rx_bytes;
static volatile uint32_t valid_frames;
static volatile uint32_t bad_frames;

void VisionLink_Init(void)
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
    uart.USART_BaudRate = 115200;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart); /* 8 bits, no parity, 1 stop bit */
    frame_count = 0;
    message_ready = 0;
    rx_bytes = valid_frames = bad_frames = 0;
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

uint8_t VisionLink_Read(K210_MSG_TypeDef *message)
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

uint32_t VisionLink_GetRxBytes(void) { return rx_bytes; }
uint32_t VisionLink_GetValidFrames(void) { return valid_frames; }
uint32_t VisionLink_GetBadFrames(void) { return bad_frames; }

static uint16_t ReadU16(const uint8_t *raw)
{
    return (uint16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
}

/* Sliding window resynchronizes after noise, truncation or a bad checksum.
 * Data bytes equal to '$', '#' or ',' must remain intact. */
static void ReceiveByte(uint8_t data)
{
    uint8_t i, sum, raw[11];
    if ((uint32_t)(g_tick_ms - byte_stamp) > 100u) frame_count = 0;
    byte_stamp = g_tick_ms;
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
        if (i == 11 && raw[9] <= 100u && raw[10] <= 1u &&
            (raw[10] == 0u ||
             (ReadU16(&raw[0]) < 320u && ReadU16(&raw[2]) < 240u &&
              ReadU16(&raw[4]) > 0u && ReadU16(&raw[6]) > 0u &&
              (uint32_t)ReadU16(&raw[0]) + ReadU16(&raw[4]) <= 320u &&
              (uint32_t)ReadU16(&raw[2]) + ReadU16(&raw[6]) <= 240u)))
        {
            latest.x = ReadU16(&raw[0]);
            latest.y = ReadU16(&raw[2]);
            latest.w = ReadU16(&raw[4]);
            latest.h = ReadU16(&raw[6]);
            latest.id = raw[8];
            latest.confidence = raw[9];
            latest.found = raw[10];
            latest.stamp = g_tick_ms;
            message_ready = 1;
            valid_frames++;
            frame_count = 0;
            return;
        }
    }
    for (i = 1; i < K210_FRAME_SIZE; ++i) frame[i - 1] = frame[i];
    frame_count = K210_FRAME_SIZE - 1;
    bad_frames++;
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
        rx_bytes++;
        if (status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
        {
            frame_count = 0;
            return;
        }
        if (status & USART_FLAG_RXNE) ReceiveByte(data);
    }
}
