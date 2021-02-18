#include "stm32f1xx_hal.h"
#include "usart_fifo.h"
#include "string.h"

uint8_t UartDMAReveiveBuffer[UART_DMA_RECEIVE_LENGTH] = {0};	// DMA接收地址
uint8_t UartDMASendBuffer[UART_DMA_SEND_LENGTH] = {0};			// DMA发送地址
uint8_t UartSendBuffer[UART_SEND_LENGTH] = {0};					// FIFO取出的数据存放在此
uint8_t FIFOBuffer[FIFO_LENGTH] = {0};							// FIFO缓冲区
USART_FIFO usartRecvFIFO;
USART_FIFO usartSendFIFO;

void FIFO_init(USART_FIFO *UsartFIFO)
{
    UsartFIFO->head = 0;
    UsartFIFO->tail = 0;
	UsartFIFO->size = sizeof(UsartFIFO->base);
    memset(UsartFIFO->base, 0, sizeof(UsartFIFO->base));
}

int FIFO_pushData(USART_FIFO *UsartFIFO, uint8_t *src, uint16_t len) 
{
    int count = 0;

    if (src == NULL) 
	{
        return -1;
    }
    while (len > 0) 
	{
        if ((UsartFIFO->tail + 1) % (UsartFIFO->size) == UsartFIFO->head) 
		{
            return count;
        }
        UsartFIFO->base[UsartFIFO->tail] = src[count++];
        UsartFIFO->tail = (UsartFIFO->tail + 1) % (UsartFIFO->size);
        len --;
    }

    return count;
}

int FIFO_popData(USART_FIFO *UsartFIFO, uint8_t *des, uint16_t len) 
{
    uint16_t count = 0;

    if (des == NULL) 
	{
        return -1;
    }

    while (len > 0) 
	{
        if ((UsartFIFO->head % UsartFIFO->size) == UsartFIFO->tail) 
		{
            return count;
        }
        des[count++] = UsartFIFO->base[UsartFIFO->head];
        UsartFIFO->head = (UsartFIFO->head + 1) % (UsartFIFO->size);
        len --;
    }

    return count;
}

int FIFO_getDataLength(USART_FIFO *UsartFIFO) 
{
    if (UsartFIFO->head > UsartFIFO->tail) 
	{
        return FIFO_LENGTH - UsartFIFO->head + UsartFIFO->tail;
    } 
	else if (UsartFIFO->head < UsartFIFO->tail) 
	{
        return UsartFIFO->tail - UsartFIFO->head;
    } 
	else 
	{
        return 0;
    }
}
