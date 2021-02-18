#ifndef __USART_FIFO_H
#define __USART_FIFO_H
#include <stdint.h>

#define FIFO_LENGTH 			256
#define UART_DMA_RECEIVE_LENGTH	128
#define UART_DMA_SEND_LENGTH	128
#define UART_SEND_LENGTH		FIFO_LENGTH - 1

typedef struct _USART_FIFO{
    uint8_t base[FIFO_LENGTH];
    uint16_t head, tail;
	uint16_t size;
}USART_FIFO;

void FIFO_init(USART_FIFO *UsartFIFO);
int FIFO_pushData(USART_FIFO *UsartFIFO, uint8_t *src, uint16_t len);
int FIFO_popData(USART_FIFO *UsartFIFO, uint8_t *des, uint16_t len);
int FIFO_getDataLength(USART_FIFO *UsartFIFO);

#endif
