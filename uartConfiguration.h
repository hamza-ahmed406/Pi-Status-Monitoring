#ifndef uartConfiguration_h
#define uartConfiguration_h

#include <stdint.h>
#include <stdbool.h>

extern char UARTResponse[2][128];
extern int indexUART[2];
extern bool UARTIntStat[2];	

bool configUART(void);
void UARTSend(uint32_t baseOfUART,const uint8_t *pui8Buffer, uint32_t ui32Count);
void UART0IntHandler(void);
void UART1IntHandler(void);
void UART2IntHandler(void);
void responseRead(uint32_t baseOfUART);

#endif

