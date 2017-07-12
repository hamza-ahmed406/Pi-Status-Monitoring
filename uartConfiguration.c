#include "uartConfiguration.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "utils/ustdlib.h"
#include "inc/tm4c123gh6pge.h"
#include "utils/uartstdio.h"
//#include "sd_card.h"


char UARTResponse[2][128];

int indexUART[2]={0,0};								//indexes of UART buffer
bool UARTIntStat[2];										//to check the interrupt status 

bool configUART()
{
	//config uart 0
	
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    ROM_IntMasterEnable();
    
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, ROM_SysCtlClockGet());
	
		ROM_IntPrioritySet(INT_UART0,4);

//configure uart 1		
    
//       ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//   
//    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
//		
//    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
//                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                             UART_CONFIG_PAR_NONE));
//		UARTFIFOEnable(UART1_BASE);
//		UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX4_8,UART_FIFO_RX4_8);
//    ROM_IntEnable(INT_UART1);
//    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

//    ROM_GPIOPinConfigure(GPIO_PC4_U1RX);
//    ROM_GPIOPinConfigure(GPIO_PC5_U1TX);
//		indexUART[1]=0;

	return true;
}


// Send a string to the UART.
void
UARTSend(uint32_t baseOfUART,const uint8_t *pui8Buffer, uint32_t ui32Count)
{	
//	writeToFile((char *) pui8Buffer);		//write the data to sd card file
   
  while(ui32Count--)		// Loop while there are more characters to send.
        ROM_UARTCharPut(baseOfUART, *pui8Buffer++);		// Write the next character to the UART.
}


//void UART1IntHandler()
//{
//	uint32_t ui32Status;
//	ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
//  UARTIntClear(UART1_BASE, ui32Status);	
//		
//	while(UARTCharsAvail(UART1_BASE)) //loop while there is data
//	{
//			if (indexUART[1]>125)
//				return;
//		UARTResponse[1][indexUART[1]]=ROM_UARTCharGetNonBlocking(UART1_BASE);
//		UARTCharPut(UART0_BASE,UARTResponse[1][(indexUART[1])++] );
//	}
//	
//	UARTResponse[1][indexUART[1]]=0;
//	UARTIntStat[1]=true;
//}		

void UART0IntHandler(void)
{
	uint32_t ui32Status = ROM_UARTIntStatus(UART0_BASE, true);	 // Get the interrrupt status.

	ROM_UARTIntClear(UART0_BASE, ui32Status);	    // Clear the asserted interrupts.

  while(ROM_UARTCharsAvail(UART0_BASE))		    // Loop while there are characters in the receive FIFO.
  {

		UARTResponse[0][indexUART[0]]=ROM_UARTCharGetNonBlocking(UART0_BASE);			// Read the next character from the UART
		ROM_UARTCharPutNonBlocking(UART0_BASE,UARTResponse[0][(indexUART[0])++]);	//and write it back to the UART.
                                   
	}

	UARTResponse[0][indexUART[0]]=0;
	UARTIntStat[0]=true;

	if (ustrcmp(UARTResponse[0],"sd\r")==0)
//		sdCard();

	if (ustrstr(UARTResponse[0],"\r")!=NULL)
		responseRead(UART0_BASE);
		
}


void responseRead(uint32_t baseOfUART)	//clears the UART buffer
{
	int UARTNo= (baseOfUART - UART0_BASE)/ 0x1000;

//	writeToFile(UARTResponse[UARTNo]);	
	UARTResponse[UARTNo][0]=indexUART[UARTNo]=0;	//setting index and buffer to 0
	UARTIntStat[UARTNo]=false;								//clear interrupt flag
}		
