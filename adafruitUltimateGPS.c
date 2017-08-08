
#include <stdint.h>
#include <stdbool.h>
#include <String.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include <time.h>
#include "driverlib/timer.h"
#include "drivers/buttons.h"
#include "inc/tm4c123gh6pge.h"
#include "inc/SysTick.h"
#include "inc/lcd.h"
#include "strfunc.h"
#include "uartConfiguration.h"
#include "utils/scheduler.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
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
#include "adafruitUltimateGPS.h"
//#include "buffer.h"


void RingBuffer_Init(RingBuffer *temp)
{
	temp->head=0;
	temp->tail=0;
	temp->state='A';
	temp->sentence_flag=false;
	temp->size=LENGTH_BUFFER;
}

void RingBuffer_Push(char letter,RingBuffer *temp)
{
	
	temp->buffer[temp->head]=letter;
	temp->head=((temp->head)+1)%(temp->size);
	if(temp->head==temp->tail)
	{
		temp->tail=((temp->tail)+1)%temp->size;
	}
}

char RingBuffer_PopChar(RingBuffer *temp)
{
	char out_letter;
	out_letter=temp->buffer[temp->tail];
	temp->tail=((temp->tail)+1)%temp->size;
	return out_letter;
}

void RingBuffer_TransBuffer(RingBuffer *source, RingBuffer *destination)
{
	while((source->tail)!=(source->head))
	{
		RingBuffer_Push(RingBuffer_PopChar(source),destination);
	}
}
void RingBuffer_Clear(RingBuffer *temp)
{
	int k=0;
	for(k=0;k<temp->size;k++)
	{
		temp->buffer[k]='0';
	}
	temp->head=0;
	temp->tail=0;
	temp->sentence_flag=false;
	temp->state='A';
}


void RingBuffer_Print(RingBuffer *temp)
{
	int i=0;
	int o_head=temp->head;
	int o_tail=temp->tail;
	while((temp->tail)!=(temp->head))
	{
		printf("%d\t%c\n",i,temp->buffer[temp->tail]);
		temp->tail=((temp->tail)+1)%(temp->size);
		i++;
	}
	temp->tail=o_tail;
	temp->head=o_head;
}

void RingBuffer_PrintIndex(RingBuffer *curr)
{
	printf("\nHead: %d\t Tail: %d\n",curr->head,curr->tail);
}


RingBuffer rb_GPS;
bool gpsStringComp=false;

bool garbageflaggps=false;
struct GPSDATA data; //create GPSDATA object for this .c file
struct GPSDATA const * const GPSData=&data;	//create constant pointer so that Apllication can (only)read the data


void initGPS()
{
	//configure uart 1		
  RingBuffer_Init(&rb_GPS); 
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
   
  ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	
  ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
	UARTFIFOEnable(UART1_BASE);
	UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX4_8,UART_FIFO_RX4_8);
  ROM_IntEnable(INT_UART1);
  ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

  ROM_GPIOPinConfigure(GPIO_PC4_U1RX);
  ROM_GPIOPinConfigure(GPIO_PC5_U1TX);
	gpsStringComp=false;

}

//GPS Input UART
void GPSIntHandler(void)
{
	uint32_t ui32Status;
	ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
	ROM_UARTIntClear(UART1_BASE, ui32Status);
	char temp;
	while(ROM_UARTCharsAvail(UART1_BASE))
    {
			temp=ROM_UARTCharGetNonBlocking(UART1_BASE);
//			UARTCharPut(UART0_BASE,temp );
			RingBuffer_Push(temp,&rb_GPS);
			if(temp=='\n')
			{
				gpsStringComp=true;
			}
    }
}




int commagps=0;
char gpsheader[3];
	char gps[5];
void GPSGetData(void)
{
	char temp;
	rb_GPS.sentence_flag=false;
	while ((rb_GPS.tail) != (rb_GPS.head))
	{
		//TO_PC("\r\n");
		//TO_PC(&rb_GPS.state);
		
		switch (rb_GPS.state)
		{
		case 'A':
			temp = RingBuffer_PopChar(&rb_GPS);
			//TO_PC(&temp);
			//searching for $
			if (temp == '$')
			{
				//RingBuffer_Push(temp, destination);
				rb_GPS.state = 'B';
				//rb_GPS.sentence_flag = false;
			}
			break;
		case 'B':
			//Obtain the header and comparing it
			for (int j = 0; j <= 1; j++)
			{
				gps[j] = RingBuffer_PopChar(&rb_GPS);
				//TO_PC(&gps[j]);
			}
			if (gps[0] == 'G' && gps[1] == 'P')
			{
				garbageflaggps = false;
			for(int i=0;i<3;i++)
				{
				gpsheader[i]=RingBuffer_PopChar(&rb_GPS);
					//TO_PC(&gpsheader[i]);
				}
       if(gpsheader[0]=='R' && gpsheader[1]=='M' && gpsheader[2]=='C')
       {
				rb_GPS.state='C';
				 commagps=0;
			 }
      else
			 {
				rb_GPS.state='A';
			 }				
				
			}
			else
			{
				garbageflaggps = true;
				rb_GPS.state = 'A';
			}

			//destination->sentence_flag = false;
			break;

		case 'C':
			//Check whether the data after specified commas is correct				
			temp = RingBuffer_PopChar(&rb_GPS);
		//TO_PC(&temp);
			if (temp == ',')
			{commagps = commagps + 1;
				//rb_GPS.sentence_flag = false;
				if (commagps == 1)
				{
					for (int j=0;;j++)
					{
						data.Time[j]= RingBuffer_PopChar(&rb_GPS);
						//TO_PC(&data.Time[j]);
					if(data.Time[j]==',')
					{
						commagps++;
						data.Time[j]=NULL;

						break;
					}
				  }
				}	
				if (commagps == 11)
				{
					//commaGyro = 0;
					commagps=0;
					//rb_GPS.sentence_flag = false;
					rb_GPS.state = 'D';
				}
	    	}
			else
			{
				//RingBuffer_Push(temp, destination);
				//rb_GPS.sentence_flag = false;
			}
			break;
		case 'D':
			//Check for invalid data and \n:
			temp = RingBuffer_PopChar(&rb_GPS);
		//TO_PC(&temp);
			//BoatMagnitude = atoi(gpsdata);
//			BoatDirection = atoi(gpsdata1);
			

			if (temp == '\n')
			{
			//	RingBuffer_Push(temp, destination);
				rb_GPS.sentence_flag = true;
				rb_GPS.state = 'A';
				commagps=0;
						
			}
			else
			{
		//		RingBuffer_Push(temp, destination);
				rb_GPS.sentence_flag = false;
			}
			break;
		}
	}
	
	gpsStringComp=false;
}

bool isGPSSentenceComplete(void)
{
	return gpsStringComp;
}

bool isGPSDataUpdated()
{
	return rb_GPS.sentence_flag;
}