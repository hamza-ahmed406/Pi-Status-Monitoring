/*********************************************************** 
	This is Main program Application for Monitoring Pi status
	
	Status from Pi is received by PD0 pin. On every toggle,
	status is updated which is displayed by LED's. There are 4 states:
	First status is for representing whether PI was restarted lately,
	where as other 3 pins shows PI status. The status on which PI is currently at,
	is represented by bliking of the respective LED 
	It utilizes WTimer0 of TM4C123G for its internal timing purposes.
	If program doesn't receive any status from PI within 60 seconds,
	program will re-initialize. 
	
	The system also gets the updated time every second with the help of GPS.
	It is attached to UART1 and NMEA sentences are stored in ring buffer through
	UART interrupt. Time information is extracted by data parsing when sentence
	terminates.
	
	It generates its own pulse to tell PI when to transmit data. It syncs itself
	every 30 minutes with PPS of GPS to eliminate timer drifts. WTimer 1 is used
	to monitor the resync timeout and Timer 1 is used to set the pulse to zero
	
	
	Connections:
	PD0			->	Pi status toggle Pin
	PB1			->	LED1 (for representing whether Pi was restarted lately)
	PB2			->	LED2
	PB3			->	LED3
	PB4			->	LED4
	GND			->	Pi Ground
	
	PC4(U1RX)->	GPS TX
	PC6			-> 	GPS PPS
	PC7   	-> 	pi GPIO pin (to tell pi to initiate transmission)
	3.3V 		-> 	GPS VIN pin
	GND  		-> 	GPS GND
	
***********************************************************/ 

 
 
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
#include "inc/tm4c123gh6pm.h"
#include "inc/SysTick.h"
#include "inc/lcd.h"
#include "strfunc.h"
#include "uartConfiguration.h"
#include "utils/scheduler.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "adafruitUltimateGPS.h"

//systick frequency(for systick interrupt timing), set to 200Hz
#define TICKS_PER_SECOND 200				
#define PI_TIMER_TIMOUT		60			//timeout in seconds after which to reset PI in case of no state change																		
#define PI_TOTAL_STATES		4				//total number of states/LED
#define PPS_TIMEOUT				1				//1 second
#define PPS_RESYNC_TIME		1800			//1800sec=30minutes: time after which  internal timer has to resync with GPS PPS
#define PPS_HIGH_PULSE_TIME	100		//time in ms for the high pulse of the PPS pulse



//---------------------
// Function prototypes
//---------------------

//void configWtimer(void);		//configur
void configWatchDog(void);
void piCurrStateBlink(void *pvParam);

//------------------
// Global Variables/
//------------------


bool piStatus[PI_TOTAL_STATES];		//bool array to store the status of all the states
int piCurrState=0;								//variable to store the current status	
int piCurrStateToggle=0xFF;				//variable to store the logic on the blinking (current) status 
bool PPSInterruptOccured=false;		//bool variable to store whether pps interrupt has occured or not after resync request
bool ResyncTimeout=true;					//flag to state whether timeout for resync with PPS as occured or not
bool PPSTimerEnabled=false;				//flag to  store whether timer for PPS is enabled or not
uint32_t lastSyncWithPPS=0;				//variable to store time elapsed after the last resync with PPS

tSchedulerTask g_psSchedulerTable[] =	//Table consisting of tasks(functions) for scheduler to perform
{

	{piCurrStateBlink, (void*)0, TICKS_PER_SECOND/10, 0, false},				//blinks the current status LED every 10th of sec
};

uint32_t g_ui32SchedulerNumTasks = (sizeof(g_psSchedulerTable) / sizeof(tSchedulerTask)); //no of tasks


//------------------------------
//////////Function Definitions\\\\\\\\\\
//-------------------------------


void configPiStatusGPIO()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);			//enabling PORTD peripherals
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);		//setting Port D pin 0 to input type
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);//setting int to both edge(will be called at toggle of status)
	ROM_IntEnable(INT_GPIOD);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_0);		//enabling the interrupt
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);		//enabling port B peripheral
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, 0xFE);		//setting Port B pin 1-7 to output type
	GPIOPinWrite(GPIO_PORTB_BASE, 0xFE, 0x00);			//set PB0-7 pins to 0
	
}

void configPiStatusTimer()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);		//enabling wide timer0 peropheral
	ROM_TimerDisable(WTIMER0_BASE, TIMER_A);							//disabling the timer for proper configuration
	ROM_TimerIntDisable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);//disabling timer
	TimerClockSourceSet(WTIMER0_BASE,TIMER_CLOCK_SYSTEM);	//configuring timer clock source as of system	
	ROM_TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC);	//setting timer to be periodic down timer

	TimerLoadSet64(WTIMER0_BASE, ( (uint64_t) (ROM_SysCtlClockGet() )*PI_TIMER_TIMOUT)); //loading value of 1 minute

	ROM_IntMasterEnable();																//enabling he master interrupt
	ROM_IntEnable(INT_WTIMER0A);													//enabling wide timer0 interrupt
	ROM_TimerIntEnable(WTIMER0_BASE,TIMER_TIMA_TIMEOUT);	
	ROM_TimerEnable(WTIMER0_BASE, TIMER_A);								//enabling timer

}

//interrupt on detecting toggle on status pin
void piStatusGPIOInterrupt()				
{
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_0); //clear interrupt
	TimerLoadSet64(WTIMER0_BASE,( (uint64_t) (ROM_SysCtlClockGet() )*PI_TIMER_TIMOUT) ); //reload timer value
	if (++piCurrState== PI_TOTAL_STATES) //resetting pin states when all work is done
		piCurrState=1;
	
	if (piCurrState !=1)		//updating status LED's
		GPIOPinWrite(GPIO_PORTB_BASE, 1<<piCurrState, 1<<piCurrState);
	else
		GPIOPinWrite(GPIO_PORTB_BASE, 0xFE, 0x00);
	
}

//Toggle the current status pin LED
void piCurrStateBlink(void *pvParam)
{
	piCurrStateToggle= ~piCurrStateToggle;
	GPIOPinWrite(GPIO_PORTB_BASE, 0x02<<piCurrState, piCurrStateToggle);
}

//interrupt on timer timeout
void PiStatusTimerInterrupt()
{
	ROM_TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//g_psSchedulerTable[0].bActive=false;
	GPIOPinWrite(GPIO_PORTB_BASE, 0xFE, 0x00);
	//ROM_TimerDisable(TIMER0_BASE, TIMER_A);
	
	piCurrState=0;
  piCurrStateToggle=0xFF;
	///////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	////////////////ENTER CODE TO RESET PI HERE///////////////////////////
	/////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	
}


void configPPSGPIO()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);			//enabling PORTC peripherals
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);		//setting Port C pin 6 to input type
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_RISING_EDGE);//setting int to rising edge(will be called at from low to high change)
//	ROM_IntEnable(INT_GPIOC);
//	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_6);		//enabling the interrupt
	

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_7);		//setting Port C pin 7 to output type
	GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_7, 0x00); //writing 0 to PC7
}

void configPPSTimer()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);		//enabling wide timer0 peropheral
	ROM_TimerDisable(TIMER0_BASE, TIMER_A);							//disabling the timer for proper configuration
	ROM_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);//disabling timer
	TimerClockSourceSet(TIMER0_BASE,TIMER_CLOCK_SYSTEM);	//configuring timer clock source as of system	
	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);	//setting timer to be periodic down timer

	TimerLoadSet(TIMER0_BASE, TIMER_A,  ((ROM_SysCtlClockGet() )* PPS_TIMEOUT )); //loading value of 1 minute

	ROM_IntMasterEnable();																//enabling he master interrupt
	ROM_IntEnable(INT_TIMER0A);													//enabling wide timer0 interrupt
	ROM_TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);	
//	ROM_TimerEnable(TIMER0_BASE, TIMER_A);								//enabling timer
	
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);		//enabling wide timer0 peropheral
	ROM_TimerDisable(TIMER1_BASE, TIMER_A);							//disabling the timer for proper configuration
	ROM_TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);//disabling timer
	TimerClockSourceSet(TIMER1_BASE,TIMER_CLOCK_SYSTEM);	//configuring timer clock source as of system	
	ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);	//setting timer to be periodic down timer

	TimerLoadSet(TIMER1_BASE, TIMER_A,  ((ROM_SysCtlClockGet() /1000 )* PPS_HIGH_PULSE_TIME )); //loading value of 100ms

	ROM_IntMasterEnable();																//enabling he master interrupt
	ROM_IntEnable(INT_TIMER1A);													//enabling wide timer0 interrupt
	ROM_TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);	
	

}

void syncWithGPSPPS()
{
			TO_PC("wtng 4 pps\r\n");
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_6);
	ROM_IntEnable(INT_GPIOC);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_6);		//enabling the interrupt

	while (!PPSInterruptOccured)
	{
	}
		
	ROM_IntDisable(INT_GPIOC);
	GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_6);		//disabling the interrupt
	
	PPSInterruptOccured=false;
	ResyncTimeout=false;
		
	
}


void PPSGPIOInterrupt()
{
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_6);
	PPSInterruptOccured=true;
	TimerLoadSet(TIMER0_BASE, TIMER_A,  ((ROM_SysCtlClockGet() )* PPS_TIMEOUT ));
	if (!PPSTimerEnabled)
		ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	
	lastSyncWithPPS=0;
	TO_PC("pps ocrd\r\n");
}

void PPSHighTimerInterrupt()
{
	
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //writing 1 to PC
	//delay_ms(100);
	ROM_TimerEnable(TIMER1_BASE, TIMER_A);

	if (++lastSyncWithPPS >= PPS_RESYNC_TIME)
		ResyncTimeout=true;
		TO_PC("tmr intrpt\r\n");
}

void PPSLowTimerInterrupt()
{
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00);
	TimerLoadSet(TIMER1_BASE, TIMER_A,  ((ROM_SysCtlClockGet() /1000 )* PPS_HIGH_PULSE_TIME )); //loading value of 100ms
}


//------------------------------
//////////Main Function\\\\\\\\\\
//-------------------------------


int main()
{
	
	//initializations
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | 
                       SYSCTL_XTAL_16MHZ); //setting system frequency 80MHz
  ROM_FPULazyStackingEnable();

	SysTick_Init();
	SchedulerInit(TICKS_PER_SECOND);				//initialize scheduler
//	configWatchDog();												//initialize watchdog
	configUART();								//initializing UART used by the application
	                                                     initGPS();
	TO_PC("initialized\r\n");
	
	configPiStatusGPIO();
	configPiStatusTimer();
	configPPSGPIO();
	configPPSTimer();
		TO_PC("configured\r\n");
	
	
	while (1)
	{
		if (ResyncTimeout)
			syncWithGPSPPS();
		if (isGPSSentenceComplete() )
		{ 
//			TO_PC("comp\r\n");
			GPSGetData();
//			TO_PC("receivd\r\n");
			if (isGPSDataUpdated())
			{
				TO_PC("\r\n\n");
				TO_PC(GPSData->Time);
				
				TO_PC("\r\n\n");
				delay_ms(100);
			}
			
		}
		
	}
	
	
}//end main





//----------------------
// Function definitions
//----------------------





void configWatchDog() //configure watchdog timer, with resetting capability after 2nd timeout
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
	ROM_IntMasterEnable();
	ROM_IntEnable(INT_WATCHDOG);
	ROM_WatchdogReloadSet(WATCHDOG0_BASE, ROM_SysCtlClockGet()*20);//setting watcdog timeout to 20 seconds
	ROM_WatchdogResetEnable(WATCHDOG0_BASE);
	ROM_WatchdogEnable(WATCHDOG0_BASE);
	ROM_IntPrioritySet(INT_WATCHDOG,3);
}


void WatchdogIntHandler(void)
{    
  ROM_WatchdogIntClear(WATCHDOG0_BASE); // Clear the watchdog interrupt
	delay_us(1);
}


