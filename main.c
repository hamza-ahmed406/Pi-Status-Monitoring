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
		
	Connections:
	PD0->Pi status toggle Pin
	PB1->LED1 (for representing whether Pi was restarted lately)
	PB2->LED2
	PB3->LED3
	PB4->LED4
	GND-> Pi Ground
	
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


//systick frequency(for systick interrupt timing), set to 200Hz
#define TICKS_PER_SECOND 200				
#define PI_TIMER_TIMOUT		60																			
#define PI_TOTAL_STATES		4																		

//---------------------
// Function prototypes
//---------------------

void configWtimer(void);
void configWatchDog(void);
void piCurrStateBlink(void *pvParam);

//------------------
// Global Variables/
//------------------


bool piStatus[PI_TOTAL_STATES];
int piCurrState=0;
int piCurrStateToggle=0xFF;
bool piCurrStateWorking=true;

tSchedulerTask g_psSchedulerTable[] =	//Table consisting of tasks(functions) for scheduler to perform
{

	{piCurrStateBlink, (void*)0, TICKS_PER_SECOND/10, 0, true},				//blinks the current status LED every 10th of sec
};

uint32_t g_ui32SchedulerNumTasks = (sizeof(g_psSchedulerTable) / sizeof(tSchedulerTask)); //no of tasks


//------------------------------
//////////Function Definitions\\\\\\\\\\
//-------------------------------


void configPiStatusGPIO()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);			//enabling PORTH peripherals
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);		//setting Port H pin 0 to input type
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);//setting int to falling edge(will be called at from high-to-low change)
	ROM_IntEnable(INT_GPIOD);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_0);		//enabling the interrupt
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, 0xFE);		//setting Port H pin 0 to input type
	GPIOPinWrite(GPIO_PORTB_BASE, 0xFE, 0x00);
	
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

int d,e;

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
	configWatchDog();												//initialize watchdog
	configUART();								//initializing UART used by the application
	
	configPiStatusGPIO();
	configPiStatusTimer();
	while (1)
	{
		d=GPIO_PORTD_DATA_R;
		e=GPIO_PORTB_DATA_R;
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

