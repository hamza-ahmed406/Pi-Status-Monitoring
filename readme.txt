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
