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