////============================================
///	Code by: Muneeb Abid
//	Dated: 22 Feb 2015
///	Details: UART Basic Functions
////============================================


void Init_UART(void);

unsigned char UARTn_RxCh(int n);

unsigned char UARTn_RxChNonBlocking(int n);

void UARTn_TxCh(int n,unsigned char data);

void UARTn_TxStr(int n,char* string);
