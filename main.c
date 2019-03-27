
/*
 * GPS-UART-BUF.c
 *
 * Created: 22/03/2019 5:37:46 PM
 * Author : Madiva
 */ 
#define F_CPU 1000000UL
#include <string.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ctype.h> //for identifying digits and alphabets
#include <avr/wdt.h>

#include "24c64.h"
#include "USART.h"
#include "sdcard.h"

#define FOSC 2000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (((FOSC / (BAUD * 16UL))) - 1)

char byteGPS=-1;
char linea[100] = "";
char satellites[2] = "";
char IP[21] = "";
char MemIP[23] ="";
char FinalIP[23] = "";
char ExlonIP[] ="XXX.XXX.XXX.XXX\",\"XXXX";
char ID[] = "201800XXX"; //003-KCR , 002-KBY, 001-KCN, 004-KCQ777T, 005-KCQ-885W
uint8_t EEmemory[22] = "";

char comandoRMC[7] = "$GPRMC";
char comandoGGA[7] = "$GPGGA";
char *RMC, *GGA, w, L;
char status[4], phone[13], owner[13], *Digits; 
int GPS_position_count=0;
int datacount=0, datacount1=0;
int x, y, a, i, b, H, R, S=0, p=0;
int cont=0, bien=0, bien1=0, conta=0, address;
char fix; int e;
uint8_t failed;

char input;
char buff[20];
char company[]	= "+254XXXXXXXXX";
char CarOwner[13];
char company2[]	= "+254XXXXXXXXX";

//static FILE uart0_output = FDEV_SETUP_STREAM(USART0_Transmit, NULL, _FDEV_SETUP_WRITE);
static FILE uart1_output = FDEV_SETUP_STREAM(USART1_Transmit, NULL, _FDEV_SETUP_WRITE);
static FILE uart1_input = FDEV_SETUP_STREAM(NULL, USART1_Receive, _FDEV_SETUP_READ);
static FILE uart0_input = FDEV_SETUP_STREAM(NULL, USART0_Receive, _FDEV_SETUP_READ);

void WDT_off(void);
void WDT_Prescaler_Change(void);

void HTTPTransmit1 ();
void HTTPTransmit2 ();
void grabGPS();
unsigned char CheckSMS();
void checknewSMS();
unsigned char sender();
unsigned char CompareNumber();
void PrintSender();
void CreateDraft(char m);
uint8_t IP_Change_Command(); //Text => $IP:PORT
void StoreIP (char *NewIP);
void Change_owner(); //to change owner's no# send => #+254XXXXXXXXX#
int watchdog_delay(int i);
void Trans_Delay(); //To change delay time text-> &X , where X is either [1-4] : 1-1min,2-10min,3-30min,4-1hr
void Blink_LED();
void SendConf_Text();
void Track(); //send text to request for Location <LOC>

/*current EEPROM Structure
#--0[IP+PORT]23--25[vehicle_status]--26[sender_Mobile]39--40[Owner_Mobile]53---54[Trans_Delay]--55[Text status]
*/
#define CAR_ON	PORTD |= (1<<PORTD5)
#define CAR_OFF	PORTD &= ~(1<<PORTD5)
#define ACC_ON	PORTC |= (1<<PORTC4)
#define ACC_OFF	PORTC &= ~(1<<PORTC4)
#define	wakeGSM	PORTA |= (1<<PORTA5)
#define	wakeGSM1	PORTA &= ~(1<<PORTA5)
#define	RST_GSM1	PORTA |= (1<<PORTA4)
#define	RST_GSM2	PORTA &= ~(1<<PORTA4)
#define CHOSEN_SECTOR 0
#define BUFFER_SIZE 24

/**************AVR_WDT********************/
// Function Prototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
// Function Implementation
void wdt_init(void)
{
	MCUSR = 0;
	wdt_disable();
	//MCUSR &= ~(1<<WDRF); //clear Watchdog sys rst flag - should be done even if WD is not used
	//WDTCSR &= ~(1<<WDE); //clear WD timer WD Reset Enable
	return;
}

/**********************************
A simple delay routine to wait
between messages given to user
so that he/she gets time to read them
***********************************/
void Wait()
{	uint8_t i;
	for(i=0;i<100;i++)
	_delay_loop_2(0);
}

void setup()
{
	DDRD |= (1<<DDD5); //set PORTA3 as output for LED
	DDRA &= ~(1<<DDA0); //set PORTA0 as input RING pin
	DDRC |= (1<<DDC4); //set PORTB0 AS acc output
	DDRA |= (1<<DDA5); //set PORTB0 AS GSM-power output
	DDRA |= (1<<DDA4); //set PORTB0 AS GSM-RST output

	
/*	cli();*/
	CAR_OFF;
	ACC_OFF;
	
	char L;
	EEOpen();
	_delay_loop_2(0);
	L = EEReadByte(25);
	if (L== 0x030) //If vehicle status is '0' Then switch OFF the vehicle
	{ CAR_OFF; ACC_OFF; status[0]= L;}
	else if (L == 0x031) //If vehicle status is '1' Then switch ON the vehicle
	{ CAR_ON; ACC_ON; status[0]= L;}
	else {}
	//Powering up the GSM	
	wakeGSM;
	_delay_ms(4000);
	wakeGSM1;
	_delay_ms(3000);
	
	//Enable interrupts at this stage cause it was firing off prematurely when GSM boots up above
	PCICR	|= (1<<PCIE0); //set interrupt enable
	PCMSK0	|= (1<<PCINT0); //set interrupt mask bit on PA0
	WDT_Prescaler_Change(); //change wdt prescaler to 8 Sec
	
}

int main( void )
{
	USART1_Init(MYUBRR);
	USART0_Init(MYUBRR);
	setup();
	//Below 2 lines ensure the GSM is reset successfully, otherwise delay >8sec to trigger RESET
	if((PINA & (1<<PINA0))){} 
	else{_delay_ms(10000);}
	//sei(); //Enable global interrupts by setting the SREG's I-bit
	
	fdev_close();
	stdin = &uart0_input;
	stdout = &uart1_output; //changed to TX1 for GSM communication. TX0 on Atmega SMD isn't working
	_delay_us(500);	
		
	//*********************************GRAB OWNER'S NUMBER********************************************//
	EEOpen(); //grab the owner's no# from memory before matching it with the sender
	_delay_loop_2(0);
	for(address=40;address<53;address++) //Now grab the previous sender's no#
	{	L = EEReadByte(address);
		CarOwner[address-40] = L;
	}
	CarOwner[13]=0x00;
	//***********************************************************************************************//
		/* Max clock frequency 8MHz (pre-scaler 0) */
		//CLKPR = (1 << CLKPCE);	/* Enabled clock pre-scaler change */
		//CLKPR = 0; /* Write pre-scaler to 0 */
	watchdog_delay(1);
	putchar(0x1A); //putting AT-MSG termination CTRL+Z in USART0
	watchdog_delay(2);
	printf("AT+CFUN=1\r\n"); watchdog_delay(4);
	
	int T;
	EEOpen();
	_delay_loop_2(0);
	T = EEReadByte(55);
	if (T==1) { SendConf_Text();} //if the send text flag was set please send vehicle status to sender
	else{}
		
	checknewSMS();
	S=0;	
 	
	fdev_close();
	stdout = &uart1_output;
	stdin = &uart1_input;		
	while(1)
	{ // start:
		HTTPTransmit1 ();
		watchdog_delay(1);
		
		grabGPS();
		watchdog_delay(1);
		
		HTTPTransmit2 ();
		_delay_ms(500);
		putchar(0x1A); //putting AT-MSG termination CTRL+Z in USART just in case it hangs at message or TCP sending
		watchdog_delay(1);
		
/****************Adding Delay on Transmission*************/
		char q; 
		EEOpen();
		_delay_loop_2(0);
		q = EEReadByte(54);
			//NOTE: transmission by default happens after around 40sec
			if	   (q==0x31) { watchdog_delay(10);}		//1-1min,
			else if(q==0x32) { watchdog_delay(280);}	//2-10min
			else if(q==0x33) { watchdog_delay(880);}	//3-30min
			else if(q==0x34) { watchdog_delay(1780);}	//4-1hr
			else   {}
/*********************************************************/
		S++;
		if (S==100)
		{ S=0; _delay_ms(15000);} //This will reset the device after every 100 transmissions
		else { }
	}
	return 0;
}

ISR(PCINT0_vect)
{
	fdev_close();
	stdout = &uart1_output;
	stdin = &uart0_input;
	watchdog_delay(1);
	
	//*********************************GRAB OWNER'S NUMBER********************************************//
	EEOpen(); //grab the owner's no# from memory before matching it with the sender
	_delay_loop_2(0);
	for(address=40;address<53;address++)
	{	L = EEReadByte(address);
		CarOwner[address-40] = L;
	}
	CarOwner[13]=0x00;
	//***********************************************************************************************//
//	watchdog_delay(1);
	putchar(0x1A); //putting AT-MSG termination CTRL+Z in USART0
	watchdog_delay(3);
	 
	if((PINA & (1<<PINA0))){checknewSMS();}
	else
	{ printf("ATA\r\n"); watchdog_delay(15); }
	
	_delay_ms(500);
	fdev_close();
	stdout = &uart1_output;
	stdin = &uart1_input;
	_delay_ms(500);wdt_reset();
	HTTPTransmit1 ();
	_delay_us(500);wdt_reset();
	grabGPS();
	_delay_us(500);wdt_reset();
	HTTPTransmit2 ();
	_delay_us(2000);wdt_reset();
}

int watchdog_delay(int i)
{
	for (int g=0; g<i; g++)
	{ wdt_reset(); _delay_ms(2000);}
}

void Blink_LED()
{ CAR_ON; watchdog_delay(1); CAR_OFF; watchdog_delay(1);}

unsigned char CheckSMS()
{
	int z = 0, T=0; //char w;
	y=0; p=100;
	a=0;
	printf("AT\r\n");
  	watchdog_delay(1);
	printf("AT+CMGF=1\r\n");
  	watchdog_delay(1);
	printf("AT+CMGL=\"REC UNREAD\"\r\n");
	while (a < 2) //skip the <LF>
	{//
		w = getchar();
		if (w==0x0A) { a++; }
		else {}
	}//
	w = getchar();
	if (w==0x02B) // if w = +
	{
		sender();
		w = getchar();
		while (w !=  0x0A) //w is not <LF>
		{ w = getchar();}
		
		w = getchar();
		if (w==0x030 || w==0x031)//w is '0' or '1'
		{
			CompareNumber();
			z = status[1] + status[2] + status[3]; //sum values of the 3 buffer values
			if (z < 3) //A scenario of receiving text from an authorized no# with '1' or '0'
			{	status[0] = w;
				if ( w==0x030)		//If the text received is a '0'
					{CAR_OFF; ACC_OFF;	EEOpen(); _delay_loop_2(0); EEWriteByte(25,0x030); T=1;}//Write New Vehicle STATUS in EEPROM			
				else if ( w==0x031) //If the text received is a '1'
					{CAR_ON; ACC_ON;	EEOpen(); _delay_loop_2(0); EEWriteByte(25,0x031); T=1;}//Write New Vehicle STATUS in EEPROM
				else{}
				CreateDraft(w);
			}
			else //A scenario of receiving text from Unauthorized no#
			{	printf("AT+CMGD=1,2\r\n"); } //clearing all SMS in storage AREA except Draft and UNREAD SMS
		 }
		else if (w==0x24) //If a $ is received
		{	IP_Change_Command(); status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); T=1;}

		else if (w==0x23) //If a # is received
		{	Change_owner();  status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); T=1;}
		
		else if (w==0x26) //If a & is received
		{	Trans_Delay(); status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); T=1;}

		else if (w==0x3C) //If a < is received
		{	Track(); status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); T=1;}
			
		else
		{	status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); } //clearing all SMS in storage AREA except Draft and UNREAD SMS
	}
	else if(w==0x04F) // if w = 'O'
	{
			w = getchar();
			if (w==0x04B) // if w = 'K'  If there is no new message
			{	status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); }
			else
			{	status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); }
	}
	else {	status[1] = status[2] = status[3] = 0; printf("AT+CMGD=1,2\r\n"); }
		
	/////////////////////////////////////////////////////
		
	if (T == 1 ) //write sender's no# in EEPROM only if its a valid SMS 
	{
		EEOpen();
		_delay_loop_2(0);
		//Write Sender no# in EEPROM
		for(address=26;address<39;address++)
		{ EEWriteByte(address,phone[address-26]); }
	}
	////////////////////////////////////////////////////
	int L;
	EEOpen();
	_delay_loop_2(0);
	L = EEReadByte(55);
	if (L==1) { _delay_ms(10000);} //if the send text flag was set reset device & send vehicle status to sender
	else{}
	
		
	return *status;
}

void checknewSMS()
{
		watchdog_delay(1);
		putchar(0x1A); //putting AT-MSG termination CTRL+Z in USART0
		watchdog_delay(2);
		printf("AT\r\n");
		watchdog_delay(1);
		printf("AT+CMGF=1\r\n");
		watchdog_delay(1);
		printf("AT+CPMS=\"MT\",\"SM\",\"ME\"\r\n");
		watchdog_delay(1);
		printf("AT+CPMS?\r\n");
			failed =0;
			int q=0, M=0;;
			w = getchar();
			while (q<7)
			{	w = getchar();
				if (w==0x2C) { q++;} //if w is "," in the string +CPMS: "MT",1,75,"SM",1,25,"ME",0,50
				if (w==0x52) { M++; if(M==2){failed = 1; q=7;} else{}} //if u meet 2X'R' in ERROR abort process
				else{}
			}
			if (!failed)
			{
				w = getchar();
				if (w != 0x30)	{CheckSMS();}
				else {}
			} 
			else
			{printf("AT+CFUN=1\r\n"); _delay_ms(15000); }
		watchdog_delay(1);
}

void CreateDraft(char m)
{
	printf("AT+CMGD=1,4\r\n"); //clearing all SMS in storage AREA
	watchdog_delay(1);
	printf("AT+CMGW=\"");
	PrintSender();
	printf("\",145,\"STO UNSENT\"\r\n%c",m);
	watchdog_delay(1);
	putchar(0x1A); //putting AT-MSG termination CTRL+Z in USART0
	watchdog_delay(1);
}

unsigned char sender()
{
	int n;
	w = getchar();
	while (w != 0x02B) // while w is not +
	{ w = getchar(); }
	for (n=0; n<13; n++) //capture 13 digit phone number
	{	phone[n] = w; w = getchar(); }
	
	return *phone;
}

void PrintSender()
{
	int n;
	for (n=1; n<13; n++) //for 13 digit phone number //something fishy :D if u begin from n=0 we'll have ++ before the no#
	{	printf("%c", phone[n]);}
}

unsigned char CompareNumber()
{
	int j;
	status[1] = status[2] = status[3] =0;
	
	for (j=0; j<13; j++)
	{
		if (phone[j]!=company[j])
		{ status[1] = 1;}
		else{}
		if (phone[j]!=company2[j])
		{ status[2] = 1;}
		else{}
		if (phone[j]!=CarOwner[j])
		{ status[3] = 1;}
		else{}
	}
	return *status;
}

void HTTPTransmit1 ()
{
	printf("AT\r\n");
  	watchdog_delay(1);
	printf("AT+CGATT=1\r\n");
	watchdog_delay(1);
	printf("AT+CIPMUX=0\r\n");
	watchdog_delay(1);
	printf("AT+CSTT=\"safaricom\",\"\",\"\"\r\n");
	watchdog_delay(1);
	printf("AT+CIICR\n");
	watchdog_delay(2);
	printf("AT+CIFSR\r\n");
	watchdog_delay(2);
//********************* READ IP STORED IN EEPROM ******************************************************

		EEOpen();
		_delay_loop_2(0);
		for(address=0;address<23;address++)
		{	w = EEReadByte(address);
			EEmemory[address]=w;
		}		
		if(EEmemory[0]!='\0')
			{
				printf("AT+CIPSTART=\"TCP\",\"%s\"\r\n", EEmemory);
				watchdog_delay(2);
			}		
		
		else
			{
				printf("AT+CIPSTART=\"TCP\",\"%s\"\r\n", ExlonIP); //muad
				watchdog_delay(2);
			}
	printf("AT+CIPSEND\r\n");
	watchdog_delay(1); _delay_ms(500);
}

void HTTPTransmit2 ()
{
	printf("\r\n\r\nAT+CIPCLOSE\r\n");
	watchdog_delay(1);
	printf("AT+CIPSHUT\r\n");
	watchdog_delay(1);
}

void grabGPS()
{
  y=0; x=0;
  while(x==0 || y==0)
	{
	byteGPS=getchar();
	// Read a byte of the serial port
	if (byteGPS == -1)
	{  /* See if the port is empty yet*/ }
	else
	{
		// note: there is a potential buffer overflow here!
		linea[conta]=byteGPS;        // If there is serial port data, it is put in the buffer
		conta++;
		datacount++;
		datacount1++;
		
		//Serial.print(byteGPS);    //If you delete '//', you will get the all GPS information
		
		if (byteGPS==13)
		{
			// If the received byte is = to 13, end of transmission
			// note: the actual end of transmission is <CR><LF> (i.e. 0x13 0x10)
			cont=0;
			bien=0;
			bien1=0;
			// The following for loop starts at 1, because this code is clowny and the first byte is the <LF> (0x10) from the previous transmission.
			for (int i=1;i<7;i++)     // Verifies if the received command starts with $GPGGA
			{
				if (linea[i]==comandoGGA[i-1])
				{bien++;}
			}
			for (int i=1;i<7;i++)     // Verifies if the received command starts with $GPRMC
			{
				if (linea[i]==comandoRMC[i-1])
				{bien1++;}
			}
			
			//-----------------------------------------------------------------------
			if(bien==6) // If initial characters match "+GPGGA" string, process the data
			{
				//linea[0]='\n';
				int p=0;
				int u=0;
				e=0;
				satellites[0]='\0';
				satellites[1]='\0';

			for (int i=1;i<datacount;i++)
			{
				if (linea[i]==0x2C)
				{e++; if (e==6) {p=i;} if (e==7) {u=i;}}	
			}			
				fix=linea[p+1];
				satellites[0]=linea[u+1];
				satellites[1]=linea[u+2];
							
				GGA = (char*) malloc(datacount+1);			//allocating memory to the GGA string, where datacount is length of GPS GGA string
				strcpy(GGA,linea);							//using function strcpy to copy the char array(linea) to GGA string which we have allocated memory above
				y=1;										//one half of the conditions to exit while(x==0 || y==0) function
			}
			if(bien1==6) // If initial characters match "+GPRMC" string, process the data
			{
				//linea[0]='\n';
				linea[0]=0x2D;
				linea[datacount1]=0x00;
				linea[datacount1-1]=0x00;
				RMC = (char*) malloc(datacount1+1);
				strcpy(RMC,linea);
				x=1;
			}

			if (x==1 && y==1)
			{	
				EEOpen();
				_delay_loop_2(0);
				char L;
				for(address=40;address<53;address++) //Now grab the owner's no#
				{	L = EEReadByte(address);
					CarOwner[address-40] = L;
				}
				CarOwner[13]=0x00;
				Digits = (char*) malloc(13);//allocate memory for the Owner's phone number
				strcpy(Digits,CarOwner);
				
				_delay_ms(500);

				printf("\r\nID=%s&String=%c:%s:%c:%c%c:%s\"\r\n",ID, status[0],Digits,fix, satellites[0], satellites[1], RMC);
				
				_delay_ms(500);
				putchar(0x1A); //putting AT-MSG termination CTRL+Z in USART0
				watchdog_delay(2);

				free(GGA); //The memory location pointed by GGA is freed. Otherwise it will cause error
				free(RMC); //The memory location pointed by RMC is freed. Otherwise it will cause error
				free(Digits);
				_delay_ms(500);
			}
			else{}
			//-----------------------------------------------------------------------
			conta=0;
			datacount=0;
			datacount1=0;
			// Reset the buffer
			for (int i=0;i<100;i++)
			{    //
				linea[i]='\0';
			}
		} //byteGPS==13
	}  //else byteGPS is not null
} //Serial1.available
//return 0;
} //testGPS

uint8_t IP_Change_Command()
{
	int Flag=0, L=0, F=0, U=0;
	char number;
	
	while(Flag==0)
	{
		number=getchar();
		if (L<21 && isdigit(number) )
		{ MemIP[L+U]= number; L++; } //U is when i add "," in the IP string
		else
		{
			if (number == 0X2E)// && F<3) //if "." is captured and are less than 3
			{ MemIP[L+U]= number; F++; L++;}
			else if ( number == 0X3A)// && F==3) //if number ":" && 3X "." must have been captured
			//{ MemIP[L]= number; U++; L++;}
			{ MemIP[L]= 0X22;  MemIP[L+1]= 0X2C; MemIP[L+2]= 0X22; U=2; L++;}
			else if (number==0x0D) // if number "CR"
			{ Flag=1; }
			else
			{ Flag=2;}
		}
	}
	
	while ((L+U) < 23)
	{
		MemIP[L+U]= 0X00; //write NULL in the remaining array slots
		L++;
	}

	StoreIP (MemIP);
	
	return *MemIP;
}

void StoreIP (char *NewIP)
{	
	//Init EEPROM
	EEOpen();
	_delay_loop_2(0);
	//Write New IP in EEPROM
	for(address=0;address<23;address++)
		{ EEWriteByte(address,NewIP[address]); }
}

void Trans_Delay() //1-1min,2-10min,3-30min,4-1hr
{
	int Flag=0;
	char number;
	
		number=getchar();
			if	   (number==0x31) { Flag=1;} 
			else if(number==0x32) { Flag=1;}
			else if(number==0x33) { Flag=1;}
			else if(number==0x34) { Flag=1;}
			else   { Flag=2;}
			
/******store delay time in EEPROM*******/			
	if (Flag==1)
	{   //Init EEPROM
		EEOpen();
		_delay_loop_2(0);
		//Write New Delay-Time in EEPROM
		EEWriteByte(54,number);
	} 
	else {}
}

void Change_owner()
{
	int Flag=0, L=0;
	char number;
	
	for(int G=0;G<13;G++) //clear the volatile owner no# buffer
		{owner[G]='\0';}
	
	while(Flag==0)
	{
		number=getchar();
		if (L<13 && isdigit(number) )
		{ owner[L]= number; L++; } //U is when i add "," in the IP string
		else
		{	
			if (number == 0X2B)//if its + in phone no# store it
				{if (L<2) {owner[L]= number; L++;} else {Flag=2;}}
			else {if (L>11) {Flag=1;} else {Flag=2;}}
		}
	}
////////	
	if (Flag==1) // if the text was valid, Write new owner no# in EEPROM
	{		
			EEOpen();	//Init EEPROM
			_delay_loop_2(0);
			for(address=40;address<53;address++)
			{	EEWriteByte(address,owner[address-40]); }
	} 
	else { }
////////		
	for(int G=0;G<13;G++) //clear the volatile owner no# buffer
		{owner[G]='\0';}
 }

void SendConf_Text()
{
	char line1[70] = "";
	int comma[4]; a=0;
	int p=0, j=0;
	for (int i=0;i<71;i++) { line1[i]='\0'; }
	watchdog_delay(2);
	printf("AT+SAPBR=?\r\n");watchdog_delay(1);
	printf("AT+SAPBR=1,1\r\n");watchdog_delay(1);
	printf("AT+CIPGSMLOC=?\r\n");watchdog_delay(2);
	printf("AT+CIPGSMLOC=1,1\r\n"); //+CIPGSMLOC: 0,36.833633,-1.304255,2019/03/22,10:08:02
	while (a < 2) //skip the 2 <LF>
	{	w = getchar();
		if (w==0x0A) { a++; }
		else {}
	}
	w = getchar();
	while (w !=  0x0A) /*while w is not <LF>*/
	{	line1[p]=w; if (w==0X2C){comma[j]=p; j++;}  else{}
	w = getchar(); p++;} //start storing GSM Loc data to Line1 string till <LF>

		watchdog_delay(1);

		printf("AT\r\n"); watchdog_delay(1);
		printf("AT+CMGF=1\r\n");watchdog_delay(1);

	//****************************Now grab the previous sender's no#****************************************//
		EEOpen();
		_delay_loop_2(0);
		for(address=26;address<39;address++)
		{	L = EEReadByte(address);
			phone[address-26] = L;
		}
		phone[13]=0x00;
	//****************Retrieve Transmission Delay*********************************************//
			char q; int T=0;
			_delay_ms(200);
			EEOpen();
			_delay_loop_2(0);
			q = EEReadByte(54);
			//NOTE: transmission by default happens after around 40sec
			if	   (q==0x31) { T=1;}	//1-1min,
			else if(q==0x32) { T=10;}	//2-10min
			else if(q==0x33) { T=30;}	//3-30min
			else if(q==0x34) { T=60;}	//4-1hr
			else   {}
	/*********************************************************/
		watchdog_delay(1); printf("AT+CMGS=\"%s\"\r\n", phone); watchdog_delay(1);

		printf("ID: %s || Owner: %s || Interval: %d Minutes\r\n(~200m Accurate) http://maps.google.com/maps?q=", ID, CarOwner, T); //Location ~200m radius Accurate:
		p=comma[1]; j=comma[2]; for (int i=p+1;i<j;i++) { printf("%c", line1[i]);} _delay_ms(50); printf(","); _delay_ms(50);
		p=comma[0]; j=comma[1]; for (int i=p+1;i<j;i++) { printf("%c", line1[i]);} _delay_ms(50); printf("\r\n"); _delay_ms(50);
		p=comma[2]; for (int i=p+1;i<70;i++) { printf("%c", line1[i]);}
		_delay_ms(500);
		putchar(0x1A); //putting AT-MSG termination CTRL+Z in USART0
		watchdog_delay(2);

		for (int i=0;i<70;i++)
		{ line1[i]='\0'; }
		for (int i=0;i<5;i++)
		{ comma[i]='\0'; }
		//Clear the Text flag in EEPROM	
		EEOpen();
		_delay_loop_2(0);
		//Set flag to send location data later
		EEWriteByte(55,0);
	}  


void WDT_off(void)
{
	cli();
	wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE (WD Change Enable) and WDE (WD Reset Enable) */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE); //WDTCSR (WD Timer Control Register)
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();
}

void WDT_Prescaler_Change(void)
{
	cli();
	wdt_reset();
	/* Start timed sequence */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Set new prescaler(time-out) value = 64K cycles (~8 s) */
	WDTCSR = (1<<WDE) | (1<<WDP3) | (1<<WDP0);
	sei();
}

void Track() //<LOCATION>
{
	char location[5];
	char match[]="LOC>";
	int Flag=0, p=0;
	w = getchar();
	while(Flag==0)
	{
		location[p]=w;
		p++;
		w = getchar();
		if (w==0X3E)// || p==9 || w==0X0A) //if w is ">",LF or we have reached 9 char
		{Flag=1;}
		else{}
	}
	location[p]=w;
	p=0;
	for (int y=0; y<4; y++) //compare the 2 strings
	{
		if (location[y]!=match[y]) {p++;}// printf("%c(%d)\r\n",location[y],y);}
		else{}
	}
	if (p==0)
	{ 
		/******Set Text Flag in EEPROM*******/
			//Init EEPROM
			EEOpen();
			_delay_loop_2(0);
			//Set flag to send location data later
		if (Flag==1)
			 {EEWriteByte(55,1);}
		else {EEWriteByte(55,0);}
	}
	
}
