/*********************************************************************************
 * Logomatic V2 Firmware is modified based on the PCBA developed by Peter.
 * Author: Muhammad Moiz khan
 * Developed for: Peter Barret
 
 * The firmware records the reading of Heel and FFT stores it into SD-Card and sends
   total output via Xbee.
   
 * The firmware also goes into calibration mode when CALIB_SWITCH is pressed within 
   first 5 seconds, when logomatic boots up, then CALIB_LED is flashed and then it
   give user 5 more seconds to press the switch for calibration. like 1 press = 10kg
   supports 4 presses
   
 TODO: Calibration need to tested on digipots
 * ******************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "LPC21xx.h" //LPC214x.h

//UART0 Debugging
#include "serial.h"
#include "rprintf.h"   

//Needed for main function calls
#include "main_msc.h"
#include "fat.h"
#include "armVIC.h"
#include "itoa.h"
#include "rootdir.h"
#include "sd_raw.h"
#include "string_printf.h"
#include "delay.h"

// PINS DESCRIPTION
#define Calib      22 //Calib Button (D5)
#define Calib_LED  25 //Calib_LED    (D4)
#define CS_1       28 //FFT digipot  (D3) 
#define CS_0       20 //CS1 on breakout board CS0  ?????  CS0 = P0.7, CS1 = P0.20
#define STAT1      11
#define STAT0      2      
//#define STEPS_DIGIPOT  50   //50 STEPS FOR DIGIPOTS
#define CALIB_TIME 	500    //10MS * 500 = 5000


/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size            : 8 bits    */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI       */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle high */  //(Needs high for SCP1000)
#define SSP_CPHA 0x00 << 7  /* clock phase          : 1         */
#define SSP_SCR  0x0F << 8  /* serial clock rate    : 58.59kHz = PCLK / (CPSDVSR * [SCR+1]) = 15000000 / (16 * [15+1]) */ 
#define PINSEL1_SCK  (2 << 2)
#define PINSEL1_MISO (2 << 4)
#define PINSEL1_MOSI (2 << 6)

/*******************************************************
 *         Global Variables
 ******************************************************/

#define ON  1
#define OFF 0
#define BUF_SIZE 512

char RX_array1[BUF_SIZE];
char RX_array2[BUF_SIZE];
char log_array1 = 0;
char log_array2 = 0;
short RX_in = 0;
char get_frame = 0;

signed int stringSize;
struct fat_file_struct* handle;
struct fat_file_struct * fd;
char stringBuf[256];

// Default Settings
static char   mode = 0;
static char    asc = 'N';
static int    baud = 9600;
static int    freq = 100;
static char   trig = '$';
static short frame = 100;
static char  ad1_7 = 'N';
static char  ad1_6 = 'N';
static char  ad1_3 = 'N';
static char  ad1_2 = 'N';
static char  ad0_4 = 'N';
static char  ad0_3 = 'N';
static char  ad0_2 = 'N';
static char  ad0_1 = 'N';


/*******************************************************
 *     Function Declarations
 ******************************************************/

void Initialize(void);

void setup_uart0(int newbaud, char want_ints);
void uart0_SendString (char* str);
void uart0_SendChar (char ch);

void mode_0(void);
void mode_1(void);
void mode_2(void);
void mode_action(void);

void Log_init(void);
void test(void);
void stat(int statnum, int onoff);
void AD_conversion(int regbank);

void feed(void);

//BHW Never defined... static void IRQ_Routine(void) __attribute__ ((interrupt("IRQ")));
static void UART0ISR(void); //__attribute__ ((interrupt("IRQ")));
static void UART0ISR_2(void); //__attribute__ ((interrupt("IRQ")));
static void MODE2ISR(void); //__attribute__ ((interrupt("IRQ")));

void FIQ_Routine(void) __attribute__ ((interrupt("FIQ")));
void SWI_Routine(void) __attribute__ ((interrupt("SWI")));
void UNDEF_Routine(void) __attribute__ ((interrupt("UNDEF")));

void fat_initialize(void);

void reverse(char* str, int len) ;
int intToStr(int x, char str[], int d) ;
void ftoa(float n, char* res, int afterpoint) ;





/******************************************************/
void clear_gpio(uint32_t pin);
void set_gpio (uint32_t pin);
void flash_CalibLED(void);

void sleep_xbee (void);
void wake_xbee (void);


//SPI prototypes
void SPI_Write(uint8_t data);
void SPI_Init();
char SPI_Read(void);

//SPI1 prototypes
void SPI1_Init (void);
void SPI1_Write(uint8_t data);
void programHeel_DIGIPOTS(uint8_t steps);
void programFFT_DIGIPOTS(uint8_t steps);


// XBEE MACRO AND VARIABLES
/*********************************************************/
uint16_t xbee_cnt = 0;
uint8_t XBEE_SEND_FLAG = 0;
static uint8_t WeightAvg[20];


int weight_Total=0;
float heel_weight=0.0, fft_weight=0.0;
uint8_t k=0, iter=0,total_WeightTemp=0;

#define XBEE_TICKS  10

/*********************************************************/
uint8_t SwFlag = 0;  //flag for maintain switch high and lows
uint8_t timerFLAG = 1; //you can start and stop timer using timerFLAG , 1 means start 0 means stop
uint16_t  SwCount=0; //it is timer counts.
uint16_t countH=0,countL=0; //debounce handling counts
uint16_t  swHighCount=0; //captures how many times switch have been pressed.
uint8_t calibrationModeFLAG = 0; //this flag tells whether we are in calibration mode or not.
uint8_t firstCapture=1, secondCapture =0;
uint8_t debugTemp[50]; //for printing it on uart0
uint8_t flash_Calib_led=1; //flashes the led on calibration
uint8_t programDigipots_FLAG = 0; //when it is high program digipots.
uint8_t STEPS_DIGIPOT = 25;  //digipots will be program will a step of 50.
/********************************************************/



/*******************************************************
 *              Super Loop
 ******************************************************/
//Fosc = 12 Mhz 
//CCLK = PCLK =  12x4=48Mhz , hence 48 Mhz is a peripheral clock
int main (void)
{
  int i;
  char name[32];
  int count = 0;
	
	enableFIQ();
	Initialize();
	
	//uart0 is used for xbee and no uart interrupt.
	setup_uart0(9600, 0);
	
	//initializing spi for digipots
	SPI1_Init();

	fat_initialize();  
	uart0_SendString("Welcome to logomatic\n\r");
 
	// Flash Status Lights
	for(i = 0; i < 5; i++)
	{
		//set_gpio(1<<Calib_LED);
		stat(0,ON);
		delay_ms(50);
		//clear_gpio(1<<Calib_LED);
		stat(0,OFF);
		stat(1,ON);
		delay_ms(50);
		stat(1,OFF);
	}
	Log_init();
	count++;  //BHW TODO: this makes the count start from 1, not 0 to match docs
	string_printf(name,"LOG%02d.txt",count);
			
	while(root_file_exists(name))
	{
		count++;
		if(count == 250)  //BHW TODO: This less than 250 limit doesn't match docs
		{
			  while(1)
			  {
					stat(0,ON);
					stat(1,ON);
					delay_ms(1000);
					stat(0,OFF);
					stat(1,OFF);
					delay_ms(1000);
					//uart0_SendString ("\r\n only toggling led...");
			  }
		
		}
		string_printf(name,"LOG%02d.txt",count);
	
	}
	//uart0_SendString ("\r\n\r\n\r\n Intialized  \r\n");
	handle = root_open_new(name);

	sd_raw_sync();  
	
	if(mode == 0)	
	{
		mode_0(); 
	}
	else if(mode == 1)
	{ 
		mode_1(); 
	}
	else if(mode == 2)
	{ 
		mode_2(); 
	}

	return 0;
}


/*********************************************************************************
PROCESSING OF ADC VALUES , ALGORITHM IMPLEMENTATION
			
This function gathers the adc values and apply algorithm in it, to make fft 
and heel values
Moreover compose the values of total weight and takes its average before sending
to xbee every 100ms
**********************************************************************************/
																								
static inline int pushValue(char* q, int ind, int value, volatile unsigned long* ADxCR, int mask)
{
	  char* p = q + ind;

	  if(asc == 'Y') // ASCII
	  {
			int NoOfBytes=0;
			
		
			// Below is the AD Control Register Address  AD0CR Address is E0034000  AD1CR Address is E0060000  
			//Conversion for heel
			/************************************* Gather value of A0.1 (HEEL WEIGHT) **********************************************/
			if ((ADxCR == 0xE0034000) && (mask == 8)) //ADxCR is the peripheral address 
			{
								
				
				heel_weight = (float)((value - 2.44)/1.1);    //2.44  // 1023 =  927.7818181818182
				heel_weight =  heel_weight / 4.0; 				//add upon request
		
				if (heel_weight > 0.0 ){
					
					ftoa( heel_weight, p , 1); 		
					
					//calculate the number of bytes....
					NoOfBytes = strlen(p) + ind + 1;		
				
				}
				else{
				
					heel_weight = 0.0;
					p[0]='0';
					p[1]='.';
					p[2]='0';
					p[3]='\0';
					//ftoa( heel_weight, p , 1); 		
					NoOfBytes = strlen(p) + ind + 1;					
				}										
			}
			/************************************* Gather value of A0.2 (FFT WEIGHT) ***************************************************/
			else if ((ADxCR == 0xE0034000) && (mask == 4)){

				fft_weight = (float)((value - 6)/0.71); //1.5*4	
				
				//added upon peter request 
				fft_weight = fft_weight/4.0;
				
				if (fft_weight > 0.0){
					ftoa( fft_weight, p , 1); 		 //927.7817
					NoOfBytes = strlen(p) + ind + 1;	

				}
				else{
					fft_weight = 0.0;
					p[0]='0';
					p[1]='.';
					p[2]='0';
					p[3]='\0';
					NoOfBytes = strlen(p) + ind + 1;	
				}
								
				/************************************* SEND XBEE ***********************************************************************/
				//Gather total weight for average
				total_WeightTemp = (float)(heel_weight + fft_weight);
				total_WeightTemp = total_WeightTemp ;      //this weight total sent by uart to control unit.has to be incorporated				
				
				if (total_WeightTemp > 255){   //code to do with sending Total data (Heel + Fft)via UART 
					total_WeightTemp = 255;
				}				
				WeightAvg[iter++] = (unsigned char)(total_WeightTemp);
				
				//Take average of only 16 samples....
				if (iter > 16){
					
					iter = 0;
					
					for (k=0; k<16; k++){
						weight_Total += WeightAvg[k];
					}
					weight_Total = weight_Total >> 4; //Divide by 16
				}
				/***************************************************************************************************************************/
			}
			//all other pins except A0.2 and A0.3				
			else{
				// itoa returns the number of bytes written excluding
				// trailing '\0', hence the "+ 1"
				NoOfBytes = itoa(value, p, 10) + ind + 1;	
			}
				
		
			return NoOfBytes;
	  }
	  else if(asc == 'N') // binary
	  {
			p[0] = value >> 8;
			p[1] = value;
			return ind + 2;
	  }
	  else // invalid
	  {
			return ind;
	  }
}
  /******************************************************************************/
							//get adc sample
  /******************************************************************************/

static int sample(char* q, int ind, volatile unsigned long* ADxCR, volatile unsigned long* ADxDR, int mask, char adx_bit)
{	
	
	//adx_bit tells whether adc is enabled or not.....
	if(adx_bit == 'Y')
	{

		int value = 0;
		 
		*ADxCR = 0x00020FF00 | mask; // 0000 0000 0010 0000 1111 1111 0000 0000
		*ADxCR |= 0x01000000;  // start conversion
		
		while((value & 0x80000000) == 0)
		{
		  value = *ADxDR;
		}
		*ADxCR = 0x00000000;
	
		// The upper ten of the lower sixteen bits of 'value' are the
		// result. The result itself is unsigned. Hence a cast to
		// 'unsigned short' yields the result with six bits of
		// noise. Those are removed by the following shift operation.
		return pushValue(q, ind, (unsigned short)value >> 6, ADxCR, mask );
	}
	else
	{
		return ind;
	}
}
  /******************************************************************************/
		//Timer 0 ISR  (10MS INTERRUPT or 100 FREQUENCY)
  /******************************************************************************/
static void MODE2ISR(void)
{
  int ind = 0;
  int j;
  char q[50];


  T0IR = 1; // reset TMR0 interrupt
  
  for(j = 0; j < 50; j++)
  {
		q[j] = 0;
  }

#define SAMPLE(X, BIT) ind = sample(q, ind, &AD##X##CR, &AD##X##DR, 1 << BIT, ad##X##_##BIT)
  /*******************************************************************************************/
  //every 100ms send the data on the xbee
	if (freq == 100){
	++xbee_cnt; 
	
		//CASE 1: SEND THE DATA AND GO TO SLEEP
		if (xbee_cnt > XBEE_TICKS ){
			
			xbee_cnt = 0;
			uart0_SendChar(weight_Total);        /* Send Data through Xbee */

			
			sleep_xbee(); //sleep the xbee.
		}
		//CASE 2: WAKE UP THE XBEE 
		else if (xbee_cnt == 9){
			wake_xbee(); //Wake up xbee	
		}
		
		
	}
	//      Switch    //
	if (SwCount >CALIB_TIME){
		SwCount = 0;
		timerFLAG = 0;
	}
	else
	{
		//timerFlag means timer is working
		if (timerFLAG == 1)
		{
			++SwCount;
		}
			
		// first capture captures first 5 seconds of the startup.
		if (firstCapture == 1)
		{
			if (swHighCount > 0)
			{
				/* reset counters */ 
				SwCount = 0;
				
				/* disable timer for switch */
				timerFLAG = 0;
				
				/* disable first capture */
				firstCapture = 0;
				
				/* restart counting once again */
				swHighCount = 0;
				
				/* flash calib_LED and re-enable timer after flashing LED */
				calibrationModeFLAG = 1 ; 

			}				
		}
		/* 1 press detected =  scan heel value of 10 = program the digipot only for heel = flash led
		 *scan fft value for 10 = program the fft  = flash led ...
		 *capture no of switch press wait till timer 5 second is finished and timerFlag becomes 0 
		 */
		if (secondCapture == 1)
		{	
			if (timerFLAG == 0)
			{
				//calibrationModeFLAG = 0 ;
				
				//second capture time is completed
				secondCapture = 0;
				
				// now programming the digipots.
				programDigipots_FLAG = 1;
			}
		}
	}
	

		/*  Calib sensing part */
	if  ( ( ( IOPIN0 & (1U<<Calib) ) == 0) && (SwFlag==0) && (timerFLAG == 1) )	//Check sw of calibration
	{	
		countL=0;
		++countH;
		
		/* 40ms debouncing */
		if (countH > 10)
		{
			SwFlag = 1;
			swHighCount++;
			
			//reset flags
			countL = 0;
			countH = 0;
		}
	}
	if  ( ( ( IOPIN0 & (1U<<Calib) ) != 0) && (SwFlag==1) && (timerFLAG == 1) ) //Check sw of calibration
	{
		countH = 0; 
		++countL;
		
		if (countL > 10)
		{
			SwFlag = 0;
	
			//reset flags
			countH = 0;
			countL = 0;
		}
		//clear_gpio (1<<Calib_LED); //low
	}
  /*******************************************************************************************/

  SAMPLE(1, 3); //AD1.3
  SAMPLE(0, 3); //AD0.3
  SAMPLE(0, 2); //AD0.2
  SAMPLE(0, 1); //AD0.1
  SAMPLE(1, 2); //AD1.2
  SAMPLE(0, 4); //AD0.4
  SAMPLE(1, 7); //AD1.7
  SAMPLE(1, 6); //AD1.6
#undef SAMPLE
  
  for(j = 0; j < ind; j++)
  {
		//less than buf size
		if(RX_in < BUF_SIZE)
		{
			RX_array1[RX_in] = q[j];
			RX_in++;
		
			if(RX_in == BUF_SIZE) {
				log_array1 = 1;
			}	//Raise Log_Array1 FLAG HIGH if Rx_array1 buffer is FULL.
		}
		//buffer overflow handling
		else if(RX_in >= BUF_SIZE)
		{
			RX_array2[RX_in - BUF_SIZE] = q[j];
			RX_in++;
			
			//if buffer is full raise the log_array2 flag
			if(RX_in == 2 * BUF_SIZE)
			{
				log_array2 = 1;
				RX_in = 0;   // CLEAR THE COUNTS
			}
		}
  }
  /******************************************************************************/
  if(RX_in < BUF_SIZE)
  {
		if(asc == 'N') { 
			RX_array1[RX_in] = '$'; 
		}
		
		else if(asc == 'Y'){
			RX_array1[RX_in] = 13; 
		}
		RX_in++;
	
		if(RX_in == BUF_SIZE){
			log_array1 = 1;
		}
  }
  
  else if(RX_in >= BUF_SIZE)
  {

		if(asc == 'N'){
			RX_array2[RX_in - BUF_SIZE] = '$';
		}
		
		else if(asc == 'Y'){ 
			RX_array2[RX_in - BUF_SIZE] = 13; 
		}
		RX_in++;
		
		if(RX_in == 2 * BUF_SIZE){
		  log_array2 = 1;
		  RX_in = 0;
		}
  }
  /******************************************************************************/
  if(RX_in < BUF_SIZE)
  {
    if(asc == 'N'){
		RX_array1[RX_in] = '$';
	}
    else if(asc == 'Y'){
		RX_array1[RX_in] = 10; 
	}
    RX_in++;

    if(RX_in == BUF_SIZE) log_array1 = 1;
  }
  
  else if(RX_in >= BUF_SIZE)
  {
    
    if(asc == 'N') RX_array2[RX_in - BUF_SIZE] = '$';
    else if(asc == 'Y'){ RX_array2[RX_in - BUF_SIZE] = 10; }
    RX_in++;
    
    if(RX_in == 2 * BUF_SIZE)
    {
      log_array2 = 1;
      RX_in = 0;
    }
  }

  VICVectAddr = 0;  // Acknowledge interrupt
}
/*********************************************************************************/
void FIQ_Routine(void)
{
  int j;

  stat(0,ON);
  for(j = 0; j < 5000000; j++); // TODO: Why are we using a blocking delay n ISR
  stat(0,OFF);
  U0RBR;  // Trash oldest byte in UART0 Rx FiFO Why??

  U0IIR;  // Have to read this to clear the interrupt

  // TODO: Should we be acking int here?
}

/*******************************************************
 *         Initialize
 ******************************************************/

#define PLOCK 0x400

void Initialize(void)
{
  rprintf_devopen(putc_serial0);
  
// DEFAULT LOGOMATIC
// 0xCF351505 =   0b 11 00 11 11 00 11 01 01 00 01 01 01 00 00 01 01
// 0x15441801 = //0b 00 01 01 01 01 00 01 00 00 01 10 00 00 00 00 01

// PIN_SEL1
// 0xCC351505 =   0b 11 00 11 00 00 11 01 01 00 01 01 01 00 00 01 01

// PIN_SEL2
// 0x14441801 = //0b 00 01 01 00 01 00 01 00 00 01 10 00 00 00 00 01

 /*
Digital 5 (Calib)   		(1st press within first 5sec, then within 5 seconds check the number of press and program digipots)
Digital 4 (Calib_LED)          (flash high for 1sec and then low when 1st press is detected) 
Digital 3 (CS_1)
Digital 4 (CS)

P3 = P0.28/AD0.1   (CS_1)
P4 = P0.25/AD0.4   (CALIB_LED)
P5 = P0.22/AD1.6   (CALIB)
CS1 = P0.20
*/
  
  //
  // Symbol | Value | Function
  // -------|-------|----------------------------------
  // PINSEL0|       |
  // P0.0   | 01    | TXD (UART0)
  // P0.1   | 01    | RxD (UART0)
  // P0.2   | 00    | GPIO Port 0.2
  // P0.3   | 00    | GPIO Port 0.3
  // P0.4   | 01    | SCK0 (SPI0)
  // P0.5   | 01    | MISO0 (SPI0)
  // P0.6   | 01    | MOSI0 (SPI0)
  // P0.7   | 00    | GPIO Port 0.7
  // P0.8   | 01    | TXD (UART1)
  // P0.9   | 01    | RXD (UART1)
  // P0.10  | 11    | AD1.2
  // P0.11  | 00    | GPIO Port 0.11
  // P0.12  | 11    | AD1.3     				//00 = GPIO_PORT_12 SET - SLEEP PIN=P8
  // P0.13  | 11    | AD1.4
  // P0.14  | 00    | GPIO Port 0.14
  // P0.15  | 11    | AD1.5

  // PINSEL1|       |
  // P0.16  | 01    | EINT0
  // P0.17  | 10    | GPIO Port 0.17
  // P0.18  | 10    | GPIO Port 0.18
  // P0.19  | 10    | GPIO Port 0.19
  // P0.20  | 00    | GPIO Port 0.20				//CS1     (OUPUT)  (DONE)
  // P0.21  | 10    | AD1.6
  // P0.22  | 01    | AD1.7							//CALIB    (INPUT) (DONE)
  // P0.23  | 00    | GPIO Port 0.23
  // P0.24  | 00    | Reserved
  // P0.25  | 01    | AD0.4(Default) /gpio (current				//CALIB_LED  //OUTPUT
  // P0.26  | 00    | Reserved 
  // P0.27  | 01    | Reserved
  // P0.28  | 01    | AD0.1                   		 //00 = GPIO_PORT_28 SET - BATTERY SENSING=P3      x    CS_1 (OUTPUT)
  // P0.29  | 01    | AD0.2   (Read the foot)
  // P0.30  | 01    | AD0.3   (Read the heel)
  // P0.31  | 00    | GPO Port only

  PINSEL0 = 0xCC351505;				// 1100 1111 0011 0101 0001 0101 0000 0101

  PINSEL1 = 0x144008A9;//0x14400801;	 //0001 0100 0100 0000 0000 1010 1010 1001	
  // P0.0  = INPUT  | TXD (UART0)
  // P0.1  = INPUT  | RxD (UART0)
  // P0.2  = OUTPUT | STAT0 LED
  // P0.3  = INPUT  | STOP Button
  // P0.4  = INPUT  | SCK0 (SPI0)
  // P0.5  = INPUT  | MISO0 (SPI0)
  // P0.6  = INPUT  | MOSI0 (SPI0)
  // P0.7  = OUTPUT | Chip Select 0
  // P0.8  = INPUT  | TXD (UART1)
  // P0.9  = INPUT  | RXD (UART1)
  // P0.10 = INPUT  | AD1.2
  // P0.11 = OUTPUT | STAT1 LED 
  // P0.17 = SCK1 (SSP)
  // P0.18 = MISO1 (SSP)
  // P0.19 = MOSI1 (SSP)
   //P0.20 = OUTPUT GPIO (SSP)
  
  // Rest of Port 0 are inputs
  
  //0001 0010 0001 0000 0001 1000 1000 0100
  IODIR0 |= 0x12101884; //(v2)   //12101884 (v2)      //0x00001884  (v1)                  
  //10010000100000001100010000100
  IOSET0 = 0x00000080;  // Set P0.7 HIGH | CS0 HIGH

  S0SPCR = 0x08;  // SPI clk to be pclk/8
  S0SPCR = 0x30;  // master, msb, first clk edge, active high, no ints

}

// Make values in PLL control & configure registers take effect 
void feed(void)
{
	// Interrupts must be disabled to make consecutive APB bus cycles
	PLLFEED=0xAA;
	PLLFEED=0x55;
}

static void UART0ISR(void)
{
  if(RX_in < BUF_SIZE)
  {
     RX_array1[RX_in] = U0RBR;
     RX_in++;

    if(RX_in == BUF_SIZE) log_array1 = 1;
  }
  else if(RX_in >= BUF_SIZE)
  {
     RX_array2[RX_in-BUF_SIZE] = U0RBR;
     RX_in++;

    if(RX_in == 2 * BUF_SIZE)
    {
      log_array2 = 1;
      RX_in = 0;
    }
  }


  U0IIR; // Have to read this to clear the interrupt 

  VICVectAddr = 0;  // Acknowledge interrupt
}

static void UART0ISR_2(void)
{
  char temp;
  temp = U0RBR; // Read a byte from UART0 receive buffer

  if(temp == trig)
  {
		get_frame = 1;
  }
  
  if(get_frame)
  {
		if(RX_in < frame)
		{
			  RX_array1[RX_in] = temp;
			  RX_in++;
		
			  if(RX_in == frame)
			  {
					// Delimiters
					RX_array1[RX_in] = '\n';
					RX_array1[RX_in + 1] = '\r';
					log_array1 = 1;
					get_frame = 0;
			  }
		}
		else if(RX_in >= frame)
		{
			  RX_array2[RX_in - frame] = temp;
			  RX_in++;
		
			  if(RX_in == 2*frame)
			  {
					// Delimiters
					RX_array2[RX_in - frame] = '\n';
					RX_array2[RX_in + 1 - frame] = '\r';
					log_array2 = 1;
					get_frame = 0;
					RX_in = 0;
			  }
		}
  }

  temp = U0IIR; // Have to read this to clear the interrupt

  VICVectAddr = 0;  // Acknowledge interrupt
}


void SWI_Routine(void)
{
  while(1);
}

void UNDEF_Routine(void)
{
  stat(0,ON);
}


//setup uart0
void setup_uart0(int newbaud, char want_ints)
{
  baud = newbaud;
  U0LCR = 0x83;   // 8 bits, no parity, 1 stop bit, DLAB = 1
  
  //set baud rate
  if(baud == 1200)
  {
    U0DLM = 0x0C;
    U0DLL = 0x00;
  }
  else if(baud == 2400)
  {
    U0DLM = 0x06;
    U0DLL = 0x00;
  }
  else if(baud == 4800)
  {
    U0DLM = 0x03;
    U0DLL = 0x00;
  }
  else if(baud == 9600)
  {
    U0DLM = 0x01;
    U0DLL = 0x80;
  }
  else if(baud == 19200)
  {
    U0DLM = 0x00;
    U0DLL = 0xC0;
  }
  else if(baud == 38400)
  {
    U0DLM = 0x00;
    U0DLL = 0x60;
  }
  else if(baud == 57600)
  {
    U0DLM = 0x00;
    U0DLL = 0x40;
  }
  else if(baud == 115200)
  {
    U0DLM = 0x00;
    U0DLL = 0x20;
  }

  U0FCR = 0x01;
  U0LCR = 0x03;   

  if(want_ints == 1)
  {
		enableIRQ(); 					          //enable the interrupt
		VICIntSelect &= ~0x00000040;    		  //Interrupt select register = 0000 0000 0000 0000 0000 0000 0100 0000  = Selected UART for an interrupt by assigning 0
		VICIntEnable |= 0x00000040;    		  //Interrupt Enable Register = 0000 0000 0000 0000 0000 0000 0100 0000  = This register enable interrupt request
		VICVectCntl1 = 0x26;          			  //Vector Control Register   = 0000 0000 0000 0000 0000 0000 0010 0110  = Assigned slot1 to UART (0x20|6 = 0x26) where 6 means UART0
		VICVectAddr1 = (unsigned int)UART0ISR;     //Holds the address of the ISR function, from where it will start its execution, hence pass the address of that function.
		U0IER = 0x01;					          //Enable interrupt of the UART.
  }
  else if(want_ints == 2)
  {
		enableIRQ();
		VICIntSelect &= ~0x00000040;
		VICIntEnable |= 0x00000040;
		VICVectCntl2 = 0x26;
		VICVectAddr2 = (unsigned int)UART0ISR_2;
		U0IER = 0X01;
  }
  //don't want UART as an Interrupt
  else if(want_ints == 0)
  {
		VICIntEnClr = 0x00000040;
		U0IER = 0x00;
  }
}


/****************************************************************/
//control status led on logomatic
void stat(int statnum, int onoff)
{
  if(statnum) // Stat 1
  {
    if(onoff){ IOCLR0 = 0x00000800; } // On
    else     { IOSET0 = 0x00000800; } // Off
  }
  else // Stat 0 
  {
    if(onoff){ IOCLR0 = 0x00000004; } // On
    else     { IOSET0 = 0x00000004; } // Off
  }
}
/****************************************************************/


//set and reset gpio
/*********************************************/

// Sets the output to HIGH
void set_gpio (uint32_t pin)
{
	IOSET0 = pin;
}
// Sets the output to LOW
void clear_gpio(uint32_t pin)
{
	IOCLR0 = pin;
}

/*********************************************/
void sleep_xbee (void){
	set_gpio (1U<<12);   //P8= P0.12=A1.3
}

void wake_xbee (void){
	clear_gpio(1U<<12);  //P8= P0.12=A1.3
}
/*********************************************/

//check the logcon.txt in the sd card if it is present read the string size, else create default L
void Log_init(void)
{
  int x, mark = 0, ind = 0;
  char temp, temp2 = 0, safety = 0;
//  signed char handle;

  if(root_file_exists("LOGCON.txt"))
  {
    fd = root_open("LOGCON.txt");
    stringSize = fat_read_file(fd, (unsigned char *)stringBuf, 512);
    stringBuf[stringSize] = '\0';
    fat_close_file(fd);
  }
  else
  {
    fd = root_open_new("LOGCON.txt");
    if(fd == NULL)
    {
      while(1)
      {
        stat(0,ON);
        delay_ms(50);
        stat(0,OFF);
        stat(1,ON);
        delay_ms(50);
        stat(1,OFF);
      }
    }

    strcpy(stringBuf, "MODE = 0\r\nASCII = N\r\nBaud = 4\r\nFrequency = 100\r\nTrigger Character = $\r\nText Frame = 100\r\nAD1.3 = N\r\nAD0.3 = N\r\nAD0.2 = N\r\nAD0.1 = N\r\nAD1.2 = N\r\nAD0.4 = N\r\nAD1.7 = N\r\nAD1.6 = N\r\nSafety On = Y\r\n");
    stringSize = strlen(stringBuf);
    fat_write_file(fd, (unsigned char*)stringBuf, stringSize);
    sd_raw_sync();
  }
	
  //read the configuration in logcon.txt
  for(x = 0; x < stringSize; x++)
  {
    temp = stringBuf[x];
    if(temp == 10)
    {
      mark = x;
      ind++;
      if(ind == 1)
      {
        mode = stringBuf[mark-2]-48; // 0 = auto uart, 1 = trigger uart, 2 = adc
      }
      else if(ind == 2)
      {
        asc = stringBuf[mark-2]; // default is 'N'
      }
      else if(ind == 3)
      {
        if(stringBuf[mark-2] == '1'){ baud = 1200; }
        else if(stringBuf[mark-2] == '2'){ baud = 2400; }
        else if(stringBuf[mark-2] == '3'){ baud = 4800; }
        else if(stringBuf[mark-2] == '4'){ baud = 9600; }
        else if(stringBuf[mark-2] == '5'){ baud = 19200; }
        else if(stringBuf[mark-2] == '6'){ baud = 38400; }
        else if(stringBuf[mark-2] == '7'){ baud = 57600; }
        else if(stringBuf[mark-2] == '8'){ baud = 115200; }

      }
      else if(ind == 4)
      {
        freq = (stringBuf[mark-2]-48) + (stringBuf[mark-3]-48) * 10;
        if((stringBuf[mark-4] >= 48) && (stringBuf[mark-4] < 58))
        {
          freq+= (stringBuf[mark-4]-48) * 100;
          if((stringBuf[mark-5] >= 48) && (stringBuf[mark-5] < 58)){ freq += (stringBuf[mark-5]-48)*1000; }
        }
      }
      else if(ind == 5)
      {
        trig = stringBuf[mark-2]; // default is $
      }
      else if(ind == 6)
      {
        frame = (stringBuf[mark-2]-48) + (stringBuf[mark-3]-48) * 10 + (stringBuf[mark-4]-48)*100;
        if(frame > 510){ frame = 510; } // up to 510 characters
      }
      else if(ind == 7)
      {
        ad1_3 = stringBuf[mark-2]; // default is 'N'
        if(ad1_3 == 'Y'){ temp2++; }
      }
      else if(ind == 8)
      {
        ad0_3 = stringBuf[mark-2]; // default is 'N'
        if(ad0_3 == 'Y'){ temp2++; }
      }
      else if(ind == 9)
      {
        ad0_2 = stringBuf[mark-2]; // default is 'N'
        if(ad0_2 == 'Y'){ temp2++; }
      }
      else if(ind == 10)
      {
        ad0_1 = stringBuf[mark-2]; // default is 'N'
        if(ad0_1 == 'Y'){ temp2++; }
      }
      else if(ind == 11)
      {
        ad1_2 = stringBuf[mark-2]; // default is 'N'
        if(ad1_2 == 'Y'){ temp2++; }
      }
      else if(ind == 12)
      {
        ad0_4 = stringBuf[mark-2]; // default is 'N'
        if(ad0_4 == 'Y'){ temp2++; }
      }
      else if(ind == 13)
      {
        ad1_7 = stringBuf[mark-2]; // default is 'N'
        if(ad1_7 == 'Y'){ temp2++; }
      }
      else if(ind == 14)
      {
		ad1_6 = stringBuf[mark-2]; // default is 'N'
		if(ad1_6 == 'Y'){ temp2++; }
      }
      else if(ind == 15)
      {
        safety = stringBuf[mark-2]; // default is 'Y'
      }
    }
  }

  if(safety == 'Y')
  {
    if((temp2 ==10) && (freq > 150)){ freq = 150; }
    else if((temp2 == 9) && (freq > 166)){ freq = 166; }
    else if((temp2 == 8) && (freq > 187)){ freq = 187; }
    else if((temp2 == 7) && (freq > 214)){ freq = 214; }
    else if((temp2 == 6) && (freq > 250)){ freq = 250; }
    else if((temp2 == 5) && (freq > 300)){ freq = 300; }
    else if((temp2 == 4) && (freq > 375)){ freq = 375; }
    else if((temp2 == 3) && (freq > 500)){ freq = 500; }
    else if((temp2 == 2) && (freq > 750)){ freq = 750; }
    else if((temp2 == 1) && (freq > 1500)){ freq = 1500; }
    else if((temp2 == 0)){ freq = 100; }
  }
  
  if(safety == 'T'){ test(); }

}

/*
 Logs everything that comes in on UART0, provided that it's the right UART configuration (8 data bits, one stop bit, no parity, data rate of your choosing).
*/
void mode_0(void) // Auto UART mode
{
  setup_uart0(baud,1);
  stringSize = BUF_SIZE;
  mode_action();
}

/*
 Logs a specified number of characters ("Text Frame = 100" in this case will result in 99 characters logged after the trigger) after a specified character ("Trigger = $" in this case).
*/
void mode_1(void)
{
  setup_uart0(baud,2);
  stringSize = frame + 2;

  mode_action();
}

/*
Logs ADC measurements according to which are selected as active (see below) at whatever frequency is specified ("Frequency = 100" in this case).
*/
//Every 10 ms or 100 adc measurement is taken
//send data on xbee every 100ms 
void mode_2(void)
{
	  enableIRQ();
	  
	  // Timer0  interrupt is an IRQ interrupt
	  VICIntSelect &= ~0x00000010;
	  
	  // Enable Timer0 interrupt
	  VICIntEnable |= 0x00000010;
	  
	  // Use slot 2 for Timer0 interrupt
	  VICVectCntl2 = 0x24;
	  
	  // Set the address of ISR for slot 1
	  VICVectAddr2 = (unsigned int)MODE2ISR;
	
	//When Timer Counter (TC) matches the MR0 interrupt is generated!
	
	  T0TCR = 0x00000002;	    // Reset counter and prescaler on the positive edge of PCLK 
	  
	  T0MCR = 0x00000003; 		// On match reset the counter and generate interrupt
	  T0MR0 = 58982400 / freq;  // 58982400/100 =  589824
	  
	  //Prescale Register....
	  T0PR = 0x00000000;       //prescale value is 0 
	  
	  //
	  T0TCR = 0x00000001;      // enable timer
	
	  stringSize = BUF_SIZE;
	  mode_action();
}
//anydesk 
void mode_action(void)
{
  int j;
  while(1)
  {
    
	//entered in the calibration mode if you are out of this 
	if (calibrationModeFLAG == 1)
	{
	
		if (flash_Calib_led == 1){
			
			//uart0_SendString ("\r\n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Entered in calibration mode<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
			flash_CalibLED();
			
			//enable timer and check for switches in second capture. //
			timerFLAG =1;
			secondCapture = 1;
			
			flash_Calib_led = 0;
			
			//uart0_SendString ("\r\n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>second capture time started<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
		}
		
		
		if (programDigipots_FLAG == 1)
		{
		
			char printbuf[30];
			int calibrationSuccessStat = 0;
		
			//uart0_SendString ("\r\n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>second capture time finished<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
			uart0_SendString ("\r\nsno of switch pressed ="); uart0_SendChar (swHighCount+48);

			// Read switch count here 
			// Check heel and fft here
			// Check if not equal to zero then continue
			// Increase steps till 30 output is 10.
			// Flash LED...
			/*******************   Heel Weight *************************/
			if (heel_weight > 0.00)
			{
				
				uart0_SendString ("\r\n Before Program Heel Weight Detected, Heel Value = "); ftoa (heel_weight, printbuf, 3);uart0_SendString (printbuf);
				programHeel_DIGIPOTS(1); //25 heel value at 1
				delay_ms(1);

				uart0_SendString ("\r\n After Program Heel Weight Detected, Heel Value = "); ftoa (heel_weight, printbuf, 3);uart0_SendString (printbuf);
				delay_ms(2);
												
				if (swHighCount == 1)
				{ 
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++){
						
						//Calibrated
						if (heel_weight >= 10.00){
							flash_CalibLED();
							calibrationSuccessStat = 1;
							break;
						}
						
						//else program  and check its resistance
						else{
							programHeel_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat = 0;
						}
						delay_ms (20);
					}
				}
				else if (swHighCount == 2)
				{
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++){
						
						//Calibrated
						if (heel_weight >= 20.00){
							flash_CalibLED();
							calibrationSuccessStat = 1;
							break;
						}
						
						//else program heel digipot and check its resistance
						else{
							programHeel_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat =0;
						}
						delay_ms (20);
					}
				}
				else if (swHighCount == 3)
				{
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++){
						
						//Calibrated
						if (heel_weight >= 30.00){
							flash_CalibLED();
							calibrationSuccessStat=1;
							break;
						}
						
						//else program heel digipot and check its resistance
						else{
							programHeel_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat=0;
						}
						delay_ms (20);
					}
				}
				else if (swHighCount == 4)
				{
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++){
						
						//Calibrated
						if (heel_weight >= 40.00){
							flash_CalibLED();
							calibrationSuccessStat =1;
							break;
						}
						
						//else program heel digipot and check its resistance
						else{
							programHeel_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat=0;
						}
						delay_ms (20);
					}
				}
				uart0_SendString ("\r\n HEEL DIGIPOT PROGRAMMED THE STEPS:"); 
				intToStr (STEPS_DIGIPOT, printbuf, 10); uart0_SendString (printbuf);	
				if (calibrationSuccessStat==1){
					uart0_SendString ("\r\nHEEL Calibration Successful."); 
				}
				else{
					uart0_SendString ("\r\nHEEL Calibration Unsuccessful."); 
				}
			}
			else
			{
				uart0_SendString ("\r\nKindly put weight before calibrating as current heel weight is zero...."); 
			
			}
			
			/************* FFT Weight ******************/
			if (fft_weight > 0.00){
			
				uart0_SendString ("\r\n FFT Weight Detected, FFT Value = "); ftoa (fft_weight, printbuf, 3);uart0_SendString (printbuf);
				programFFT_DIGIPOTS(1);
												
				if (swHighCount == 1)
				{
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++)
					{
						//Calibrated
						if (fft_weight >= 10.00)
						{
							flash_CalibLED();
							calibrationSuccessStat=1;
							break;
						}
						
						//else program heel digipot and check its resistance
						else
						{
							programFFT_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat=0;
						}
						delay_ms (20);
					}
				}
				else if (swHighCount == 2)
				{
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++)
					{
						
						/* Calibrated */
						if (fft_weight >= 20.00)
						{
							flash_CalibLED();
							calibrationSuccessStat=1;
							break;
						}
						
						/* program heel digipot and check its resistance */
						else
						{
							programFFT_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat=0;
						}
						delay_ms (20);
					}
				}
				
				
				else if (swHighCount == 3)
				{
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++)
					{
						
						/* Calibrated */
						if (fft_weight >= 30.00)
						{
							flash_CalibLED();
							calibrationSuccessStat=1;
							break;
						}
						
						/* program heel digipot and check its resistance */
						else{
							programFFT_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat=0;
						}
						delay_ms (20);
					}
				}
				else if (swHighCount == 4)
				{
					for (STEPS_DIGIPOT = 6; STEPS_DIGIPOT <= 150 ; STEPS_DIGIPOT++)
					{
						/* Calibrated*/
						if (fft_weight >= 40.00)
						{
							flash_CalibLED();
							calibrationSuccessStat=1;
							break;
						}
						/* program heel digipot and check its resistance */
						else{
							programFFT_DIGIPOTS(STEPS_DIGIPOT);
							calibrationSuccessStat=0;
						}
						delay_ms (20);
					}
				}
				
				uart0_SendString ("\r\n FFT DIGIPOT PROGRAMMED THE STEPS:"); 
				intToStr (STEPS_DIGIPOT, printbuf, 10); uart0_SendString (printbuf);
				
				if (calibrationSuccessStat==1)
				{
					uart0_SendString ("\r\nHEEL Calibration Successful."); 
				}
				
				else
				{
					uart0_SendString ("\r\nHEEL Calibration Unsuccessful."); 
				}
			}
			else
			{
				uart0_SendString ("\r\nKindly put weight before calibrating as current FFT weight is zero...."); 
			
			}

			// calibration completed
			calibrationModeFLAG = 0;
			programDigipots_FLAG =0;
		}
		
	}
	else
	{
			if(log_array1 == 1)
			{
			  stat(0,ON);
			  
			//print the data on the console before saving in sd-card.
			  
			  //WRITE THE RX_array1 values in the SD_CARD as it full
			  if(fat_write_file(handle,(unsigned char *)RX_array1, stringSize) < 0)
			  {	  
			  
					while(1)
					{
					  stat(0,ON);
					  for(j = 0; j < 500000; j++);
					  stat(0,OFF);
					  stat(1,ON);
					  for(j = 0; j < 500000; j++);
					  stat(1,OFF);
					}
			  }
			  sd_raw_sync();
			  stat(0,OFF);
			  log_array1 = 0;
			}
		
			if(log_array2 == 1)
			{
			  stat(1,ON);
			  //print the data on the console before saving in sd-card.
			  //WRITE THE RX_array2 values in the SD_CARD as it full
			  if(fat_write_file(handle,(unsigned char *)RX_array2, stringSize) < 0)
			  {	  
					while(1)
					{
						stat(0,ON);
						for(j = 0; j < 500000; j++);
						stat(0,OFF);
						stat(1,ON);
						for(j = 0; j < 500000; j++);
						stat(1,OFF);
					}
			  }
			  sd_raw_sync();
			  stat(1,OFF);
			  log_array2 = 0;
			}
		
			//if stop button is pressed 
			if((IOPIN0 & 0x00000008) == 0) // if button pushed, log file & quit
			{
				  VICIntEnClr = 0xFFFFFFFF;
			
				  if(RX_in < BUF_SIZE)
				  {
					fat_write_file(handle, (unsigned char *)RX_array1, RX_in);
					sd_raw_sync();
				  }
				  else if(RX_in >= BUF_SIZE)
				  {
					fat_write_file(handle, (unsigned char *)RX_array2, RX_in - BUF_SIZE);
					sd_raw_sync();
				  }
				  while(1)
				  {
						stat(0,ON);
						for(j = 0; j < 500000; j++);
						stat(0,OFF);
						stat(1,ON);
						for(j = 0; j < 500000; j++);
						stat(1,OFF);
				  }
			}
		}
	}
}

void test(void)
{
  delay_ms(5000);
  while((IOPIN0 & 0x00000008) == 0x00000008)
  {
    // Get AD1.3
    AD1CR = 0x0020FF08;
    AD_conversion(1);

    // Get AD0.3
    AD0CR = 0x0020FF08;
    AD_conversion(0);
    
    // Get AD0.2
    AD0CR = 0x0020FF04;
    AD_conversion(0);

    // Get AD0.1
    AD0CR = 0x0020FF02;
    AD_conversion(0);

    // Get AD1.2
    AD1CR = 0x0020FF04;
    AD_conversion(1);
    
    // Get AD0.4
    AD0CR = 0x0020FF10;
    AD_conversion(0);

    // Get AD1.7
    AD1CR = 0x0020FF80;
    AD_conversion(1);

    // Get AD1.6
    AD1CR = 0x0020FF40;
    AD_conversion(1);

    delay_ms(1000);
  }

  while(1);
    
}

/*
*Analog to digital conversion
*/
void AD_conversion(int regbank)
{
  int temp = 0, temp2;

  if(!regbank) // bank 0
  {
    AD0CR |= 0x01000000; // start conversion
    while((temp & 0x80000000) == 0)
    {
      temp = AD0DR;//AD0DR0;//AD0DR;
    }
    temp &= 0x0000FFC0;
    temp2 = temp / 0x00000040;

    AD0CR = 0x00000000;
  }
  else      // bank 1
  {
    AD1CR |= 0x01000000; // start conversion
    while((temp & 0x80000000) == 0)
    {
      temp = AD1DR;//AD1DR0;//AD1DR;
    }
    temp &= 0x0000FFC0;
    temp2 = temp / 0x00000040;

    AD1CR = 0x00000000;
  }
  
}

void fat_initialize(void)
{
  if(!sd_raw_init())
  {
    while(1);
  }

  if(openroot())
  { 
  }
}

// Reverses a string 'str' of length 'len' 
void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 
  
// Converts a given integer x to string str[].  
// d is the number of digits required in the output.  
// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating-point/double number to a string. 
void ftoa(float n, char* res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i] = '.'; // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter  
        // is needed to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 
/*
====================================
	SEND STRING  OVER UART
====================================
*/
void uart0_SendString (char* str){

	while (*str != '\0'){
	
		//THRE (Threshold Holding Register Empty) 
		U0THR = *str;
		while ((U0LSR & (1<<5)) == 0); //If there is data in the buffer run while loop.
		str++;
	}
}

/*
====================================
	SEND CHARACTERS  OVER UART
====================================
*/
void uart0_SendChar (char ch){
	U0THR = ch;
	while ((U0LSR & (1<<5)) == 0); //If there is data in the buffer run while loop.
}

/*
====================================
	SPI0 INITIALIZATION
====================================
*/
void SPI_Init()
{
	//PINSEL0 = PINSEL0 | 0x00001500; /* Select P0.4, P0.5, P0.6, P0.7 as SCK0, MISO0, MOSI0 and GPIO */
	S0SPCR = 0x0020; /* SPI Master mode, 8-bit data, SPI0 mode */
	S0SPCCR = 0x08; /* Even number, minimum value 8, pre scalar for SPI Clock */
}
/*
====================================
	SPI1 INITIALIZATION
====================================
*/
void SPI1_Init (void){
	
	PINSEL1 |= PINSEL1_SCK|PINSEL1_MISO|PINSEL1_MOSI;
	SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
	SSPCPSR= 16;            //Clock prescale register
	SSPCR1 = (1<<1);       //SSP Enable

}
/*
====================================
	SPI1 WRITE
====================================
*/
void SPI1_Write(uint8_t data)
{
	//char flush;
	SSPDR = data;  					 /* Load data to be written into the data register */
	while(((SSPSR & (1<<0)) == 0));
	delay_ms(1);
	//while (!(S0SPSR & 0x80));    		/* Wait till data transmission is completed */
}

/*
====================================
	SPI0 WRITE
====================================
*/
void SPI_Write(uint8_t data)
{
	//char flush;
	S0SPDR = data;  					 /* Load data to be written into the data register */
	while (!(S0SPSR & 0x80));    		/* Wait till data transmission is completed */
}
/*
char SPI_Read(void)
{
	IO0CLR = (1<<CS_0); 				 	//SSEL = 0, enable SPI communication with slave 
	S0SPDR = 0xFF;  					   // Transmit Flush byte 
	while ( (S0SPSR & 0x80) == 0 );        //Wait till data transmission is completed 
	IO0SET = (1<<CS_0);  					// SSEL = 1, disable SPI communication with slave
	return S0SPDR;  					// Return the data received 
}*/
/*
====================================
	PROGRAM DIGIPOTS FOR HEEL
====================================
*/
void programHeel_DIGIPOTS(uint8_t steps){

		//PROGRAM HEEL DIGIPOT
			clear_gpio (1<<CS_0); 					   
			SPI1_Write(0);
			SPI1_Write(STEPS_DIGIPOT);
			set_gpio (1<<CS_0);	
			delay_ms(5);
}
/*
====================================
	PROGRAM DIGIPOTS FOR FFT
====================================
*/
void programFFT_DIGIPOTS(uint8_t steps){
			//PROGRAM FFT DIGIPOT
			clear_gpio (1<<CS_1); 					   
			SPI1_Write(0);
			SPI1_Write(STEPS_DIGIPOT);
			set_gpio (1<<CS_1);
			delay_ms(5);
}

void flash_CalibLED(void){
	set_gpio (1<<Calib_LED); //high
	delay_ms(1000);
	clear_gpio (1<<Calib_LED); //low
}
