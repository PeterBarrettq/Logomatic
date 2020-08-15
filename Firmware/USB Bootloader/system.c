/******************************************************************************/
/*                                                                            */
/*                     Copyright Spark Fun Electronics                        */
/******************************************************************************/
#include "system.h"
#include "LPC214x.h"

//Debug UART0
#include "rprintf.h"
#include "serial.h"

#include "sd_raw.h"

unsigned char charY;
unsigned char charX;

/* Routine Block for startup code */
/* Define catchall routines for vector table */
void IRQ_Routine (void)   __attribute__ ((interrupt("IRQ")));
void IRQ_Routine (void)
{
}

void FIQ_Routine (void)   __attribute__ ((interrupt("FIQ")));
void FIQ_Routine (void)
{
}

void SWI_Routine (void)   __attribute__ ((interrupt("SWI")));
void SWI_Routine (void)
{
}

void UNDEF_Routine (void) __attribute__ ((interrupt("UNDEF")));
void UNDEF_Routine (void)
{
};


void boot_up(void)
{
  //Initialize the MCU clock PLL
  system_init();

  IODIR0 |= (1 << 31);
  IOCLR0 |= (1 << 31); //Turn on USB LED

  //Init UART0 for debug
  PINSEL0 |= 0x00000005; //enable uart0
  U0LCR = 0x83; // 8 bits, no Parity, 1 Stop bit, DLAB = 1 
  U0DLM = 0x00; 
  U0DLL = 0x20; // 115200 Baud Rate @ 58982400 VPB Clock  
  U0LCR = 0x03; // DLAB = 0                          

  //Init rprintf
  rprintf_devopen(putc_serial0); 
  rprintf("\n\n\nUSB Bootloader v1.1\n");

  //IOSET0 |= (1 << 31); //Turn off USB LED
}

/**********************************************************
  Initialize
 **********************************************************/

#define PLOCK 0x400  //PLL STATUS REGISTER = 0000 0100 0000 0000   , 10th bit is PLOCK bit

void system_init(void)
{
 // Fosc  = 12Mhz
 // CCLK  = M x Fosc      = 4  x 12Mhz     = 48 Mhz processor clock (CCLK)
 // FCCO  = CCLK x 2 x P  = 48 x  2  x  2  = 192 Mhz  
 
 /*   PLL CONFIGURATION REGISTER     */
 
  //Setting Multiplier and Divider values
  //Multiplier (MSEL)= M = 4  
  //Divider  (PSEL)  = P = 2
  PLLCFG=0x24;        					    //0010 0100                      [8:PLLE ][7:Reserved][6:5 (PSEL)][MSEL (4:0)]
  feed();									//apply feed so pll cfg can take affect

  // Enabling the PLL */
  PLLCON=0x1;								//0000 0001                      [7:2 RESERVED]  [1:PLLC] [0:PLLE]  PLLE : PLL ENABLE , PLLC: PLL CONNECT
  feed();									//apply feed     	 //PLLC is not used means pll clock is not used, the clock is used directly from oscillator.

  // Wait for the PLL to lock to set frequency
  while(!(PLLSTAT & PLOCK)) ;  // IF 0 MEANS NOT LOCKED, 1 MEANS LOCKED

  // Connect the PLL as the clock source
  PLLCON=0x3;		 	 					//0000 0011 //Connect PLL after PLL is locked
  feed();

	//MAM : Memory Accelerator Module....
  // Enabling MAM and setting number of clocks used for Flash memory fetch
  // (4 cclks in this case)
  //MAMTIM=0x3; //VCOM?
  MAMCR=0x2;							   //0000 0010	
  MAMTIM=0x4; //Original                   //0000 0100  //Number of clock to be used for flash memory fetches.

  // Setting peripheral Clock (pclk) to System Clock (cclk)
  VPBDIV=0x1; // PCLK is same as CCLK = 48Mhz
}

// Make values in PLL control & configure registers take effect 
void feed(void)
{
	  // Interrupts must be disabled to make consecutive APB bus cycles
	  PLLFEED=0xAA;      //1010 1010
	  PLLFEED=0x55;      //0101 0101
}

void reset(void)
{
  // Intentionally fault Watchdog to trigger a reset condition
  WDMOD |= 3;
  WDFEED = 0xAA;
  WDFEED = 0x55;
  WDFEED = 0xAA;
  WDFEED = 0x00;
}
