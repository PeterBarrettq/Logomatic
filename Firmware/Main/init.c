#include "init.h"

#define GPIO0_0_TXD0	(0x00<<0)  // 00
#define GPIO0_1_RXD0	(0x01<<2)  // 00
#define GPIO0_2_IO	(0x00<<4)  // 00
#define GPIO0_3_IO	(0x00<<6)  // 00
#define GPIO0_4_SCK0	(0x01<<8)  // 01
#define GPIO0_5_MISO0	(0x01<<10) // 01
#define GPIO0_6_MOSI0	(0x01<<12) // 01
#define GPIO0_7_IO	(0x00<<14) // 00
#define GPIO0_8_TXD1_10	(0x01<<16) // 01
#define GPIO0_9_RXD1_10	(0x01<<18) // 01
#define GPIO0_10_IO	(0x03<<20) // 11
#define GPIO0_11_IO	(0x00<<22) // 00
#define GPIO0_12_IO	(0x00<<24) // 00
#define GPIO0_13_AD1_4	(0x03<<26) // 11
#define GPIO0_14_IO	(0x00<<28) // 00
#define GPIO0_15_AD1_5	(0x03<<30) // 11
//0xCC351505
struct fat_file_struct* fd;


void fat_initialize(void)
{
	if(!sd_raw_init()){
		while(1);
	}
	if(openroot()){
	}
}

void Initialize(dev_ *device)
{
	memset(device, 0, sizeof(dev_));
	device->mode = 0;
	device->asc = 'N';
	device->baud = 9600;
	device->freq = 100;
	device->trig = '$';
	device->frame = 100;
	device->ad0_1 = 'N';
	device->ad0_2 = 'N';
	device->ad0_3 = 'N';
	device->ad0_4 = 'N';
	device->ad1_2 = 'N';
	device->ad1_3 = 'N';
	device->ad1_6 = 'N';
	device->ad1_4 = 'Y';
	device->ad1_7 = 'N';
	device->log.start_log_timer = 1;
	device->calibsw.timerFLAG = 1;
	device->calibsw.firstCapture=1;
	device->calibsw.secondCapture=0;
	device->sensor.battery_percent=0;
	device->sensor.resistor_ratio=0.5;
	device->sensor.adc_resolution=1024.0;
	device->sensor.adc_ref_volts=3.3;
	rprintf_devopen(putc_serial0);
	PINSEL0 = 0xCC351505;
	PINSEL1 = 0x144008A9;
	IODIR0 |= 0x12101884;   // 00010010 00010000 00011000 10000100 [0-input 1-output]
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


void test(void)
{
	delay_ms(5000);
	while((IOPIN0 & 0x00000008) == 0x00000008){
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
		while((temp & 0x80000000) == 0){
		temp = AD0DR;
		}
		temp &= 0x0000FFC0;
		temp2 = temp / 0x00000040;
		AD0CR = 0x00000000;
	}
	else{
		AD1CR |= 0x01000000; // start conversion
		while((temp & 0x80000000) == 0){
			temp = AD1DR;//AD1DR0;//AD1DR;
		}
		temp &= 0x0000FFC0;
		temp2 = temp / 0x00000040;
		AD1CR = 0x00000000;
	}
}
//check the logcon.txt in the sd card if it is present read the string size, else create default L
void Log_init (dev_ *device)
{
	int x, mark = 0, ind = 0;
	char temp, temp2 = 0, safety = 0;

	//  signed char handle;

	if(root_file_exists("LOGCON.txt")) {
		fd = root_open("LOGCON.txt");
		device->log.stringSize = fat_read_file(fd, (unsigned char *)device->log.stringBuf, 512);
		device->log.stringBuf[device->log.stringSize] = '\0';
		fat_close_file(fd);
	}
	else {
		fd = root_open_new("LOGCON.txt");
		if(fd == 0) {
			while(1) {
				stat(0,ON);
				delay_ms(50);
				stat(0,OFF);
				stat(1,ON);
				delay_ms(50);
				stat(1,OFF);
			}
		}

		strcpy(device->log.stringBuf, "MODE = 2\r\nASCII = Y\r\nBaud = 4\r\nFrequency = 100\r\nTrigger Character = $\r\nText Frame = 100\r\nAD1.3 = N\r\nAD0.3 = Y\r\nAD0.2 = Y\r\nAD0.1 = N\r\nAD1.2 = N\r\nAD0.4 = N\r\nAD1.7 = N\r\nAD1.6 = N\r\nSafety On = Y\r\n");
		device->log.stringSize = strlen(device->log.stringBuf);
		fat_write_file(fd, (unsigned char*)device->log.stringBuf, device->log.stringSize);
		sd_raw_sync();
	}

	//read the configuration in logcon.txt
	for(x = 0; x < device->log.stringSize; x++)
	{
		temp = device->log.stringBuf[x];
		if(temp == 10)
		{
			mark = x;
			ind++;
			if(ind == 1) {
				device->mode = device->log.stringBuf[mark-2]-48; // 0 = auto uart, 1 = trigger uart, 2 = adc
			}
			else if(ind == 2) {
				device->asc = device->log.stringBuf[mark-2]; // default is 'N'
			}
			else if(ind == 3) {
				if(device->log.stringBuf[mark-2] == '1')
					device->baud = 1200;
				else if(device->log.stringBuf[mark-2] == '2')
					device->baud = 2400;
				else if(device->log.stringBuf[mark-2] == '3')
					device->baud = 4800;
				else if(device->log.stringBuf[mark-2] == '4')
					device->baud = 9600;
				else if(device->log.stringBuf[mark-2] == '5')
					device->baud = 19200;
				else if(device->log.stringBuf[mark-2] == '6')
					device->baud = 38400;
				else if(device->log.stringBuf[mark-2] == '7')
					device->baud = 57600;
				else if(device->log.stringBuf[mark-2] == '8')
					device->baud = 115200;
			}
			else if(ind == 4) {
				device->freq = (device->log.stringBuf[mark-2]-48) +(device->log.stringBuf[mark-3]-48) * 10;
				if((device->log.stringBuf[mark-4] >= 48) && (device->log.stringBuf[mark-4] < 58)) {
					device->freq+= (device->log.stringBuf[mark-4]-48) * 100;
					if((device->log.stringBuf[mark-5] >= 48) && (device->log.stringBuf[mark-5] < 58)) {
						device->freq += (device->log.stringBuf[mark-5]-48)*1000;
					}
				}
			}
			else if(ind == 5) {
				device->trig = device->log.stringBuf[mark-2]; // default is $
			}
			else if(ind == 6) {
				device->frame = (device->log.stringBuf[mark-2]-48) + (device->log.stringBuf[mark-3]-48) * 10
						+ (device->log.stringBuf[mark-4]-48)*100;
				if(device->frame > 510)
					device->frame = 510; // up to 510 characters
			}
			else if(ind == 7) {
				device->ad1_3 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad1_3 == 'Y')
					temp2++;
			}
			else if(ind == 8) {
				device->ad0_3 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad0_3 == 'Y')
					temp2++;
			}
			else if(ind == 9) {
				device->ad0_2 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad0_2 == 'Y')
					temp2++;
			}
			else if(ind == 10) {
				device->ad0_1 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad0_1 == 'Y')
					temp2++;
			}
			else if(ind == 11) {
				device->ad1_2 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad1_2 == 'Y')
					temp2++;
			}
			else if(ind == 12) {
				device->ad0_4 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad0_4 == 'Y')
					temp2++;
			}
			else if(ind == 13) {
				device->ad1_7 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad1_7 == 'Y')
					temp2++;
			}
			else if(ind == 14) {
				device->ad1_6 = device->log.stringBuf[mark-2]; // default is 'N'
				if(device->ad1_6 == 'Y')
					temp2++;
			}
			else if(ind == 15) {
				safety = device->log.stringBuf[mark-2]; // default is 'Y'
			}
			/* Hard coded this for battery sensing */
			device->ad1_4 = 'Y';
		}
	}

	if(safety == 'Y')
	{
		if((temp2 ==10) && (device->freq > 150))
			device->freq = 150;
		else if((temp2 == 9) && (device->freq > 166))
			device->freq = 166;
		else if((temp2 == 8) && (device->freq > 187))
			device->freq = 187;
		else if((temp2 == 7) && (device->freq > 214))
			device->freq = 214;
		else if((temp2 == 6) && (device->freq > 250))
			device->freq = 250;
		else if((temp2 == 5) && (device->freq > 300))
			device->freq = 300;
		else if((temp2 == 4) && (device->freq > 375))
			device->freq = 375;
		else if((temp2 == 3) && (device->freq > 500))
			device->freq = 500;
		else if((temp2 == 2) && (device->freq > 750))
			device->freq = 750;
		else if((temp2 == 1) && (device->freq > 1500))
			device->freq = 1500;
		else if((temp2 == 0))
			device->freq = 100;
	}

	if(safety == 'T') {
		test();
	}
}

//setup uart0
void setup_uart0(dev_ *device, int newbaud, char want_ints)
{
	device->baud = newbaud;
	U0LCR = 0x83;   // 8 bits, no parity, 1 stop bit, DLAB = 1

	//set baud rate
	if(device->baud == 1200) {
		U0DLM = 0x0C;
		U0DLL = 0x00;
	}
	else if(device->baud == 2400){
		U0DLM = 0x06;
		U0DLL = 0x00;
	}
	else if(device->baud == 4800){
		U0DLM = 0x03;
		U0DLL = 0x00;
	}
	else if(device->baud == 9600){
		U0DLM = 0x01;
		U0DLL = 0x80;
	}
	else if(device->baud == 19200){
		U0DLM = 0x00;
		U0DLL = 0xC0;
	}
	else if(device->baud == 38400){
		U0DLM = 0x00;
		U0DLL = 0x60;
	}
	else if(device->baud == 57600){
		U0DLM = 0x00;
		U0DLL = 0x40;
	}
	else if(device->baud == 115200){
		U0DLM = 0x00;
		U0DLL = 0x20;
	}

	U0FCR = 0x01;
	U0LCR = 0x03;

	if(want_ints == 1) {
		enableIRQ();				//enable the interrupt
		VICIntSelect &= ~0x00000040;		//Interrupt select register = 0000 0000 0000 0000 0000 0000 0100 0000  = Selected UART for an interrupt by assigning 0
		VICIntEnable |= 0x00000040;		//Interrupt Enable Register = 0000 0000 0000 0000 0000 0000 0100 0000  = This register enable interrupt request
		VICVectCntl1 = 0x26;			//Vector Control Register   = 0000 0000 0000 0000 0000 0000 0010 0110  = Assigned slot1 to UART (0x20|6 = 0x26) where 6 means UART0
		VICVectAddr1 = (unsigned int)UART0ISR;	//Holds the address of the ISR function, from where it will start its execution, hence pass the address of that function.
		U0IER = 0x01;				//Enable interrupt of the UART.
	}
	else if(want_ints == 2) {
		enableIRQ();
		VICIntSelect &= ~0x00000040;
		VICIntEnable |= 0x00000040;
		VICVectCntl2 = 0x26;
		VICVectAddr2 = (unsigned int)UART0ISR_2;
		U0IER = 0X01;
	}
	/*Configure UART without Interrupt*/
	else if(want_ints == 0){
		VICIntEnClr = 0x00000040;
		U0IER = 0x00;
	}
}

//control status led on logomatic
void stat(int statnum, int onoff)
{
	/*Status 1 or Status LEDs are supported*/
	if(statnum){
		if(onoff){ IOCLR0 = 0x00000800; } // On
		else     { IOSET0 = 0x00000800; } // Off
	}
	else{
		if(onoff){ IOCLR0 = 0x00000004; } // On
		else     { IOSET0 = 0x00000004; } // Off
	}
}

void flash_CalibLED(uint8_t num_flash)
{
	for (int i=0;i<num_flash;i++)
	{
		set_gpio (1<<Calib_LED); //high
		delay_ms(300);
		clear_gpio (1<<Calib_LED); //low
		delay_ms(300);
	}
}

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

void sleep_xbee (void)
{
	set_gpio (1U<<12);   //P8= P0.12=A1.3
}

void wake_xbee (void)
{
	clear_gpio(1U<<12);  //P8= P0.12=A1.3
}

void UNDEF_Routine(void)
{
	stat(0,ON);
}

void SWI_Routine(void)
{
	while(1);
}

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

/*
	====================================
		SEND STRING  OVER UART
	====================================
*/
void uart0_SendString (uint8_t en, char* str)
{
	if (en)
	{
		while (*str != '\0')
		{
			//THRE (Threshold Holding Register Empty)
			U0THR = *str;
			while ((U0LSR & (1<<5)) == 0); //If there is data in the buffer run while loop.
			str++;
		}
	}
}

/*
	====================================
		SEND CHARACTERS  OVER UART
	====================================
*/
void uart0_SendChar (uint8_t en, char ch)
{
	if (en){
		U0THR = ch;
		while ((U0LSR & (1<<5)) == 0); //If there is data in the buffer run while loop.
	}
}
/*
	====================================
		SPI1 INITIALIZATION
	====================================
*/
void SPI1_Init (void)
{

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
