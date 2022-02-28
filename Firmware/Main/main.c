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
#include "init.h"

//Needed for main function calls
#include "main_msc.h"
#include "itoa.h"
#include "string_printf.h"
#include "delay.h"
#include <errno.h>

#define PLOCK			0x400

// PINS DESCRIPTION
#define CS_1			28 //FFT digipot  (D3)
#define CS_0			20 //CS1 on breakout board CS0  ?????  CS0 = P0.7, CS1 = P0.20
#define STAT1			11
#define STAT0			2
#define BATLVL			13//P0.13 A1.4
#define ENABLE_SD_LOGS		1

void mode_0(dev_ *device);
void mode_1(dev_ *device);
void mode_2(dev_ *device);
void mode_action(dev_ *device);
void write_sd_card(float gain_fft, float gain_heel);
void calib_init(void);
void calibrate_load_cell(calib *sensor, uint8_t type, float weight, char* msg);
void print_init_values (dev_ *device);

uint8_t flash_Calib_led=1; //flashes the led on calibration
uint8_t DEBUG_LOGOMATIC=1;

struct fat_file_struct* handle;
struct fat_file_struct* cfg;
struct fat_file_struct* cfg1;

//Fosc = 12 Mhz 
//CCLK = PCLK =  12x4=48Mhz , hence 48 Mhz is a peripheral clock
int main (void)
{
	int i;
	char filename[32];
	memset (&s1,0,sizeof(s1));
	memset (&s1,0,sizeof(s2));
	enableFIQ();
	Initialize (&dev);
	setup_uart0 (&dev, 9600, 0);
	SPI1_Init();
	fat_initialize();
	print_init_values(&dev);

	/* Flash Status Lights */
	for(i = 0; i < 5; i++){
		stat(0,ON);
		delay_ms(50);
		stat(0,OFF);
		stat(1,ON);
		delay_ms(50);
		stat(1,OFF);
	}
	Log_init(&dev);
	calib_init();
	string_printf(filename,"CALIB02.txt");

	if(root_file_exists(filename)) {
		uart0_SendString (DEBUG_LOGOMATIC,"\r\nFile Exist.");
		cfg = root_open(filename);
		sd_raw_sync();
	} else {
		uart0_SendString (DEBUG_LOGOMATIC,"\r\nFile don't exist.");
	}

	print_init_values(&dev);

	if(dev.mode==0)
		mode_0(&dev);
	else if(dev.mode==1)
		mode_1(&dev);
	else if(dev.mode==2)
		mode_2(&dev);
	return 0;
}

void print_init_values (dev_ *device)
{
	rprintf ("mode = %d\r\n",device->mode);
	rprintf ("asc = %c\r\n",device->asc);
	rprintf ("baud = %d\r\n",device->baud);
	rprintf ("freq = %d\r\n",device->freq);
	rprintf ("trig = %c\r\n",device->trig);
	rprintf ("frame = %d\r\n",device->frame);
	rprintf ("ad1_7 = %c\r\n",device->ad1_7);
	rprintf ("ad1_6 = %c\r\n",device->ad1_6);
	rprintf ("ad1_3 = %c\r\n",device->ad1_3);
	rprintf ("ad1_2 = %c\r\n",device->ad1_2);
	rprintf ("ad0_4 = %c\r\n",device->ad0_4);
	rprintf ("ad0_3 = %c\r\n",device->ad0_3);
	rprintf ("ad0_2 = %c\r\n",device->ad0_2);
	rprintf ("ad0_1 = %c\r\n",device->ad0_1);

	rprintf ("------------ Log--------------------------\r\n");
	rprintf ("calibrationModeFLAG = %d\r\n",device->calibrationModeFLAG);
	rprintf ("stringSize = %d\r\n",device->log.stringSize);
	rprintf ("create_log_file = %d\r\n",device->log.create_log_file);
	rprintf ("start_log_timer = %d\r\n",device->log.start_log_timer);
	rprintf ("savelogs = %d\r\n",device->log.savelogs);
	rprintf ("log_enable_cnt = %d\r\n",device->log.log_enable_cnt);

	rprintf ("------------ Sensor--------------------------\r\n");
	rprintf ("heel_weight = %d\r\n",device->sensor.heel_weight);
	rprintf ("fft_weight = %d\r\n",device->sensor.fft_weight);
	rprintf ("weight_Total = %d\r\n",device->sensor.weight_Total);
	rprintf ("iter = %d\r\n",device->sensor.iter);
	rprintf ("k = %d\r\n",device->sensor.k);
	rprintf ("total_WeightTemp",device->sensor.total_WeightTemp);

	rprintf ("------------ CalibSW--------------------------\r\n");
	rprintf ("SwFlag = %d\r\n",device->calibsw.SwFlag);
	rprintf ("timerFLAG = %d\r\n",device->calibsw.timerFLAG);
	rprintf ("SwCount = %d\r\n",device->calibsw.SwCount);
	rprintf ("countH = %d\r\n",device->calibsw.countH);
	rprintf ("countL = %d\r\n",device->calibsw.countL);
	rprintf ("swHighCount = %d\r\n",device->calibsw.swHighCount);
	rprintf ("firstCapture = %d\r\n",device->calibsw.firstCapture);
	rprintf ("secondCapture = %d\r\n",device->calibsw.secondCapture);
	rprintf ("calibrateSensor_FLAG = %d\r\n",device->calibsw.calibrateSensor_FLAG);

	rprintf ("------------ UART--------------------------\r\n");
	rprintf ("log_array1 = %d\r\n",device->uart.log_array1);
	rprintf ("log_array2 = %d\r\n",device->uart.log_array2);
	rprintf ("RX_in = %d\r\n",device->uart.RX_in);
	rprintf ("get_frame = %d\r\n",device->uart.get_frame);
}

/*
 Logs everything that comes in on UART0, provided that it's the right UART configuration (8 data bits, one stop bit, no parity, data rate of your choosing).
 Auto UART mode
*/
void mode_0 (dev_ *device)
{
	setup_uart0(device, device->baud , 1);
	device->log.stringSize = BUF_SIZE;
	rprintf ("stringSize = %d\r\n",device->log.stringSize);
	mode_action(device);
}

/*
 Logs a specified number of characters ("Text Frame = 100" in this case will result in 99 characters logged after the trigger) after a specified character ("Trigger = $" in this case).
*/
void mode_1 (dev_ *device)
{
	setup_uart0(device, device->baud,2);
	device->log.stringSize = device->frame + 2;
	mode_action(device);
}

/*
Logs ADC measurements according to which are selected as active (see below) at whatever frequency is specified ("Frequency = 100" in this case).
Every 10 ms or 100 adc measurement is taken
send data on xbee every 100ms 
*/
void mode_2 (dev_ *device)
{
	enableIRQ();
	VICIntSelect &= ~0x00000010;		//Timer0  interrupt is an IRQ interrupt
	VICIntEnable |= 0x00000010;			//Enable Timer0 interrupt
	VICVectCntl2 = 0x24;				//Use slot 2 for Timer0 interrupt
	VICVectAddr2 = (unsigned int)MODE2ISR;	//Set the address of ISR for slot 1
	//When Timer Counter (TC) matches the MR0 interrupt is generated!
	T0TCR = 0x00000002;					//Reset counter and prescaler on the positive edge of PCLK 
	T0MCR = 0x00000003;					//On match reset the counter and generate interrupt
	T0MR0 = 58982400 / device->freq;			// 58982400/100 =  589824
	T0PR = 0x00000000;					//prescale value is 0 
	T0TCR = 0x00000001;					//enable timer
	device->log.stringSize = BUF_SIZE;
	
	/*Perform Action based on the mode*/
	mode_action(device);
}

/*
* This function normal routine of LOGOMATIC
*/
void mode_action(dev_ *device)
{
	while(1)
	{
		/*
		* Calibration Mode Detected
		*/
		if (device->calibrationModeFLAG == 1)
		{
			if (flash_Calib_led == 1)
			{
				uart0_SendString (DEBUG_LOGOMATIC,"\r\n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Entered in calibration mode<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
				flash_CalibLED(1);

				/*Enabling timer and check for switches in second capture.*/
				device->calibsw.timerFLAG =1;
				device->calibsw.secondCapture = 1;
				flash_Calib_led = 0;
				uart0_SendString (DEBUG_LOGOMATIC,"\r\n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>second capture time started<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
			}
			if (device->calibsw.calibrateSensor_FLAG == 1)
			{
				uart0_SendString (DEBUG_LOGOMATIC,"\r\n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>second capture time finished<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
				uart0_SendString (DEBUG_LOGOMATIC,"\r\nNumber of Switch Pressed="); uart0_SendChar (DEBUG_LOGOMATIC,device->calibsw.swHighCount+48);
				flash_CalibLED(2);
				/* Read switch count here
				* Check heel and fft here
				* Check if not equal to zero then continue
				* Increase steps till 30 output is 10.
				* Flash LED...
				* Heel Weight
				*/
				switch (device->calibsw.swHighCount)
				{
					case 1:
						uart0_SendString (DEBUG_LOGOMATIC,"\r\n SW: Pressed 1 times.");
						calibrate_load_cell(&s1, FFT_TYPE,10.0,"FFT sensor");
						flash_CalibLED(3);
						uart0_SendString (DEBUG_LOGOMATIC,"\r\n Wait for 6 seconds");
						delay_ms(6000); //6s delay
						flash_CalibLED(4);
						calibrate_load_cell(&s2,HEEL_TYPE,10.0,"HEEL sensor");
						write_sd_card(s1.gain, s2.gain);
						flash_CalibLED(5);
						break;
					case 2:
						uart0_SendString (DEBUG_LOGOMATIC,"\r\n SW: Pressed 2 times.");
						calibrate_load_cell(&s1, FFT_TYPE,20.0,"FFT sensor");
						flash_CalibLED(3);
						uart0_SendString (DEBUG_LOGOMATIC,"\r\n Wait for 6 seconds");
						delay_ms(6000); //6s delay
						flash_CalibLED(4);
						calibrate_load_cell(&s2, HEEL_TYPE,20.0,"HEEL sensor");
						write_sd_card(s1.gain, s2.gain);
						flash_CalibLED(5);
						break;
					case 3:
						uart0_SendString (DEBUG_LOGOMATIC,"\r\n SW: Pressed 3 times.");
						calibrate_load_cell(&s1,FFT_TYPE,30.0,"FFT sensor");
						uart0_SendString (DEBUG_LOGOMATIC,"\r\n Wait for 6 seconds");
						flash_CalibLED(3);
						delay_ms(6000); //6s delay
						flash_CalibLED(4);
						calibrate_load_cell(&s2,HEEL_TYPE,30.0,"HEEL sensor");
						write_sd_card(s1.gain, s2.gain);
						flash_CalibLED(5);
						break;
					default:
						break;
				}
				// calibration completed
				device->calibrationModeFLAG = 0;
				device->calibsw.calibrateSensor_FLAG =0;
			}
		}
		/*Normal Condition*/
		else
		{
#ifdef ENABLE_SD_LOGS
			long j=0;

			/* 10s UP after startup then create new log file */
			if (device->log.create_log_file == 1)
			{
//				rprintf ("if (device->log.create_log_file == 1)\r\n");
				if ((device->calibsw.secondCapture==0) && (device->calibsw.calibrateSensor_FLAG==0) &&
						(device->calibrationModeFLAG==0) && (device->calibsw.timerFLAG==0))
				{
//					rprintf ("if (device->log.create_log_file == 1)->check2\r\n");
					char name[32];
					int count = 0;
					count++;
					string_printf(name,"LOG%02d.txt",count);
					fat_close_file(cfg);
					while(root_file_exists(name))
					{
						count++;
						if(count == 250)
						{
							while(1)
							{
								stat(0,ON);
								stat(1,ON);
								delay_ms(1000);
								stat(0,OFF);
								stat(1,OFF);
								delay_ms(1000);
							}
						}
						string_printf(name,"LOG%02d.txt",count);
					}
					handle = root_open_new(name);
					sd_raw_sync();
					device->log.create_log_file=0;
					device->log.savelogs=1;
				}
			}

			/* Start saving the LOGS */
			if (device->log.savelogs == 1)
			{
//				rprintf ("if (device->log.savelogs)\r\n");
				if(device->uart.log_array1 == 1)
				{
//					rprintf ("if(device->uart.log_array1 == 1)\r\n");
					stat(0,ON);
					/* print the data on the console before saving in sd-card */
					/*WRITE THE RX_array1 values in the SD_CARD as it full */
					if(fat_write_file(handle,(unsigned char *)device->uart.RX_array1, device->log.stringSize) < 0)
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
					device->uart.log_array1 = 0;
				}

				if(device->uart.log_array2 == 1)
				{
//					rprintf ("if(device->uart.log_array2 == 1)\r\n");
					stat(1,ON);
					/* print the data on the console before saving in sd-card*/
					/* WRITE THE RX_array2 values in the SD_CARD as it full */
					if(fat_write_file(handle,(unsigned char *)device->uart.RX_array2, device->log.stringSize) < 0)
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
					device->uart.log_array2 = 0;
				}
				/* STOP Button Condition */
				if((IOPIN0 & 0x00000008) == 0)
				{
					uart0_SendString (DEBUG_LOGOMATIC,"\r\nSTOP BT Pressed!");
					VICIntEnClr = 0xFFFFFFFF;

					if(device->uart.RX_in < BUF_SIZE)
					{
						fat_write_file(handle, (unsigned char *)device->uart.RX_array1, device->uart.RX_in);
						sd_raw_sync();
					}
					else if(device->uart.RX_in >= BUF_SIZE)
					{
						fat_write_file(handle, (unsigned char *)device->uart.RX_array2, device->uart.RX_in - BUF_SIZE);
						sd_raw_sync();
					}
					while(1) {
						stat(0,ON);
						for(j = 0; j < 500000; j++);
							stat(0,OFF);
							stat(1,ON);
						for(j = 0; j < 500000; j++);
							stat(1,OFF);
					}
				}
			}
#endif
		}
	}
}

/*
* Calibrate Load Cell Heel or FFT
* @ adc: ADC value of heel or fft
*/
void calibrate_load_cell(calib *sensor, uint8_t type, float weight, char* msg)
{
	uint32_t cnt=0, avg_adc=0;
	char printbuf[30];

	sensor->offset_nw = 0;
	uart0_SendString (DEBUG_LOGOMATIC,"\r\nPut the ");ftoa(weight, printbuf,1);uart0_SendString (DEBUG_LOGOMATIC,printbuf);
	uart0_SendString(DEBUG_LOGOMATIC,"KG weight on "); uart0_SendString(DEBUG_LOGOMATIC,msg);
	uart0_SendString(DEBUG_LOGOMATIC," and hold it until led flashes.");
	delay_ms(4000);
	while (1) {
		if (cnt>20) {
			avg_adc = (sensor->offset_w)/20;
			//uart0_SendString (DEBUG_LOGOMATIC,"\r\n\r\nAverage ADC Value= ");intToStr(avg_adc, printbuf, 3);		uart0_SendString (DEBUG_LOGOMATIC,printbuf);

			sensor->gain = (weight*1000.0)/(avg_adc-(sensor->offset_nw));
			uart0_SendString (DEBUG_LOGOMATIC,"\r\n\r\nGain= ");	ftoa(sensor->gain, printbuf, 1);	uart0_SendString (DEBUG_LOGOMATIC,printbuf);
			cnt=0;
			break;
		} else {
			uart0_SendString (DEBUG_LOGOMATIC,"\r\nADC= ");	intToStr(sensor->adc, printbuf, 3);	uart0_SendString (DEBUG_LOGOMATIC,printbuf);
			sensor->offset_w += sensor->adc;
			cnt++;
			delay_ms(100);
		}
	}
	uart0_SendString (DEBUG_LOGOMATIC,"\r\n Release weight from the sensor");
}

/* Function writes data in sd-card
 * @gain_fft : gain fft value
 * @gain_heel: gain heel value
 */
void write_sd_card(float gain_fft, float gain_heel)
{
	char buf_gain_fft[25], buf_gain_heel[25];
	char buffer_file[256];

	memset(buf_gain_heel,'0', sizeof(buf_gain_heel));
	memset(buf_gain_fft, '0', sizeof(buf_gain_fft));

	/* Compose FFT to save in SD-CARD */
	ftoa (gain_fft, buf_gain_fft, 1);

	/* Compose Heel to save in SD-CARD */
	ftoa (gain_heel, buf_gain_heel, 1);

	/*
	 * FORMAT= GAIN FFT=00000\r\nGAIN HEEL=00000\r\n
	 */
	strcpy(buffer_file, "GAIN FFT=");
	strcat(buffer_file, buf_gain_fft);
	strcat(buffer_file, "\r\n");
	strcat(buffer_file, "GAIN HEEL=");
	strcat(buffer_file, buf_gain_heel);
	strcat(buffer_file, "\r\n");
	uart0_SendString(DEBUG_LOGOMATIC,"\r\n->Writing Data\r\n");uart0_SendString(DEBUG_LOGOMATIC,buffer_file);

	/*
	 * Write Data in SD-CARD
	 */
	if (fat_write_file(cfg,(unsigned char *)buffer_file, strlen(buffer_file)) > 0){
		uart0_SendString (DEBUG_LOGOMATIC,"\r\nSuccessfully written");
	}
	else{
		uart0_SendString (DEBUG_LOGOMATIC,"\r\nUnable to write");
	}
	sd_raw_sync();
    fat_close_file(cfg);
}

/*Read the calibration parameters*/
void calib_init(void){
	int x, mark = 0, ind = 0, i=0;
	char gain_buf_fft[25],gain_buf_heel[25],temp_buf[30];
	char* errCheck;
	float d=0.00;
	char filename[32];
	char buffer_file[256];
	signed int buffersize;

	memset (gain_buf_heel,'0',sizeof(gain_buf_heel));
	memset (gain_buf_fft, '0',sizeof(gain_buf_fft));

	string_printf(filename,"CALIB02.txt");

	if(root_file_exists(filename))
	{
	    cfg = root_open(filename);
	    buffersize = fat_read_file(cfg, (unsigned char *)buffer_file, 512);
	    buffer_file[buffersize]='\0';
	    fat_close_file(cfg);
	}
	else
	{
		cfg = root_open_new(filename);
		if (cfg ==0)
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
		strcpy(buffer_file, "GAIN FFT=00000\r\nGAIN HEEL=00000\r\n");
		buffersize = strlen(buffer_file);
		uart0_SendString (DEBUG_LOGOMATIC,"\r\nWriting data in Calib.txt: ");uart0_SendString(DEBUG_LOGOMATIC,buffer_file);
		fat_write_file(cfg, (unsigned char*)buffer_file, buffersize);
		sd_raw_sync();
	}
	/*
	 * Parsing the data
	 */
	for (x=0; x<buffersize; x++)
	{
		if(buffer_file[x]=='=')
		{
			  mark = x;
			  ind++;
			  if (ind == 1)
			  {
				  for (i=0; buffer_file[(mark+1+i)]!='\r';i++)
				  {
					if(i > 9)
						break;
					else
						gain_buf_fft[i]=buffer_file[mark+1+i];
				  }
				  gain_buf_fft[i]=0;
				  d = strtod(gain_buf_fft, &errCheck);
				  s1.gain=d; //Save the gain of FFT
				  ftoa (d, temp_buf, 1);
				  uart0_SendString (DEBUG_LOGOMATIC,"\r\n->Read FFT Gain from SD-CARD=> ");	
				  uart0_SendString(DEBUG_LOGOMATIC,temp_buf);
			  }
			  else if (ind == 2)
			  {
				  for (i=0; buffer_file[(mark+1+i)]!='\r';i++)
				  {
					  if(i > 9)
						  break;
					  else
						  gain_buf_heel[i]= buffer_file[mark+1+i];
				  }
				  gain_buf_heel[i]=0;
			          d = strtod(gain_buf_heel, &errCheck);
				  s2.gain=d;			 //Save the gain of Heel
				  ftoa (d, temp_buf, 1);
				  uart0_SendString (DEBUG_LOGOMATIC,"\r\n->Read HEEL Gain from SD-CARD= ");	
				  uart0_SendString(DEBUG_LOGOMATIC,temp_buf);
			  }
		}
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

