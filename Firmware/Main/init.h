/*
 * init.h
 *
 *  Created on: 28-Feb-2022
 *      Author: SAR Computer
 */

#ifndef MAIN_INIT_H_
#define MAIN_INIT_H_

#include "LPC21xx.h" //LPC214x.h
#include <stdint.h>
#include "rprintf.h"
#include "serial.h" //UART0 Debugging
#include "isr.h"
#include "armVIC.h"
#include "delay.h"
#include "rootdir.h"
#include "sd_raw.h"
#include "fat.h"
#include <string.h>
#include <stdlib.h>
#include "itoa.h"

#define Calib_LED		25 	//Calib_LED    (D4)
#define ON			1
#define OFF			0
#define BUF_SIZE		512
#define SSP_DSS			0x07 << 0  /* data size            : 8 bits    */
#define SSP_FRF			0x00 << 4  /* frame format         : SPI       */
#define SSP_CPOL		0x00 << 6  /* clock polarity       : idle high */  //(Needs high for SCP1000)
#define SSP_CPHA		0x00 << 7  /* clock phase          : 1         */
#define SSP_SCR			0x0F << 8  /* serial clock rate    : 58.59kHz = PCLK / (CPSDVSR * [SCR+1]) = 15000000 / (16 * [15+1]) */
#define PINSEL1_SCK		(2 << 2)
#define PINSEL1_MISO		(2 << 4)
#define PINSEL1_MOSI 		(2 << 6)

void uart0_SendString (uint8_t en, char* str);
void uart0_SendChar (uint8_t en, char ch);
void SPI1_Init (void);
void SPI1_Write(uint8_t data);
void flash_CalibLED(uint8_t num_flash);
void clear_gpio(uint32_t pin);
void set_gpio (uint32_t pin);
void sleep_xbee (void);
void wake_xbee (void);
void stat(int statnum, int onoff);
void FIQ_Routine(void) __attribute__ ((interrupt("FIQ")));
void SWI_Routine(void) __attribute__ ((interrupt("SWI")));
void UNDEF_Routine(void) __attribute__ ((interrupt("UNDEF")));
void feed(void);
void fat_initialize(void);
void reverse(char* str, int len) ;
int intToStr(int x, char str[], int d) ;
void ftoa(float n, char* res, int afterpoint) ;

enum mode {
	FFT_TYPE=0,
	HEEL_TYPE
};

typedef struct _log {
	signed int stringSize;
	char stringBuf[256];
	uint8_t create_log_file;
	uint8_t start_log_timer;
	uint8_t savelogs;
	uint16_t log_enable_cnt;
}log_;

typedef struct _sensor {
	float heel_weight, fft_weight;
	int weight_Total;
	uint8_t k, iter1,iter2;
	uint16_t total_WeightTemp;
	float battery_volts;
	int battery_percent;
	int battery_per_temp;
	float resistor_ratio;
	float adc_resolution;
	float adc_ref_volts;
}sensor_;

typedef struct _calibsw {
	uint8_t SwFlag;  //flag for maintain switch high and lows
	uint8_t timerFLAG; //you can start and stop timer using timerFLAG , 1 means start 0 means stop
	uint16_t  SwCount; //it is timer counts.
	uint16_t countH,countL; //debounce handling counts
	uint16_t  swHighCount; //captures how many times switch have been pressed.
	uint8_t firstCapture, secondCapture;
	uint8_t calibrateSensor_FLAG; //when it is high program digipots.
} calibsw_;

typedef struct _xbee {
	uint16_t xbee_cnt;
}xbee_;

typedef struct calibration {
	float gain;
	uint32_t offset_w,offset_nw;
	uint16_t adc;
}calib;

typedef struct uart_{
	char RX_array1[BUF_SIZE];
	char RX_array2[BUF_SIZE];
	char log_array1;
	char log_array2;
	short RX_in;
	char get_frame;
}_uart;

typedef struct _dev_ {
	char   mode;
	char    asc;
	int    baud;
	int    freq;
	char   trig;
	short frame;
	char  ad1_7, ad1_6, ad1_3, ad1_2, ad0_4, ad0_3, ad0_2, ad0_1, ad1_4;
	uint8_t calibrationModeFLAG;//????? Moiz verify this flag
	log_ log;
	sensor_ sensor;
	xbee_ xbee;
	calibsw_ calibsw;
	_uart uart;
}dev_;
dev_ dev;

calib s1,s2;
void UART0ISR(void);
void UART0ISR_2(void);
void MODE2ISR(void);
void setup_uart0(dev_ *device, int newbaud, char want_ints);
void Log_init (dev_ *device);
void Initialize(dev_ *device);
void AD_conversion(int regbank);
void test(void);
#endif /* MAIN_INIT_H_ */
