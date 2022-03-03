/*
 * irq.c
 *
 *  Created on: 28-Feb-2022
 *      Author: SAR Computer
 */
#include "isr.h"
static uint8_t WeightAvg[20];

static inline int pushValue(dev_ *device, char* q, int ind, int value, volatile unsigned long* ADxCR, int mask)
{
	  char* p = q + ind;
	  if(device->asc == 'Y')
	  {
		int NoOfBytes=0;

		/* Gather value of A0.1 (HEEL WEIGHT) */
		if ((ADxCR == (unsigned long*)AD0CR_REG_ADDR) && (mask == 8)) {
			s2.adc=value;
			device->sensor.heel_weight = (s2.gain) * (s2.adc - (s2.offset_nw))/1000.0;
			if (device->sensor.heel_weight > 0.0) {
				ftoa (device->sensor.heel_weight, p , 1);
				NoOfBytes = strlen(p) + ind + 1;
			} else {
				device->sensor.heel_weight = 0.0;
				p[0]='0';
				p[1]='.';
				p[2]='0';
				p[3]='\0';
				NoOfBytes = strlen(p) + ind + 1;
			}
		}
		/* Gather value of A0.2 (FFT WEIGHT) */
		else if ((ADxCR == (unsigned long*)AD0CR_REG_ADDR) && (mask == 4)) {
			s1.adc=value;
			device->sensor.fft_weight = (s1.gain) * (s1.adc - (s1.offset_nw))/1000.0;

			if (device->sensor.fft_weight > 0.0) {
				ftoa (device->sensor.fft_weight, p , 1);
				NoOfBytes = strlen(p) + ind + 1;
			}
			else {
				device->sensor.fft_weight = 0.0;
				p[0]='0';
				p[1]='.';
				p[2]='0';
				p[3]='\0';
				NoOfBytes = strlen(p) + ind + 1;
			}

			/*
			 *	Send Data on ZigBee
			 */

			//Gather total weight for average
			device->sensor.total_WeightTemp = (float)(device->sensor.heel_weight + device->sensor.fft_weight);

			//code to do with sending Total data (Heel + Fft)via UART
			if (device->sensor.total_WeightTemp > 255)
				device->sensor.total_WeightTemp = 255;

			WeightAvg[device->sensor.iter++] = (unsigned char)(device->sensor.total_WeightTemp);

			//Take average of only 16 samples....
			if (device->sensor.iter > NUM_AVERAGE) {
				device->sensor.iter = 0;
				for (device->sensor.k=0; device->sensor.k<NUM_AVERAGE; device->sensor.k++)
					device->sensor.weight_Total += WeightAvg[device->sensor.k];

				device->sensor.weight_Total = device->sensor.weight_Total/NUM_AVERAGE; //Divide by 16
			}
		}
		else if ((ADxCR == (unsigned long*)AD1CR_REG_ADDR) && (mask == 16)) {
			device->sensor.battery_volts = (((float)value/device->sensor.adc_resolution)*
					device->sensor.adc_ref_volts)/device->sensor.resistor_ratio;
			device->sensor.battery_percent = (int)((device->sensor.battery_volts/4.2)*100.0);
		}
		//all other pins except A0.2 and A0.3
		else {
			// itoa returns the number of bytes written excluding
			// trailing '\0', hence the "+ 1"
			NoOfBytes = itoa(value, p, 10) + ind + 1;
		}
		return NoOfBytes;
	  }
	  else if(device->asc == 'N') {
			p[0] = value >> 8;
			p[1] = value;
			return ind + 2;
	  }
	  else {
			return ind;
	  }
}
/*
 * This function gets the ADC samples
 * sample()
 * @q
 * @ind
 * @ADxCR: Register Address
 * @ADxDR: Register Address
 * @mask: mask
 * @adx_bit: adc bit
 */
static int sample(dev_ *device, char* q, int ind, volatile unsigned long* ADxCR, volatile unsigned long* ADxDR, int mask, char adx_bit)
{
	if(adx_bit == 'Y')
	{
		int value = 0;

		*ADxCR = 0x00020FF00 | mask;
		*ADxCR |= 0x01000000;  // Set 24th bit of ADxCR register to start conversion

		while((value & 0x80000000) == 0) {
			value = *ADxDR;
		}
		*ADxCR = 0x00000000;

		// The upper ten of the lower sixteen bits of 'value' are the
		// result. The result itself is unsigned. Hence a cast to
		// 'unsigned short' yields the result with six bits of
		// noise. Those are removed by the following shift operation.
		return pushValue(device, q, ind, (unsigned short)value >> 6, ADxCR, mask);
	}
	else
	{
		return ind;
	}
}

void UART0ISR(void)
{
	if(dev.uart.RX_in < BUF_SIZE)
	{
		dev.uart.RX_array1[dev.uart.RX_in] = U0RBR;
		dev.uart.RX_in++;
		if(dev.uart.RX_in == BUF_SIZE)
			dev.uart.log_array1 = 1;
	}
	else if(dev.uart.RX_in >= BUF_SIZE)
	{
		dev.uart.RX_array2[dev.uart.RX_in-BUF_SIZE] = U0RBR;
		dev.uart.RX_in++;
		if(dev.uart.RX_in == 2 * BUF_SIZE){
			dev.uart.log_array2 = 1;
			dev.uart.RX_in = 0;
		}
	}
	U0IIR; // Have to read this to clear the interrupt
	VICVectAddr = 0;  // Acknowledge interrupt
}

void UART0ISR_2(void) {
  char temp;

	temp = U0RBR;
	/* Read a byte from UART0 receive buffer */
	if(temp == dev.trig){
		dev.uart.get_frame = 1;
	}

	if(dev.uart.get_frame) {
		if(dev.uart.RX_in < dev.frame) {
			dev.uart.RX_array1[dev.uart.RX_in] = temp;
			dev.uart.RX_in++;

			if(dev.uart.RX_in == dev.frame) {
				// Delimiters
				dev.uart.RX_array1[dev.uart.RX_in] = '\n';
				dev.uart.RX_array1[dev.uart.RX_in + 1] = '\r';
				dev.uart.log_array1 = 1;
				dev.uart.get_frame = 0;
			}
		}
		else if(dev.uart.RX_in >= dev.frame) {
			dev.uart.RX_array2[dev.uart.RX_in - dev.frame] = temp;
			dev.uart.RX_in++;

			if(dev.uart.RX_in == 2*dev.frame)
			{
				// Delimiters
				dev.uart.RX_array2[dev.uart.RX_in - dev.frame] = '\n';
				dev.uart.RX_array2[dev.uart.RX_in + 1 - dev.frame] = '\r';
				dev.uart.log_array2 = 1;
				dev.uart.get_frame = 0;
				dev.uart.RX_in = 0;
			}
		}
	}

	temp = U0IIR; // Have to read this to clear the interrupt

	VICVectAddr = 0;  // Acknowledge interrupt
}


/*
 * This function handles Xbee and calibration switch
 * MODE2ISR
 */
void MODE2ISR(void)
{
	int ind = 0;
	int j;
	char q[50];
	T0IR = 1; // reset TMR0 interrupt

	for(j = 0; j < 50; j++)
		q[j] = 0;

	#define SAMPLE(X, BIT) ind = sample(&dev, q, ind, &AD##X##CR, &AD##X##DR, 1 << BIT, dev.ad##X##_##BIT)

	/*Every 100ms send the data on the XBee*/
	if (dev.freq == 100) {
		++dev.xbee.xbee_cnt;

		/*CASE 1:Send the data and put XBee in sleep mode*/
		if (dev.xbee.xbee_cnt > XBEE_TICKS ) {
			dev.xbee.xbee_cnt = 0;
			/* Send Data through XBee */
			if (dev.calibrationModeFLAG == 0) {
				rprintf ("%d,%d\r\n",dev.sensor.weight_Total,dev.sensor.battery_percent);
			}

			/* Put XBee in sleep mode */
			sleep_xbee();
		}

		/* CASE 2:Wake up XBee for sending the data */
		else if (dev.xbee.xbee_cnt == 9) {
			wake_xbee();
		}
	}

	//      Switch    //
	if (dev.calibsw.SwCount >CALIB_TIME) {
		dev.calibsw.SwCount = 0;
		dev.calibsw.timerFLAG = 0;
	}
	else {
		//timerFlag means timer is working
		if (dev.calibsw.timerFLAG == 1)
			++dev.calibsw.SwCount;

		// first capture starts on first 5 seconds of the startup.
		if (dev.calibsw.firstCapture == 1) {
			if (dev.calibsw.swHighCount > 0) {
				/* reset counters */
				dev.calibsw.SwCount = 0;

				/* disable timer for switch */
				dev.calibsw.timerFLAG = 0;

				/* disable first capture */
				dev.calibsw.firstCapture = 0;

				/* restart counting once again */
				dev.calibsw.swHighCount = 0;

				/* flash calib_LED and re-enable timer after flashing LED */
				dev.calibrationModeFLAG = 1;
			}
		}
		/* 1 press detected =  scan heel value of 10 = program the digipot only for heel = flash led
		*scan fft value for 10 = program the FFT  = flash led ...
		*capture no of switch press wait till timer 5 second is finished and timerFlag becomes 0
		*/
		if (dev.calibsw.secondCapture == 1) {
			if (dev.calibsw.timerFLAG == 0) {
				//second capture time is completed
				dev.calibsw.secondCapture = 0;

				// now programming the Digipots.
				dev.calibsw.calibrateSensor_FLAG = 1;
			}
		}
	}
	/*
	* Calib Switch Sensing Part
	*/
	/* HIGH Logic */
	if  ( ( ( IOPIN0 & (1U<<Calib) ) == 0)
			&& (dev.calibsw.SwFlag==0) && (dev.calibsw.timerFLAG == 1) ) {

		dev.calibsw.countL = 0;
		++dev.calibsw.countH;
		/* 40ms Debouncing */
		if (dev.calibsw.countH > 10) {
			dev.calibsw.SwFlag = 1;
			dev.calibsw.swHighCount++;

			//reset flags
			dev.calibsw.countL = 0;
			dev.calibsw.countH = 0;
		}
	}
	/* LOW Logic */
	if  ( ( ( IOPIN0 & (1U<<Calib) ) != 0) && (dev.calibsw.SwFlag==1) && (dev.calibsw.timerFLAG == 1) )
	{
		dev.calibsw.countH = 0;
		++dev.calibsw.countL;
		if (dev.calibsw.countL > 10) {
			dev.calibsw.SwFlag = 0;

			//reset flags
			dev.calibsw.countH = 0;
			dev.calibsw.countL = 0;
		}
	}

	/*
	* This condition creates log file 10 seconds after startup
	*/
	if (dev.log.start_log_timer == 1) {
		if (dev.log.log_enable_cnt > 1000) {
			dev.log.create_log_file=1;
			dev.log.log_enable_cnt = 0;
			dev.log.start_log_timer=0;
		}
		else {
			dev.log.log_enable_cnt++;
		}
	}
//	SAMPLE(1, 3); //AD1.3
	SAMPLE(1, 4); //AD1.4
	SAMPLE(0, 3); //AD0.3
	SAMPLE(0, 2); //AD0.2
//	SAMPLE(0, 1); //AD0.1
//	SAMPLE(1, 2); //AD1.2
//	SAMPLE(0, 4); //AD0.4
//	SAMPLE(1, 7); //AD1.7
//	SAMPLE(1, 6); //AD1.6
	#undef SAMPLE

	for(j = 0; j < ind; j++)
	{
		//less than buf size
		if(dev.uart.RX_in < BUF_SIZE)
		{
			dev.uart.RX_array1[dev.uart.RX_in] = q[j];
			dev.uart.RX_in++;

			if(dev.uart.RX_in == BUF_SIZE)
			{
				dev.uart.log_array1 = 1;
			}	//Raise Log_Array1 FLAG HIGH if Rx_array1 buffer is FULL.
		}
		//buffer overflow handling
		else if(dev.uart.RX_in >= BUF_SIZE) {
			dev.uart.RX_array2[dev.uart.RX_in - BUF_SIZE] = q[j];
			dev.uart.RX_in++;

			//if buffer is full raise the log_array2 flag
			if(dev.uart.RX_in == 2 * BUF_SIZE) {
					dev.uart.log_array2 = 1;
					dev.uart.RX_in = 0;   // CLEAR THE COUNTS
			}
		}
	}
	if(dev.uart.RX_in < BUF_SIZE)
	{
		if(dev.asc == 'N')
			dev.uart.RX_array1[dev.uart.RX_in] = '$';
		else if(dev.asc == 'Y')
			dev.uart.RX_array1[dev.uart.RX_in] = 13;

		dev.uart.RX_in++;
		if(dev.uart.RX_in == BUF_SIZE)
			dev.uart.log_array1 = 1;
	}
	else if(dev.uart.RX_in >= BUF_SIZE)
	{
		if(dev.asc == 'N')
			dev.uart.RX_array2[dev.uart.RX_in - BUF_SIZE] = '$';
		else if(dev.asc == 'Y')
			dev.uart.RX_array2[dev.uart.RX_in - BUF_SIZE] = 13;
		dev.uart.RX_in++;

		if(dev.uart.RX_in == 2 * BUF_SIZE) {
			dev.uart.log_array2 = 1;
			dev.uart.RX_in = 0;
		}
	}
	if(dev.uart.RX_in < BUF_SIZE) {
		if(dev.asc == 'N')
			dev.uart.RX_array1[dev.uart.RX_in] = '$';
		else if(dev.asc == 'Y')
			dev.uart.RX_array1[dev.uart.RX_in] = 10;
		dev.uart.RX_in++;
		if(dev.uart.RX_in == BUF_SIZE)
			dev.uart.log_array1 = 1;
	}

	else if(dev.uart.RX_in >= BUF_SIZE) {
		if(dev.asc == 'N')
			dev.uart.RX_array2[dev.uart.RX_in - BUF_SIZE] = '$';
		else if(dev.asc == 'Y')
			dev.uart.RX_array2[dev.uart.RX_in - BUF_SIZE] = 10;
		dev.uart.RX_in++;
		if(dev.uart.RX_in == 2 * BUF_SIZE) {
			dev.uart.log_array2 = 1;
			dev.uart.RX_in = 0;
		}
	}
	VICVectAddr = 0;
}
