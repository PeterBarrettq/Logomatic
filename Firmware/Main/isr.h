/*
 * irq.h
 *
 *  Created on: 28-Feb-2022
 *      Author: SAR Computer
 */

#ifndef MAIN_ISR_H_
#define MAIN_ISR_H_

#include "init.h"

#define CALIB_TIME		500    //10MS * 500 = 5000
#define Calib			22 //Calib Button (D5)
#define XBEE_TICKS		10
#define NUM_AVERAGE		16
#define AD0CR_REG_ADDR		0xE0034000

#endif /* MAIN_ISR_H_ */
