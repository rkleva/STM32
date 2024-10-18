/*
 * ADBMS6948.h
 *
 *  Created on: Oct 7, 2024
 *      Author: user
 */

#ifndef INC_ADBMS6948_H_
#define INC_ADBMS6948_H_

/***** Configuration Register Commands*****/
//----------------------------------------------------------------------------------
#define WRCFGA 0x0001				/* Write Configuration Group A */
#define WRCFGB 0x0024				/* Write Configuration Group B */
#define RDCFGA 0x0002				/* Read Configuration Group A */
#define RDCFGB 0x0026				/* Read Configuration Group B */
#define RDCFGC 0x0082				/* Read Configuration Group C */
#define WRCFGC 0x0081				/* Write Configuration Group C */
#define RDCFGD 0x00A6				/* Read Configuration Group D */
#define WRCFGD 0x00A4				/* Write Configuration Group D */
#define WRCFGE 0x0073				/* Write Configuration Group E */
#define RDCFGE 0x0074				/* Read Configuration Group E */
#define WRCFGF 0x0075				/* Write Configuration Group F */
#define RDCFGF 0x0076				/* Read Configuration Group F */
#define WRCFGG 0x0077				/* Write Configuration Group G */
#define RDCFGG 0x0078				/* Read Configuration Group G */
#define WRCFGH 0x0079				/* Write Configuration Group H */
#define RDCFGH 0x007A				/* Read Configuration Group H */
#define WRCFGI 0x007B				/* Write Configuration Group I */
#define RDCFGI 0x007C				/* Read Configuration Group I */
//----------------------------------------------------------------------------------

/***** Always on Commands *****/
//----------------------------------------------------------------------------------
#define ULAO 0x0038					/* Unlock Always On Memory */
#define WRAO 0x0039					/* Write Always On Memory */
#define RDAO 0x003A					/* Read Always On Memory */
#define RDAOOC3A 0x003C				/* Read Always On OC3 Group A */
#define WRAOOC3A 0x003E				/* Write Always On OC3 Group A */
//----------------------------------------------------------------------------------


/***** Read Averaged Cell Voltage Result Register Commands *****/
//----------------------------------------------------------------------------------
#define RDACA 0x0044				/* Read Averaged Cell Voltage Register Group A */
#define RDACB 0x0046				/* Read Averaged Cell Voltage Register Group B */
#define RDACC 0x0048				/* Read Averaged Cell Voltage Register Group C */
#define RDACD 0x004A				/* Read Averaged Cell Voltage Register Group D */
#define RDACE 0x0049				/* Read Averaged Cell Voltage Register Group E */
#define RDACF 0x004B				/* Read Averaged Cell Voltage Register Group F */
//----------------------------------------------------------------------------------

/***** PWM Register Commands *****/
//----------------------------------------------------------------------------------
#define WRPWMA 0x0020				/* Write PWM Register Group A */
#define RDPWMA 0x0022				/* Read PWM Register Group A */
#define WRPWMB 0x0021				/* Write PWM Register Group B */
#define RDPWMB 0x0023				/* Read PWM Register Group B */
//----------------------------------------------------------------------------------


/***** ADC Commands *****/
//----------------------------------------------------------------------------------
#define ADCV_RD	0x0360				/* Start Cell Voltage ADC Conversion and Poll Status with RD = 1 */
#define ADCV_CONT 0x02E0			/* Start Cell Voltage ADC Conversion and Poll Status with CONT = 1 */
#define ADCV_DCP 0x0270				/* Start Cell Voltage ADC Conversion and Poll Status with DCP = 1 */
#define ADCV_RSTF 0x0264            /* Start Cell Voltage ADC Conversion and Poll Status with RSTF = 1 */






#endif /* INC_ADBMS6948_H_ */
