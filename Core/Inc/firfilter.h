/*
 * firfilter.h
 *
 *  Created on: Oct 29, 2024
 *      Author: tilen
 */

#ifndef INC_FIRFILTER_H_
#define INC_FIRFILTER_H_

#include <stdint.h>
#include <stddef.h>


#define FIR_FILTER_LENGTH 49


typedef struct
{
	float filter_buff[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	float filter_output;

} FirFilter_t;


void FirFilter_Init(FirFilter_t* fir);
float FirFilter_Update(FirFilter_t* fir, float input);


#endif /* INC_FIRFILTER_H_ */
