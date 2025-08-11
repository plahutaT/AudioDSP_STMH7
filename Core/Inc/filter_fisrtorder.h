/*
 * filter_fisrtorder.h
 *
 *  Created on: Oct 29, 2024
 *      Author: tilen
 */

#ifndef INC_FILTER_FISRTORDER_H_
#define INC_FILTER_FISRTORDER_H_


#define FIRSTORDER_FILTER_LENGTH 2


typedef struct
{
	float filter_coef[FIRSTORDER_FILTER_LENGTH];
	float fs_hz;

	float filter_output;

} LowPass_FirstOrder_t;


void LowPass_FirstOrder_Init(LowPass_FirstOrder_t* filt, float fc_hz, float fs_hz);
void LowPass_FirstOrder_SetFc(LowPass_FirstOrder_t* filt, float fc_hz);
float LowPass_FirstOrder_Update(LowPass_FirstOrder_t* filt, float input);



#endif /* INC_FILTER_FISRTORDER_H_ */
