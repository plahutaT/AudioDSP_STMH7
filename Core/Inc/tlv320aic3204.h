/*
 * tlv320aic3204.h
 *
 *  Created on: Oct 9, 2024
 *      Author: tilen
 */

#ifndef INC_TLV320AIC3204_H_
#define INC_TLV320AIC3204_H_

#include "stdint.h"
#include "spi.h"
#include "gpio.h"

#define PAGE_SELECT_REGISTER 0x00



/*   Audio Codec API
 *
 *
 * */

typedef int32_t (*tlv320aic3204_Write_Func)(void *, uint8_t, uint8_t);
typedef int32_t (*tlv320aic3204_Read_Func) (void *, uint8_t, uint8_t*);

typedef struct
{

	uint32_t SamplingFrequency;   						/*   sampling frequency         */
	uint8_t InputDevice;								/*   input selection            */
	uint8_t OutputDevice;								/*   output selection           */
	uint8_t Volume;										/*   gain selection             */
	uint8_t CommInterface;								/*   communication selection    */
	uint8_t LDOSetup;									/*   LDO selection              */

}tlv320aic3204_Init_t;

typedef struct
{
	tlv320aic3204_Write_Func   WriteReg;
	tlv320aic3204_Read_Func    ReadReg;
	void*                      handle;
} tlv320aic3204_IO_t;

typedef struct
{
	tlv320aic3204_Init_t Init;
	tlv320aic3204_IO_t   IO;
	uint8_t             IsInitialized;

}tlv320aic3204_Object_t;


typedef struct
{

	int32_t ( *Init                 ) ( tlv320aic3204_Object_t *, tlv320aic3204_Init_t* );
	int32_t ( *DeInit               ) ( tlv320aic3204_Object_t * );
	int32_t ( *Play                 ) ( tlv320aic3204_Object_t * );
	int32_t ( *Pause                ) ( tlv320aic3204_Object_t * );
	int32_t ( *Resume               ) ( tlv320aic3204_Object_t * );
	int32_t ( *Stop                 ) ( tlv320aic3204_Object_t *, uint32_t );
	int32_t ( *SetFrequency         ) ( tlv320aic3204_Object_t *, uint32_t );
	int32_t ( *GetFrequency         ) ( tlv320aic3204_Object_t *, uint32_t*);
	int32_t ( *SetVolume            ) ( tlv320aic3204_Object_t *, uint32_t, int8_t );
	int32_t ( *GetVolume            ) ( tlv320aic3204_Object_t *, uint32_t, uint8_t*);
	int32_t ( *SetMute              ) ( tlv320aic3204_Object_t *, uint32_t );
	int32_t ( *SetOutputMode        ) ( tlv320aic3204_Object_t *, uint32_t );
	int32_t ( *SetResolution        ) ( tlv320aic3204_Object_t *, uint32_t );
	int32_t ( *GetResolution        ) ( tlv320aic3204_Object_t *, uint32_t*);
	int32_t ( *SetProtocol          ) ( tlv320aic3204_Object_t *, uint32_t );
	int32_t ( *GetProtocol          ) ( tlv320aic3204_Object_t *, uint32_t*);
	int32_t ( *Reset                ) ( tlv320aic3204_Object_t * );

} tlv320aic3204_Driver_t;


extern tlv320aic3204_Driver_t TLV320AIC3204_Driver;
extern tlv320aic3204_Object_t TLV320AIC3204_Obj;


#define TLV320AIC3204_OK                (0)
#define TLV320AIC3204_ERROR             (-1)

/******************************************************************************/
/***************************  Codec User defines ******************************/
/******************************************************************************/
/* Audio Input Device */
#define TLV320AIC3204_IN_NONE           0x00U
#define TLV320AIC3204_IN_MIC1           0x01U
#define TLV320AIC3204_IN_MIC2           0x02U
#define TLV320AIC3204_IN_LINE1          0x03U
#define TLV320AIC3204_IN_LINE2          0x04U
#define TLV320AIC3204_IN_MIC1_MIC2      0x05U

/* Audio Output Device */
#define TLV320AIC3204_OUT_NONE          0x00U
#define TLV320AIC3204_OUT_SPEAKER       0x01U
#define TLV320AIC3204_OUT_HEADPHONE     0x02U
#define TLV320AIC3204_OUT_BOTH          0x03U
#define TLV320AIC3204_OUT_AUTO          0x04U

/* AUDIO FREQUENCY */
#define TLV320AIC3204_FREQUENCY_96K     96000
#define TLV320AIC3204_FREQUENCY_48K     48000
#define TLV320AIC3204_FREQUENCY_44K     44100


/* AUDIO RESOLUTION */
#define TLV320AIC3204_RESOLUTION_16b    0x00U
#define TLV320AIC3204_RESOLUTION_20b    0x01U

/* Codec stop options */
#define TLV320AIC3204_PDWN_HW           0x00U
#define TLV320AIC3204_PDWN_SW           0x01U

/* Volume Input Output selection */
#define VOLUME_INPUT                  	   0U
#define VOLUME_OUTPUT                 	   1U

/* MUTE commands */
#define TLV320AIC3204_MUTE_ON              1U
#define TLV320AIC3204_MUTE_OFF             0U

/* AUDIO PROTOCOL */
#define TLV320AIC3204_PROTOCOL_R_JUSTIFIED    ((uint16_t)0x0000)
#define TLV320AIC3204_PROTOCOL_L_JUSTIFIED    ((uint16_t)0x0001)
#define TLV320AIC3204_PROTOCOL_I2S            ((uint16_t)0x0002)
#define TLV320AIC3204_PROTOCOL_DSP            ((uint16_t)0x0003)


void tlv320aic3204_BeepTest(void);
uint8_t tlv320aic3204_Init(void);
void tlv320aic3204_hardwareReset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void tlv320aic3204_writeRegister(uint8_t addr, uint8_t value);
uint8_t tlv320aic3204_readRegister(uint8_t addr);

/*    Codec IO functions     */

int32_t tlv320aic3204_Write_Reg(void *handle, uint8_t addr, uint8_t val);
int32_t tlv320aic3204_Read_Reg (void *handle, uint8_t addr, uint8_t* data);

/*    Register manipulation functions     */

int32_t tlv320aic3204_sw_reset_w(tlv320aic3204_IO_t   *IO, uint8_t val);
int32_t tlv320aic3204_sw_reset_r(tlv320aic3204_IO_t   *IO, uint8_t* val);


/*------------------------------------------------------------------------------
                           Audio Codec driver functions
------------------------------------------------------------------------------*/
/* High Layer codec functions */

int32_t TLV320AIC3204_Play(tlv320aic3204_Object_t *pObj);
int32_t TLV320AIC3204_Pause(tlv320aic3204_Object_t *pObj);
int32_t TLV320AIC3204_Resume(tlv320aic3204_Object_t *pObj);
int32_t TLV320AIC3204_SetVolume(tlv320aic3204_Object_t *pObj, uint32_t InputOutput, int8_t Volume);
int32_t TLV320AIC3204_SetMute(tlv320aic3204_Object_t *pObj, uint32_t Cmd);


/*------------------------------------------------------------------------------
                           Helper functions
------------------------------------------------------------------------------*/

#define MIN_DB_DAC -63.5
#define MAX_DB_DAC 24.0

#define MIN_DB_ADC -12   // Minimum dB for the register
#define MAX_DB_ADC 20    // Maximum dB for the register

static uint8_t mapVolumeToDACReg(int8_t dB);
static uint8_t mapVolumeToADCRegister(int8_t dB);




#endif /* INC_TLV320AIC3204_H_ */
