/*
 * tlv320aic3204.c
 *
 *  Created on: Oct 9, 2024
 *      Author: tilen
 */

#include "tlv320aic3204.h"



/** @defgroup WM8994_Private_Types Private Types
  * @{
  */
/* Audio codec driver structure initialization */
tlv320aic3204_Driver_t TLV320AIC3204_Driver =
{
	NULL,
	NULL,
	TLV320AIC3204_Play,
	TLV320AIC3204_Pause,
	TLV320AIC3204_Resume,
	NULL,
	NULL,
	NULL,
	TLV320AIC3204_SetVolume,
	NULL,
	TLV320AIC3204_SetMute,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

const tlv320aic3204_IO_t TLV320AIC3204_IO =
{
	tlv320aic3204_Write_Reg,
	tlv320aic3204_Read_Reg,
	&hspi2
};

const tlv320aic3204_Init_t TLV320AIC3204_Init = {
    .SamplingFrequency = TLV320AIC3204_FREQUENCY_48K,      		/* Set to 48 kHz */
    .InputDevice = TLV320AIC3204_IN_LINE1,                		/* Set input to line 1 */
    .OutputDevice = TLV320AIC3204_OUT_SPEAKER,               	/* Set output to speakers */
    .Volume = 0x80,                  							/* Set volume to 50% */
    .CommInterface = TLV320AIC3204_PROTOCOL_I2S,              	/* Use I2S communication protocol */
    .LDOSetup = 1                    							/* Enable LDO power setup */
};

tlv320aic3204_Object_t TLV320AIC3204_Obj = {
    .Init = TLV320AIC3204_Init,  								/* Set the initialization parameters */
    .IO = TLV320AIC3204_IO,      								/* Set the IO functions */
    .IsInitialized = 0           								/* Set initialized flag to false (0) initially */
};





uint8_t tlv320aic3204_Init(void)
{
	/*    HARDWARE RESET   */
	tlv320aic3204_hardwareReset(nCodecRST_GPIO_Port, nCodecRST_Pin);


	/*    SOFTWARE RESET   */

	//select Page 0
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0);
	// make software reset
	tlv320aic3204_writeRegister(0x01, 0x01);
	HAL_Delay(1);

	/*    POWER SETUP   */

	// Select Page 1
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0x01);
	// Disable Internal Crude AVdd in presence of external AVdd supply or before
	//powering up internal AVdd LDO
	tlv320aic3204_writeRegister(0x01, 0x08);
	// Enable Master Analog Power Control
	tlv320aic3204_writeRegister(0x02, 0x01);

	/*    PLL AND CLK CONFIG   */
	// Select Page 0
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0);
	// Set the word length of Audio Interface to 16bits PTM_P4
	tlv320aic3204_writeRegister(0x1B, 0x00);
	// Set the data offset to 0
	tlv320aic3204_writeRegister(0x1C, 0x00);
	// Set clock setting MCLK -> PLL_CLKIN, PLL_CLK -> CODEC_CLKIN
	// MCLK = 12 Mhz
	/*
	codec_writeRegister(0x04, 0x03);
	// Set PLL J Values, J = 7
	codec_writeRegister(0x06, 0x07);
	// Set PLL D Values, D = 1680 , Reg 0x07 = MSB, Reg 0x08 = LSB
	codec_writeRegister(0x07, 0x06);
	codec_writeRegister(0x08, 0x90);
	// Set PLL P and R Values, R = 1, to get desired fs change the value of P
	codec_writeRegister(0x05, 0x91);
	*/


	/*  ADC and DAC clock deviders */

	/*
	// Program the OSR of DAC to 128
	codec_writeRegister(0x0D, 0x00);
	codec_writeRegister(0x0E, 0x80);
	// Program OSR for ADC to 128
	codec_writeRegister(0x14, 0x80);

	// Power up the NDAC divider with value 7
	codec_writeRegister(0x0B, 0x87);
	// Power up the MDAC divider with value 2
	codec_writeRegister(0x0C, 0x82);
	// Power up NADC divider with value 7
	codec_writeRegister(0x12, 0x87);
	// Power up MADC divider with value 2
	codec_writeRegister(0x13, 0x82);
	*/

	//PLL
	// MCLK pin is CODEC_CLKIN
	tlv320aic3204_writeRegister(0x04, 0x00);
	// 0: PLL is powered down, P = 8, R = 1
	tlv320aic3204_writeRegister(0x05, 0x00);
	// NDAC MDAC
	// NDAC = 1
	tlv320aic3204_writeRegister(0x0B, 0x81);
	// MDAC = 2
	tlv320aic3204_writeRegister(0x0C, 0x82);
	// NADC = 1
	tlv320aic3204_writeRegister(0x12, 0x81);
	// MDAC = 2
	tlv320aic3204_writeRegister(0x13, 0x82);

	// Program the OSR of DAC to 128
	tlv320aic3204_writeRegister(0x0D, 0x00);
	tlv320aic3204_writeRegister(0x0E, 0x80);
	// Program OSR for ADC to 128
	tlv320aic3204_writeRegister(0x14, 0x80);



	/*    DAC SETUP   */

	//TODO i don't know how to chose the right processing block
	// Set the DAC Mode to PRB_P8
	tlv320aic3204_writeRegister(0x3C, 0x08);
	//testiramo ce deluje beep
	//codec_writeRegister(0x3C, 0x19);

	// Select Page 1
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0x01);
	// Set the REF charging time to 40ms
	tlv320aic3204_writeRegister(0x7B, 0x01);
	// HP soft stepping settings for optimal pop performance at power up
	// Rpop used is 6k with N = 6 and soft step = 20usec. This should work with 47uF coupling
	// capacitor. Can try N=5,6 or 7 time constants as well. Trade-off delay vs “pop” sound.
	tlv320aic3204_writeRegister(0x14, 0x25);

	// Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to Input Common Mode
	// HPL and HPR are powered with AVDD
	// Why is the output common mode set to 0.9V and not to 1.65V
	tlv320aic3204_writeRegister(0x0A, 0x00);

	// Route Left DAC to LOL
	tlv320aic3204_writeRegister(0x0E, 0x08);
	// Route Right DAC to LOR
	tlv320aic3204_writeRegister(0x0F, 0x08);

	// Set the DAC PTM mode to PTM_P3/4
	// PowerTune Mode 3/4, high quality power saving is not a must
	// Both DAC use Class AB driver
	tlv320aic3204_writeRegister(0x03, 0x00);
	tlv320aic3204_writeRegister(0x04, 0x00);

	// Set the HPL gain to 0dB
	tlv320aic3204_writeRegister(0x10, 0x00);
	// Set the HPR gain to 0dB
	tlv320aic3204_writeRegister(0x11, 0x00);

	// Set the LOL gain to 0dB
	tlv320aic3204_writeRegister(0x12, 0x00);
	// Set the LOR gain to 0dB
	tlv320aic3204_writeRegister(0x13, 0x00);

	// Power up LOL and LOR drivers
	tlv320aic3204_writeRegister(0x09, 0x0C);
	// Wait for 2.5 sec for soft stepping to take effect
	// Else read Page 1, Register 63d, D(7:6). When = “11” soft-stepping is complete

	// Select Page 0
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0);
	// Power up the Left and Right DAC Channels with route the Left Audio digital data to
	// Left Channel DAC and Right Audio digital data to Right Channel DAC, soft-step volume change enable
	tlv320aic3204_writeRegister(0x3F, 0xD5);
	// Unmute the DAC digital volume control
	tlv320aic3204_writeRegister(0x40, 0x00);


	/*    ADC SETUP   */

	// Initialize to Page 0
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0);
	// Select ADC PRB_R1
	tlv320aic3204_writeRegister(0x3D, 0x01);

	// Select Page 1
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 1);
	// Select ADC PTM_R4
	tlv320aic3204_writeRegister(0x3D, 0x00);
	// Set MicPGA startup delay to 3.1ms
	tlv320aic3204_writeRegister(0x47, 0x32);


	// Route IN1L to LEFT_P with 20K input impedance
	tlv320aic3204_writeRegister(0x34, 0x80);
	// Route Common Mode to LEFT_M with impedance of 20K
	tlv320aic3204_writeRegister(0x36, 0x80);
	// Route IN1R to RIGHT_P with input impedance of 20K
	tlv320aic3204_writeRegister(0x37, 0x80);
	// Route Common Mode to RIGHT_M with impedance of 20K
	tlv320aic3204_writeRegister(0x39, 0x80);

	// Unmute Left MICPGA, Gain selection of 6dB to make channel gain 0dB
	// Register of 6dB with input impedance of 20K => Channel Gain of 0dB
	tlv320aic3204_writeRegister(0x3B, 0x0C);
	// Unmute Right MICPGA, Gain selection of 6dB to make channel gain 0dB
	// Register of 6dB with input impedance of 20K => Channel Gain of 0dB
	tlv320aic3204_writeRegister(0x3C, 0x0C);

	// Select Page 0
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0);
	// Power up Left and Right ADC Channels
	tlv320aic3204_writeRegister(0x51, 0xC0);
	// Unmute Left and Right ADC Digital Volume Control.
	tlv320aic3204_writeRegister(0x52, 0x00);

	HAL_Delay(100);

	return 0;
}



void tlv320aic3204_hardwareReset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    /* Check if it is not already in RST     */
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	/* Drive pin LOW for hardware RST        */
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	/* Drive pin HIGH                        */
	HAL_Delay(10);											/* Wait for analogue LDO and PLL startup   */

}



void tlv320aic3204_writeRegister(uint8_t addr, uint8_t value)
{
	uint8_t data[2] = {0};
	data[0] = addr<<1;      // for read operation we RESET the R/W bit
	data[1] = value;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, data, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}



uint8_t tlv320aic3204_readRegister(uint8_t addr)
{
	uint8_t txData[2] = {0};
	uint8_t rxData[2] = {0};

	txData[0] = (addr<<1) | 0x01; // for read operation we SET the R/W bit
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	return rxData[1];
}



void tlv320aic3204_BeepTest(void)
{
	uint32_t sampleLength = 0x3A980; // sample duration 5 sec
	uint16_t freq = 1000; // freq 1 kHz
	// Select Page 0
	tlv320aic3204_writeRegister(PAGE_SELECT_REGISTER, 0x00);
	// write sample duration
	tlv320aic3204_writeRegister(0x49, (uint8_t)((sampleLength>>16) & 0xFF));
	tlv320aic3204_writeRegister(0x4A, (uint8_t)((sampleLength>>8) & 0xFF));
	tlv320aic3204_writeRegister(0x4B, (uint8_t)(sampleLength & 0xFF));
	// write sample frequency
	tlv320aic3204_writeRegister(0x4C, (uint8_t)((freq>>8) & 0xFF));
	tlv320aic3204_writeRegister(0x4D, (uint8_t)(freq & 0xFF));

	tlv320aic3204_writeRegister(0x4E, (uint8_t)((freq>>8) & 0xFF));
	tlv320aic3204_writeRegister(0x4F, (uint8_t)(freq & 0xFF));

	// enable beep generator and set volume -6 dB
	tlv320aic3204_writeRegister(0x47, 0x86);
}

/*
 *
 *       tlv320aic3204 IO functions
 */

int32_t tlv320aic3204_Write_Reg(void *handle, uint8_t addr, uint8_t val)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) handle;
	int32_t ret = 0;
	uint8_t data[2] = {0};
	data[0] = addr<<1;      // for read operation we RESET the R/W bit
	data[1] = val;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(hspi, data, 2, 1000) != HAL_OK)
	{
		ret += TLV320AIC3204_ERROR;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	return ret;
}



int32_t tlv320aic3204_Read_Reg (void *handle, uint8_t addr, uint8_t* data)
{
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *) handle;
	int32_t ret = 0;
	uint8_t txData[2] = {0};
	uint8_t rxData[2] = {0};

	txData[0] = (addr << 1) | 0x01; // for read operation we SET the R/W bit
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive(hspi, txData, rxData, 2, 1000) != HAL_OK)
	{
		ret += TLV320AIC3204_ERROR;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	*data = rxData[1];

	return ret;
}







/*
 *
 *       tlv320aic3204 Codec driver functions
 */




/**
  * @brief Enables or disables the mute feature on the audio codec.
  * @param  pObj pointer to component object
  * @param Cmd  tlv320aic3204_MUTE_ON to enable the mute or tlv320aic3204_MUTE_OFF to disable the
  *             mute mode.
  * @retval 0 if correct communication, else wrong communication
  */
int32_t TLV320AIC3204_SetMute(tlv320aic3204_Object_t *pObj, uint32_t Cmd)
{
    int32_t ret = TLV320AIC3204_OK;
    uint8_t tmp = 0;

    /* Set the Mute mode */
    if (Cmd == TLV320AIC3204_MUTE_ON)
    {
        /* Left and Right DAC channels are muted */
        ret = pObj->IO.ReadReg(pObj->IO.handle, 0x40, &tmp);
        if (ret != TLV320AIC3204_OK)
        {
            return TLV320AIC3204_ERROR; // Return immediately if ReadReg fails
        }

        ret = pObj->IO.WriteReg(pObj->IO.handle, 0x40, tmp | 0x0C);
        if (ret != TLV320AIC3204_OK)
        {
            return TLV320AIC3204_ERROR; // Return immediately if WriteReg fails
        }
    }
    else /* TLV320AIC3204_MUTE_OFF disables the Mute */
    {
        /* Left and Right DAC channels are not muted */
        ret = pObj->IO.ReadReg(pObj->IO.handle, 0x40, &tmp);
        if (ret != TLV320AIC3204_OK)
        {
            return TLV320AIC3204_ERROR; // Return if ReadReg fails
        }

        ret = pObj->IO.WriteReg(pObj->IO.handle, 0x40, tmp &= ~0x0C);
        if (ret != TLV320AIC3204_OK)
        {
            return TLV320AIC3204_ERROR; // Return if WriteReg fails
        }
    }

    return TLV320AIC3204_OK; // All operations succeeded
}


/**
  * @brief Start the audio Codec play feature.
  * @note For this codec no Play options are required.
  * @param  pObj pointer to component object
  * @retval Component status
  */
int32_t TLV320AIC3204_Play(tlv320aic3204_Object_t *pObj)
{
	/* Resumes the audio file playing */
	/* Unmute the output first */
	return TLV320AIC3204_SetMute(pObj, TLV320AIC3204_MUTE_OFF);
}


/**
  * @brief Pauses playing on the audio codec.
  * @param  pObj pointer to component object
  * @retval Component status
  */
int32_t TLV320AIC3204_Pause(tlv320aic3204_Object_t *pObj)
{
	int32_t ret = TLV320AIC3204_OK;


	/* Pause the audio file playing */
	/* Mute the output first */
	if(TLV320AIC3204_SetMute(pObj, TLV320AIC3204_MUTE_ON) != TLV320AIC3204_OK)
	{
		ret  = TLV320AIC3204_ERROR;
	}

	/* TODO Put the Codec in Power save mode */

	return ret;
}


/**
  * @brief Resumes playing on the audio codec.
  * @param  pObj pointer to component object
  * @retval Component status
  */
int32_t TLV320AIC3204_Resume(tlv320aic3204_Object_t *pObj)
{
	/* Resumes the audio file playing */
	/* Unmute the output first */
	return TLV320AIC3204_SetMute(pObj, TLV320AIC3204_MUTE_OFF);
}



/**
  * @brief Set higher or lower the codec volume level.
  * @param  pObj pointer to component object
  * @param  InputOutput Input or Output volume
  * @param  Volume  a byte value from 0 to 63 for output and from 0 to 240 for input
  *         (refer to codec registers description for more details).
  * @retval Component status
  */
int32_t TLV320AIC3204_SetVolume(tlv320aic3204_Object_t *pObj, uint32_t InputOutput, int8_t Volume)
{
    int32_t ret;

    /* Select Page 0 */
    ret = pObj->IO.WriteReg(pObj->IO.handle, 0x00, 0x00);
    if (ret != TLV320AIC3204_OK)
    {
        return TLV320AIC3204_ERROR;
    }

    /* Output volume */
    if (InputOutput == VOLUME_OUTPUT)
    {
    	// TODO implement negative Volume 0xFF = -0.5 dB, 0x81 = -63.5 dB, step 0.5 dB
        if (Volume < 24 && Volume > -63)
        {
            /* Unmute the audio codec
            ret = TLV320AIC3204_SetMute(pObj, TLV320AIC3204_MUTE_OFF);
            if (ret != TLV320AIC3204_OK)
            {
                return TLV320AIC3204_ERROR;
            }
			*/

            /* Set Left DAC Channel Digital Volume */
            ret = pObj->IO.WriteReg(pObj->IO.handle, 0x41, mapVolumeToDACReg(Volume));
            if (ret != TLV320AIC3204_OK)
            {
                return TLV320AIC3204_ERROR;
            }

            /* Set Right DAC Channel Digital Volume */
            ret = pObj->IO.WriteReg(pObj->IO.handle, 0x42, mapVolumeToDACReg(Volume));
            if (ret != TLV320AIC3204_OK)
            {
                return TLV320AIC3204_ERROR;
            }
        }
    }
    else /* Input volume */
    {
    	// TODO implement negative Volume 0x7F = -0.5 dB, 0x68 = -12 dB, step 0.5 dB
        if (Volume < 0x28U && Volume > 0x00U)
        {
            /* Set Left AIF1 ADC1 volume */
            ret = pObj->IO.WriteReg(pObj->IO.handle, 0x53, mapVolumeToADCRegister(Volume));
            if (ret != TLV320AIC3204_OK)
            {
                return TLV320AIC3204_ERROR;
            }

            /* Set Right AIF1 ADC1 volume */
            ret = pObj->IO.WriteReg(pObj->IO.handle, 0x54, mapVolumeToADCRegister(Volume));
            if (ret != TLV320AIC3204_OK)
            {
                return TLV320AIC3204_ERROR;
            }
        }
    }

    return TLV320AIC3204_OK; // All operations succeeded
}





/*
 *
 * Register manipulation functions
 *
 */

int32_t tlv320aic3204_sw_reset_w(tlv320aic3204_IO_t   *IO, uint8_t val)
{
	//select Page 0
	IO->WriteReg(IO->handle, PAGE_SELECT_REGISTER, 0);
	// make software reset
	IO->WriteReg(IO->handle, 0x01, val);

	return TLV320AIC3204_OK;
}


int32_t tlv320aic3204_sw_reset_r(tlv320aic3204_IO_t   *IO, uint8_t* val)
{
	//select Page 0
	IO->WriteReg(IO->handle, PAGE_SELECT_REGISTER, 0);
	// make software reset
	IO->ReadReg(IO->handle, 0x01, val);


	return TLV320AIC3204_OK;
}




/*------------------------------------------------------------------------------
                           Helper functions
------------------------------------------------------------------------------*/

static uint8_t mapVolumeToDACReg(int8_t dB)
{
    // Check if dB is within the allowable range
    if (dB < MIN_DB_DAC || dB > MAX_DB_DAC) {
        // Return a default value or an error indicator (could be 0 or a specific invalid value)
        return 0;  // Example: return a reserved value to indicate invalid dB
    }

    // Convert dB to the register value
    uint8_t register_value;

    if (dB > 0) {
		// Values from 0 dB up to +24 dB map to binary values from 0x00 to 0x30
		register_value = dB << 1;
	} else if (dB == 0) {
			// 0 dB maps to 0x00
			register_value = 0x00;
	} else {
		// Values below 0 dB map to binary values from 0xFF down to 0x82
		register_value = 0xFF + (int8_t)(dB << 1) + 1;  // Adjust for negative range
	}

    return register_value;

}


// Function to map dB value to register value
static uint8_t mapVolumeToADCRegister(int8_t dB)
{
    // Check if dB is within the allowable range
    if (dB < MIN_DB_ADC || dB > MAX_DB_ADC) {
        // Return a default value or an error indicator (could be 0 or a specific invalid value)
        return 0;  // Example: return a reserved value to indicate invalid dB
    }

    // Convert dB to the register value
	uint8_t register_value;

	if (dB > 0) {
		// Values from 0 dB up to +24 dB map to binary values from 0x00 to 0x30
		register_value = dB << 1;
	} else if (dB == 0) {
	        // 0 dB maps to 0x00
	        register_value = 0x00;
	} else {
		// Values below 0 dB map to binary values from 0xFF down to 0x82
		register_value = 0x7F + (int8_t)(dB << 1) + 1;  // Adjust for negative range
	}

	return register_value;

}
