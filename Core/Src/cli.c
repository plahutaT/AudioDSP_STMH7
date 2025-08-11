/*
 * cli.c
 *
 *  Created on: Nov 6, 2024
 *      Author: tilen
 */

#include "cli.h"

extern LowPass_FirstOrder_t LowPass_Filt_L;
extern LowPass_FirstOrder_t LowPass_Filt_R;


// Define supported commands and their handlers
CLI_Command_t cli_commands[] = {
	{ "PING", HandlePing },
    { "MUTEON", HandleMuteOn },
    { "MUTEOFF", HandleMuteOff },
    { "SETVOL", HandleSetVolume },
	{ "SETFC", HandleSetFc },
};


int32_t HandlePing(void *args)
{
	char response[30];

    sprintf(response, "HELLO_FROM_STM\n");
    if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK)
    {
    	return TLV320AIC3204_ERROR;
    }

    return TLV320AIC3204_OK;
}


int32_t HandleMuteOn(void *args)
{
	int32_t ret;
	char response[30];
	/*
    ret = TLV320AIC3204_Driver.SetMute(&TLV320AIC3204_Obj, TLV320AIC3204_MUTE_ON);
    if (ret != TLV320AIC3204_OK)
	{
		return TLV320AIC3204_ERROR;
	}
	*/
    sprintf(response, "MUTE_ON_OK\n");
    if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK)
    {
    	return TLV320AIC3204_ERROR;
    }

    return TLV320AIC3204_OK;
}

int32_t HandleMuteOff(void *args)
{
	int32_t ret;
	char response[30];
	/*
	ret = TLV320AIC3204_Driver.SetMute(&TLV320AIC3204_Obj, TLV320AIC3204_MUTE_OFF);
	if (ret != TLV320AIC3204_OK)
	{
		return TLV320AIC3204_ERROR;
	}
	*/
	sprintf(response, "MUTE_OFF_OK\n");
	if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK)
	{
		return TLV320AIC3204_ERROR;
	}

	return TLV320AIC3204_OK;
}

int32_t HandleSetVolume(void *args)
{
	int32_t ret;
    uint32_t inputOutput;
    uint8_t volume;
    char *arg_str = (char *)args;
    char *type_str = NULL;
    char *vol_str = NULL;

    char response[50];

    // Split the argument into "0/1" and "volume"
	type_str = strtok(arg_str, " ");   // Get 0/1
	vol_str = strtok(NULL, " ");       // Get volume value

	if (type_str == NULL || vol_str == NULL) {
		//printf("Invalid arguments. Format: SETVOL <0/1> <volume>\n");
		return -1;
	}

	// Convert type_str to integer (0 or 1)
	inputOutput = strtol(type_str, NULL, 10);
	if (inputOutput != 0 && inputOutput != 1) {
		//printf("Invalid type. Use '0' for input or '1' for output.\n");
		return TLV320AIC3204_ERROR;
	}

	// Convert volume from string to integer
	volume = (int8_t)strtol(vol_str, NULL, 10);


	ret = TLV320AIC3204_Driver.SetVolume(&TLV320AIC3204_Obj, inputOutput, volume);
	if (ret != TLV320AIC3204_OK)
	{
		return TLV320AIC3204_ERROR;
	}


    sprintf(response, "%lu Volume set to %u\n", inputOutput, volume);
    if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK)
	{
		return TLV320AIC3204_ERROR;
	}

    return ret;
}

int32_t HandleSetFc(void *args)
{
    int32_t ret = 0; // Assume success unless an error occurs
    uint32_t cutoffFrequency;
    char *arg_str = (char *)args;
    char *freq_str = NULL;

    char response[50];

    // Split the argument to extract the frequency value
    freq_str = strtok(arg_str, " "); // Get frequency value

    if (freq_str == NULL) {
        // Invalid arguments
        return -1;
    }

    // Convert frequency from string to integer
    cutoffFrequency = strtol(freq_str, NULL, 10);

    if (cutoffFrequency < 20 || cutoffFrequency > 20000) {
        // Assuming cutoff frequency must be in the range of 20 Hz to 20 kHz
        return -1;
    }

    // Perform the operation to set the cutoff frequency
    // Uncomment and implement this as needed for your system
    LowPass_FirstOrder_SetFc(&LowPass_Filt_L, cutoffFrequency);
    LowPass_FirstOrder_SetFc(&LowPass_Filt_L, cutoffFrequency);

    // Prepare and send the response
    sprintf(response, "Cutoff frequency set to %lu Hz\n", cutoffFrequency);
    if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK) {
        return -1;
    }

    return ret;
}



// Function to process received command
int32_t cli_processCommand(uint8_t *command)
{
    char response[50];
    char command_name[20];
    uint8_t *args = NULL;

    // Remove any newline or carriage return characters from the received command
    command[strcspn((char*)command, "\r\n")] = 0;

    // Split command and arguments
    args = (uint8_t*)strchr((char*)command, ' ');
    if (args != NULL) {
        // Terminate command_name at the space, and advance args pointer
        strncpy(command_name, (char*)command, args - command);
        command_name[args - command] = '\0';
        args++;  // Point to the first character of the argument part
    } else {
        // No arguments, command_name is the entire command
        strcpy(command_name, (char*)command);
    }

    // Look for a matching command
    for (int i = 0; i < sizeof(cli_commands) / sizeof(CLI_Command_t); i++) {
        if (strcmp(command_name, cli_commands[i].command) == 0) {
            // Call the handler for the matched command with arguments
            cli_commands[i].handler(args);
            return TLV320AIC3204_OK;
        }
    }

    // If no match is found
    sprintf(response, "Invalid command\n");
    if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK) {
        return TLV320AIC3204_ERROR;
    }

    return TLV320AIC3204_OK;
}

