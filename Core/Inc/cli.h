/*
 * cli.h
 *
 *  Created on: Nov 6, 2024
 *      Author: tilen
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include "usb_device.h"      // Include the USB Device header
#include "usbd_cdc_if.h"     // Include the USB CDC interface header
#include "tlv320aic3204.h"  // Include your codec header file
#include "filter_fisrtorder.h"
#include <string.h>  // For string functions like strcmp

// Define command structure
typedef struct {
    char *command;                      // Command string
    int32_t (*handler)(void *args);         // Handler function for the command
} CLI_Command_t;

// Declare the list of supported commands
extern tlv320aic3204_Object_t TLV320AIC3204_Obj;
// Define command maximum length
#define MAX_CMD_LENGTH 50

// Function prototypes
int32_t HandlePing(void *args);
int32_t HandleMuteOn(void *args);
int32_t HandleMuteOff(void *args);
int32_t HandleSetVolume(void *args);
int32_t HandleSetFc(void *args);
int32_t cli_processCommand(uint8_t *command);
int32_t cli_sendData(uint8_t *data, uint16_t length);




#endif /* INC_CLI_H_ */
