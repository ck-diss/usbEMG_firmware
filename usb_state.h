/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _USB_STATE_H    /* Guard against multiple inclusion */
#define _USB_STATE_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
#include "app.h"


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */
#define USB_COMMAND_PACKET                          0x55

#define USB_ADS_PACKET                              0xAD

#define USB_MCU_INFO_PACKET                         0x00
#define USB_MCU_GET_UNIX_TIMESTAMP_32_BIT_PACKET    0xF0
#define USB_MCU_SET_UNIX_TIMESTAMP_32_BIT_PACKET    0xF1
    
#define USB_MCU_GET_DAQ_TIMER_24_BIT_PACKET         0xF2
#define USB_MCU_SET_DAQ_TIMER_24_BIT_PACKET         0xF3

#define USB_MCU_GET_3V_SUPPLY_PACKET                0xF4
#define USB_MCU_SET_3V_SUPPLY_PACKET                0xF5
#define USB_MCU_SET_3V_SUPPLY_CH1_CH2               0x01
#define USB_MCU_SET_3V_SUPPLY_CH3_CH4               0x02
#define USB_MCU_SET_3V_SUPPLY_CH5_CH6               0x04
#define USB_MCU_SET_3V_SUPPLY_CH7_CH8               0x08
#define USB_MCU_SET_3V_SUPPLY_OFF                   0x00
    
#define USB_MCU_GET_STATUS_LED_PACKET               0xF6
#define USB_MCU_SET_STATUS_LED_PACKET               0xF7
    
#define USB_MCU_SEND_ERROR_PACKET                   0xEE

#define USB_LED_DRIVER_PACKET                       0xD0
    
#define USB_EEPROM_PACKET                           0xD1
    
#define USB_DEVICE_RESET                            0x80
    
#define USB_TEST_LED_ON                             0x81 
#define USB_TEST_LED_OFF                            0x82 
    
int rb_usb_readout_overflow_cnt;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

typedef enum
{
    /* Application's state machine's initial state. */
    USB_STATE_INIT=0,

    /* Application waits for device configuration */
    USB_STATE_WAIT_FOR_CONFIGURATION,

    /* Application runs the main task */
    USB_STATE_MAIN_TASK,

    /* Application error occurred */
    USB_STATE_ERROR

} USB_STATES;


typedef struct
{
   /* Device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE usbDevHandle;

    /* Application state*/
    USB_STATES state;

    /* Track device configuration */
    bool deviceIsConfigured;

    /* Configuration value */
    uint8_t configValue;

    /* speed */
    USB_SPEED speed;

    /* ep data sent */
    bool epDataWritePending;

    /* ep data received */
    bool epDataReadPending;

    /* Transfer handle */
    USB_DEVICE_TRANSFER_HANDLE writeTranferHandle;

    /* Transfer handle */
    USB_DEVICE_TRANSFER_HANDLE readTranferHandle;

    /* The transmit endpoint address */
    USB_ENDPOINT_ADDRESS endpointTx;

    /* The receive endpoint address */
    USB_ENDPOINT_ADDRESS endpointRx;
    
    /* The transmit BULK data endpoint address */
    USB_ENDPOINT_ADDRESS dataendpointTx;

    /* Tracks the alternate setting */
    uint8_t altSetting;
    
} USB_DATA;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************

int usb_Initialize( void );

void usbService( void );

bool USBsendData (unsigned char*, unsigned int*);

bool USBSendCommand(unsigned char* rb_buffer);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
