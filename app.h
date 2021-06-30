/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "peripheral/oc/plib_oc.h"

#include "delay.h"
#include "device_information.h"
#include "usb_state.h"
#include "i2c_state.h"
#include "spi_state.h"
#include "dma_state.h"
#include "ads.h"
#include "timer_state.h"
#include "CRC16.h"
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

    // *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define CH1_CH2_3V_SUPPLY_ON() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_6, 1);
#define CH1_CH2_3V_SUPPLY_OFF() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_6, 0);
#define CH3_CH4_3V_SUPPLY_ON() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_7, 1);
#define CH3_CH4_3V_SUPPLY_OFF() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_7, 0);
#define CH5_CH6_3V_SUPPLY_ON() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_8, 1);
#define CH5_CH6_3V_SUPPLY_OFF() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_8, 0);
#define CH7_CH8_3V_SUPPLY_ON() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_9, 1);
#define CH7_CH8_3V_SUPPLY_OFF() PLIB_PORTS_PinWrite (PORTS_ID_0, PORT_C, PORTS_BIT_POS_9, 0);


#define SAMPLE_LENGTH 27
//#define USB_PACKET_SIZE 64
//#define RB_LENGTH 62208		// maximum buffer size (27*64*36 = 62208)
#define RB_LENGTH 58752		// maximum buffer size (27*64*34 = 58752)
//#define RB_LENGTH 51840		// buffer size (27*64*30 = 51840)
//#define RB_LENGTH 34560		// buffer size (27*64*20 = 34560)
//#define RB_LENGTH SAMPLE_LENGTH * 64 * 25		// buffer size (27*64*25 = 43200)
//#define RB_LENGTH 17280		// buffer size (27*64*10 = 17280)
//#define RB_LENGTH 3456          // buffer size (2 * 1728) 
//#define RB_LENGTH 1728          // buffer size (27*64 = 1728) kgV
//#define RB_LENGTH 100

    
// OutputCompare
#define OC_ID_GREEN         OC_ID_1
#define OC_ID_YELLOW        OC_ID_2
#define OC_MAX_GREEN        23500
#define OC_MAX_YELLOW       20000
#define OC_OFF              24000
    

    
unsigned char rb_buffer[RB_LENGTH];						// ring buffer
unsigned char dumb_buffer[SAMPLE_LENGTH];				// dumb data
unsigned char command_buffer [USB_PACKET_SIZE];
/*unsigned int status_buffer[RB_LENGTH/27];
unsigned int ch1_buffer[RB_LENGTH/27];
unsigned int ch2_buffer[RB_LENGTH/27];
unsigned int ch3_buffer[RB_LENGTH/27];
unsigned int ch4_buffer[RB_LENGTH/27];
unsigned int ch5_buffer[RB_LENGTH/27];
unsigned int ch6_buffer[RB_LENGTH/27];
unsigned int ch7_buffer[RB_LENGTH/27];
unsigned int ch8_buffer[RB_LENGTH/27];*/
unsigned int rb_read;
unsigned int rb_write;
unsigned int rb_size;
unsigned int samples_cnt;								// count samples

unsigned char data[USB_PACKET_SIZE];

unsigned char i;

unsigned char ledVals[8];


// Visual Feedback flags
bool DEVICE_ON;
bool DEVICE_ERROR;
bool ADS_RECORDING;
bool ADS_READY;
bool ADS_ERROR;
bool ADS_ERROR;
bool CH1_TRANSMITTING;
bool CH2_TRANSMITTING;
bool CH3_TRANSMITTING;
bool CH4_TRANSMITTING;
bool CH5_TRANSMITTING;
bool CH6_TRANSMITTING;
bool CH7_TRANSMITTING;
bool CH8_TRANSMITTING;

//
bool TRIGGER_FLAG;



// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
    
    APP_STATE_INIT,
            
	APP_STATE_MAIN_TASK,
            
    APP_STATE_RESET,
            
    APP_STATE_ERROR


} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;

    /* TODO: Define any additional data used by the application. */


} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );


// *****************************************************************************
/* LED Number.

  Summary:
    Defines the LEDs available on this board.

  Description:
    This enumeration defines the LEDs available on this board.

  Remarks:
    None.
*/
typedef enum
{
    /* LED 1 */
    LED_status = PORTS_BIT_POS_9,
    LED_power = PORTS_BIT_POS_3,

} LEDS;

// *****************************************************************************
/* LED State

  Summary:
    Enumerates the supported LED states.

  Description:
    This enumeration defines the supported LED states.

  Remarks:
    None.
*/

typedef enum
{
    /* LED State is on */
    LED_STATE_OFF = 1,
    /* LED State is off */
    LED_STATE_ON = 0,

} LED_STATE;

/* PORT Number.

  Summary:
    Defines the PORTS available on this board.

  Description:
    This enumeration defines the LEDs available on this board.

  Remarks:
    None.
*/

typedef enum
{
    /* Port A */
    PORT_A = PORT_CHANNEL_A,
    PORT_B = PORT_CHANNEL_B,
    PORT_C = PORT_CHANNEL_C
} PORTS;

// *****************************************************************************
// *****************************************************************************
// Section: LED Functions
// *****************************************************************************
// *****************************************************************************

//void BSP_LEDStateSet(LEDS led, LED_STATE  state);

//LED_STATE  BSP_LEDStateGet(LEDS led);


void LEDToggle(LEDS led);

void LEDOn(LEDS led);

void LEDOff(LEDS led);

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/



// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */
#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

