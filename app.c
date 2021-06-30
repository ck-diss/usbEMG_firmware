/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

#include "peripheral/oc/plib_oc.h"
#include "peripheral/osc/plib_osc.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/
#define VB1_ACTIVATE   LATCbits.LATC6   
#define VB2_ACTIVATE   LATCbits.LATC7   
#define VB3_ACTIVATE   LATCbits.LATC8   
#define VB4_ACTIVATE   LATCbits.LATC9  

APP_DATA appData;
USB_DATA usbAppData;
SPI_DATA spiAppData;
I2C_STATES i2cAppState;
TIMER_STATES timerState;
PORTS port;
LEDS leds;

bool startI2CTransfer;


static char firstTriggerFlag = 0;
static char countTime = 0;

unsigned int InitChk = 0;

unsigned char i;
unsigned int timerVal;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 * System Timer CallBack
*/
// every second
/*void Timer1CallBack()
{
    /* Toggle LED *

    if (count<=7) {
        i2cTurnLED(I2C_LED_OFF, I2C_LED1 + count );
    }        
    else {
        i2cTurnLED(I2C_LED_ON, 0x80 | (I2C_LED1 + (count-8))); // 0x80 -> autoincrement
    }
    count++;
    if (count>=16) {
        count=0;
        //LEDToggle(LED_power,PORT_B);
        LEDToggle(LED_status, PORT_A);
    }
    countTime++;
    // Read EEPROM once after 20 secs - testing purpose
    if (countTime==1) {
        eeprom_reg_address = 0x01;    //0x00 ist ID read-only
        eeprom_data[0] = 0b10110001;// config1
        eeprom_data[1] = 0b11000000;// config2
        eeprom_data[2] = 0b01110000;// config3
        eeprom_data[3] = 0x00;      // loff
        eeprom_data[4] = 0x10;      // ch1set
        eeprom_data[5] = 0x10;      // ch2set
        eeprom_data[6] = 0x10;      // ch3set
        eeprom_data[7] = 0x10;      // ch4set
        i2cAppState = I2C_WRITE_EEPROM;
    }
    if (countTime==2) {
        eeprom_reg_address += 0x08;    // 
        eeprom_data[0] = 0x10;       // ch5set
        eeprom_data[1] = 0x10;       // ch6set
        eeprom_data[2] = 0x10;       // ch7set
        eeprom_data[3] = 0x00;       // ch8set
        eeprom_data[4] = 0x00;       // bias_sensp
        eeprom_data[5] = 0x00;       // bias_sensn
        eeprom_data[6] = 0x00;       // loff_sensp
        eeprom_data[7] = 0x00;       // loff_sensn
        i2cAppState = I2C_WRITE_EEPROM;
    }
    if (countTime==3) {
        eeprom_reg_address += 0x08;    // 
        eeprom_data[0] = 0x00;       // loff_flip
        eeprom_data[1] = 0;           // readonly - loff_statp
        eeprom_data[2] = 0;           // readonly - loff_statn
        eeprom_data[3] = 0x0F;       // gpio
        eeprom_data[4] = 0x00;       // misc1
        eeprom_data[5] = 0x00;       // misc2
        eeprom_data[6] = 0x00;       // config4
        eeprom_data[7] = 0;       // 
        i2cAppState = I2C_WRITE_EEPROM;
    }
     
    if (countTime==5) {
        eeprom_reg_address = 0x01;
        max_len_of_readEEPromBytes = 24;
        i2cAppState = I2C_READ_EEPROM;
    }
        
}*/

//void Timer2CallBack()
//{
//    LEDToggle(LED_power,PORT_B);
//}
//
// /* 
// *  CallBack
//  */
//void ISR_Callback_2 ( uintptr_t context, uint32_t currTick )
//{
//    //LEDToggle(LED_status, PORT_A);
//}

/*******************************************************************
 * Interrupt service routine: external Interrupt - Trigger usbCLK  *
 *******************************************************************/
void TriggerCallBack () {
    //DRV_TMR2_CounterValueSet(0);
    
    //TRY to fix offset due to master having a faster response ca 100us
    DRV_TMR2_CounterValueSet(0);

    //set flag, on next DMA packet reset timestamp
    //TRIGGER_FLAG = true;
    /*if (firstTriggerFlag==0) {
        // für 1 sec timer2 starten -> grünes blinken
        //timerVal = DRV_TMR0_CounterValueGet();
        //if (timerVal <187500) {
        DRV_TMR1_Start();
        firstTriggerFlag=1;
    }
    else {
        DRV_TMR1_Stop();
    } */
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
void LEDToggle(LEDS led)
{
    if (led==LED_power){
        port = PORT_B;
    }     
    else if (led==LED_status){
        port = PORT_A;
    }  
    PLIB_PORTS_PinToggle(PORTS_ID_0, port, led );

}

void LEDOn(LEDS led)
{
    if (led==LED_power){
        port = PORT_B;
        PLIB_OC_PulseWidth16BitSet(OC_ID_GREEN, OC_MAX_GREEN);
    }     
    else if (led==LED_status){
        port = PORT_A;
        PLIB_OC_PulseWidth16BitSet(OC_ID_YELLOW, OC_MAX_YELLOW);

    }  
    PLIB_PORTS_PinWrite (PORTS_ID_0, port, led, LED_STATE_ON);
}

void LEDOff(LEDS led)
{
    if (led==LED_power){
        port = PORT_B;
        PLIB_OC_PulseWidth16BitSet(OC_ID_GREEN, OC_OFF);

    }     
    else if (led==LED_status){
        port = PORT_A;
        PLIB_OC_PulseWidth16BitSet(OC_ID_YELLOW, OC_OFF);
    }  
    PLIB_PORTS_PinWrite (PORTS_ID_0, port, led, LED_STATE_OFF);
}
/*******************************************************************
 * Function:	void flushBuffer (unsigned char*, unsigned char)   *
 *******************************************************************/
void flushBuffer (unsigned char* data, unsigned int length)
{
	unsigned int i;
	for (i=0; i < length; i++)
		data[i] = 0x00;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

int LED_Initialize(void )
{
    /* Switch off all the LEDS */ 
    PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_A, LED_status );
    PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_B, LED_power );
    for (i=0;i<sizeof(ledVals);i++) {
        ledVals[i] = 0;
    }


    return 1;
}

void spi_Initialize ( void )
{
    spiAppData.state = SPI_ENABLE;
}

/* Setup OCMP 1 module - Use timer 2, output is an edge aligned signal,
      disable PWM faults, duty cycle and compare values will have 16-bit values,
      buffer value is set to 0, pulse width (duty cycle) value is set to 500
      - The PWM remains high until the timer reaches 500. The PWM then goes
      - low until the timer hits its period of 2000, rolling over to repeat
      - the process again. 500/2,000 = 25% Duty Cycle */
//void OC_Initialize (void) { 
//    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_OC1, OUTPUT_PIN_RPB3);
//    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_3);
//            
//     /* Stellen Sie die PBCLK Teiler 2 und aktivieren PBCLK3 (f?r Timer 2) */
//    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_1, 2); // Teilungsfaktor 2 ist, so dass die Frequenz des Bus-Timer 24mhz
//    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_1);
//
//    PLIB_OC_TimerSelect(OC_ID_1, OC_TIMER_16BIT_TMR2);
//    PLIB_OC_ModeSelect(OC_ID_1, OC_COMPARE_PWM_EDGE_ALIGNED_MODE);
//    PLIB_OC_FaultInputSelect(OC_ID_1, OC_FAULT_DISABLE);
//    PLIB_OC_BufferSizeSelect(OC_ID_1, OC_BUFFER_SIZE_16BIT);
//    PLIB_OC_Buffer16BitSet(OC_ID_1, 0);
//    PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
//    
//    PLIB_OC_Enable(OC_ID_1);
//    PLIB_TMR_Start(TMR_ID_2);
//     /*Enable timer 2 interrupts*/
//    //PLIB_INT_Enable(INT_ID_0);
//    
//    PLIB_OC_PulseWidth16BitSet(OC_ID_1,13000); //33% Duty
//
//}

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

/*void frcToPll(){

/*    mOSCSetPBDIV(OSC_PB_DIV_1);
    OSCConfig(OSC_POSC_PLL, OSC_PLL_MULT_20, OSC_PLL_POST_2, 0);
    SYSTEMConfig(GetPLLSystemClock(), SYS_CFG_ALL);
}*/

void APP_Initialize ( void )
{
    
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;  
    
    // Activate Sensor Supply on startup
    VB1_ACTIVATE = 1;
    VB2_ACTIVATE = 1;
    VB3_ACTIVATE = 1;
    VB4_ACTIVATE = 1;
    
        
    
    InitChk += usb_Initialize();
    InitChk += Timer_Initialize();
    InitChk += LED_Initialize();  
    InitChk += dma_Initialize(); 
    InitChk += device_Initialize(); 
    
    spi_Initialize();
    
    startI2CTransfer = true;
    
    // Start Timer
    timerState = TIMER_STATE_START;  
    
    // Create the CRC16-CCIT Look up table 
    CRC16_Initialize(); 
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // Check if all initializations are completed 
            // If every modules are OK, then Power LED is turned ON
            //if (InitChk == 5)
            //{
            //}
            appData.state = APP_STATE_MAIN_TASK;
        }
        
        // usbEMG main service routine
        case APP_STATE_MAIN_TASK:
        {
            usbService();
            i2cService();
            timerService();
            spiService();            
            deviceService();            
            break;
        }
        
        //In case of usbEMG device needs to be reset!!!
        case APP_STATE_RESET:    
        {
            InitChk = 0;
            APP_Initialize ();
            appData.state = APP_STATE_INIT;
        }
        case APP_STATE_ERROR:
        {
            break;
        }
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 
// git test


/*******************************************************************************
 End of File
 */
