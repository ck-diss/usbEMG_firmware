/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
extern RUNTIME_DATA runtimeData;

void __ISR(_EXTERNAL_4_VECTOR, IPL4AUTO) _IntHandlerExternalInterruptInstance0(void)
{           
    DRDYCallBack();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_4);

}

void __ISR(_EXTERNAL_0_VECTOR, IPL5AUTO) _IntHandlerExternalInterruptInstance1(void)
{           
    TriggerCallBack();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_0);

}

void __ISR(_TIMER_5_VECTOR, ipl1AUTO) _IntHandlerDrvTmrInstance0(void)
{
    TimerOneSecondCallBack(); 
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);

}
void __ISR(_TIMER_1_VECTOR, ipl1AUTO) _IntHandlerDrvTmrInstance1(void)
{
    Timer1CallBack(); 
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_1);

}
void __ISR(_TIMER_2_VECTOR, ipl4AUTO) _IntHandlerDrvTmrInstance2(void)
{
    Timer10msCallBackTrigger(); 
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
void __ISR(_TIMER_3_VECTOR, ipl1AUTO) _IntHandlerDrvTmrInstance3(void)
{
    //OUTPUT COMPARE TIMER
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);

}
 


void __ISR(_DMA0_VECTOR, ipl6AUTO) _IntHandlerSysDmaCh0(void)
{          
    DMA0CallBack();
    /* Clear the interrupt flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_DMA_0);
    
    /* TODO: Add code as needed */ 

}

void __ISR(_DMA1_VECTOR, ipl6AUTO) _IntHandlerSysDmaCh1(void)
{          
    DMA1CallBack();
    /* Clear the interrupt flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_DMA_1);
    
    /* TODO: Add code as needed */ 

}


 	
	
	
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{

    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
            
}


 
/*******************************************************************************
 End of File
*/

