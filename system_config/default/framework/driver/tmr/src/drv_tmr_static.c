/*******************************************************************************
  Timer Static Driver File

  File Name:
    drv_tmr_static.c

  Company:
    Microchip Technology Inc.	

  Summary:
    Timer driver implementation for the static single instance driver.

  Description:
    The Timer device driver provides a simple interface to manage the Timer
    modules on Microchip microcontrollers.
    
  Remarks:
    None
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Mirochip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a MTMRrochip mTMRrocontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublTMRense terms in the accompanying lTMRense agreement).

You should refer to the lTMRense agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTTMRULAR PURPOSE.
IN NO EVENT SHALL MTMRROCHIP OR ITS LTMRENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRTMRT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVTMRES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "framework/driver/tmr/drv_tmr_static.h"

 // *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_TMR0_Initialize(void)
{	
    /* Initialize Timer Instance0 */
    /* Disable Timer */
    PLIB_TMR_Stop(TMR_ID_4);
    /* Select clock source */
    PLIB_TMR_ClockSourceSelect(TMR_ID_4, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK);
    /* Select prescalar value */
    PLIB_TMR_PrescaleSelect(TMR_ID_4, TMR_PRESCALE_VALUE_256);
    /* Enable 32 bit mode */
    PLIB_TMR_Mode32BitEnable(TMR_ID_4);
    /* Clear counter */
    PLIB_TMR_Counter32BitClear(TMR_ID_4);
    /*Set period */	
    PLIB_TMR_Period32BitSet(TMR_ID_4, 187500);

    /* Setup Interrupt */   
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_5);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_T5, INT_PRIORITY_LEVEL1);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_T5, INT_SUBPRIORITY_LEVEL0);          
}

inline void DRV_TMR0_Start(void)
{
    /* Start Timer*/
    PLIB_TMR_Start(TMR_ID_4);
}

inline void DRV_TMR0_Stop(void)
{
    /* Stop Timer*/
    PLIB_TMR_Stop(TMR_ID_4);
}

inline void DRV_TMR0_CounterClear(void)
{
    /* Clear 32-bit counter value */
    PLIB_TMR_Counter32BitClear(TMR_ID_4);
}

void DRV_TMR0_CounterValueSet(uint32_t value)
{
    /* Set 32-bit counter value*/
    PLIB_TMR_Counter32BitSet(TMR_ID_4, value);
}

uint32_t DRV_TMR0_CounterValueGet(void)
{
    /* Get 32-bit counter value*/
    return PLIB_TMR_Counter32BitGet(TMR_ID_4);
}

// *****************************************************************************
// *****************************************************************************
// Section: Instance 1 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_TMR1_Initialize(void)
{	
    /* Initialize Timer Instance1 */
    /* Disable Timer */
    PLIB_TMR_Stop(TMR_ID_1);
    /* Select clock source */
    PLIB_TMR_ClockSourceSelect(TMR_ID_1, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK);
    /* Select prescalar value */
    PLIB_TMR_PrescaleSelect(TMR_ID_1, TMR_PRESCALE_VALUE_8);
    /* Enable 16 bit mode */
    PLIB_TMR_Mode16BitEnable(TMR_ID_1);
    /* Clear counter */	
    PLIB_TMR_Counter16BitClear(TMR_ID_1);
    /*Set period */	
    PLIB_TMR_Period16BitSet(TMR_ID_1, 65535);

    /* Setup Interrupt */   
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_1);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_T1, INT_PRIORITY_LEVEL1);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0);          
}

inline void DRV_TMR1_Start(void)
{
    /* Start Timer*/
    PLIB_TMR_Start(TMR_ID_1);
}

inline void DRV_TMR1_Stop(void)
{
    /* Stop Timer*/
    PLIB_TMR_Stop(TMR_ID_1);
}

inline void DRV_TMR1_CounterClear(void)
{
    /* Clear 16-bit counter value */
    PLIB_TMR_Counter16BitClear(TMR_ID_1);    
}

void DRV_TMR1_CounterValueSet(uint32_t value)
{
    /* Set 16-bit counter value*/
    PLIB_TMR_Counter16BitSet(TMR_ID_1, (uint16_t)value);
}

uint32_t DRV_TMR1_CounterValueGet(void)
{
    /* Get 16-bit counter value*/
    return (uint32_t) PLIB_TMR_Counter16BitGet(TMR_ID_1);
}

// *****************************************************************************
// *****************************************************************************
// Section: Instance 2 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_TMR2_Initialize(void)
{	
    /* Initialize Timer Instance2 */
    /* Disable Timer */
    PLIB_TMR_Stop(TMR_ID_2);
    /* Select clock source */
    PLIB_TMR_ClockSourceSelect(TMR_ID_2, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK);
    /* Select prescalar value */
    PLIB_TMR_PrescaleSelect(TMR_ID_2, TMR_PRESCALE_VALUE_8);
    /* Enable 16 bit mode */
    PLIB_TMR_Mode16BitEnable(TMR_ID_2);
    /* Clear counter */	
    PLIB_TMR_Counter16BitClear(TMR_ID_2);
    /*Set period */	
    PLIB_TMR_Period16BitSet(TMR_ID_2, 60000);

    /* Setup Interrupt */   
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_T2, INT_PRIORITY_LEVEL4);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_T2, INT_SUBPRIORITY_LEVEL1);          
}

inline void DRV_TMR2_Start(void)
{
    /* Start Timer*/
    PLIB_TMR_Start(TMR_ID_2);
}

inline void DRV_TMR2_Stop(void)
{
    /* Stop Timer*/
    PLIB_TMR_Stop(TMR_ID_2);
}

inline void DRV_TMR2_CounterClear(void)
{
    /* Clear 16-bit counter value */
    PLIB_TMR_Counter16BitClear(TMR_ID_2);    
}

void DRV_TMR2_CounterValueSet(uint32_t value)
{
    /* Set 16-bit counter value*/
    PLIB_TMR_Counter16BitSet(TMR_ID_2, (uint16_t)value);
}

uint32_t DRV_TMR2_CounterValueGet(void)
{
    /* Get 16-bit counter value*/
    return (uint32_t) PLIB_TMR_Counter16BitGet(TMR_ID_2);
}

// *****************************************************************************
// *****************************************************************************
// Section: Instance 3 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_TMR3_Initialize(void)
{	
    /* Initialize Timer Instance3 */
    /* Disable Timer */
    PLIB_TMR_Stop(TMR_ID_3);
    /* Select clock source */
    PLIB_TMR_ClockSourceSelect(TMR_ID_3, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK);
    /* Select prescalar value */
    PLIB_TMR_PrescaleSelect(TMR_ID_3, TMR_PRESCALE_VALUE_32);
    /* Enable 16 bit mode */
    PLIB_TMR_Mode16BitEnable(TMR_ID_3);
    /* Clear counter */	
    PLIB_TMR_Counter16BitClear(TMR_ID_3);
    /*Set period */	
    PLIB_TMR_Period16BitSet(TMR_ID_3, 24000);

    /* Setup Interrupt */   
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_3);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_T3, INT_PRIORITY_LEVEL1);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_T3, INT_SUBPRIORITY_LEVEL0);          
}

inline void DRV_TMR3_Start(void)
{
    /* Start Timer*/
    PLIB_TMR_Start(TMR_ID_3);
}

inline void DRV_TMR3_Stop(void)
{
    /* Stop Timer*/
    PLIB_TMR_Stop(TMR_ID_3);
}

inline void DRV_TMR3_CounterClear(void)
{
    /* Clear 16-bit counter value */
    PLIB_TMR_Counter16BitClear(TMR_ID_3);    
}

void DRV_TMR3_CounterValueSet(uint32_t value)
{
    /* Set 16-bit counter value*/
    PLIB_TMR_Counter16BitSet(TMR_ID_3, (uint16_t)value);
}

uint32_t DRV_TMR3_CounterValueGet(void)
{
    /* Get 16-bit counter value*/
    return (uint32_t) PLIB_TMR_Counter16BitGet(TMR_ID_3);
}

 
 
/*******************************************************************************
 End of File
*/
