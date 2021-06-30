/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef _DELAY_H
#define	_DELAY_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <GenericTypeDefs.h> // include for UINT32  

#define SYS_FREQ    48000000ul
#define us_SCALE    (SYS_FREQ/2000000) 
#define ms_SCALE    (SYS_FREQ/2000) 

unsigned int startCnt;
unsigned int waitCnt;

static inline void __attribute__((always_inline)) delay_us(UINT32 usDelay ) 
{ 
    startCnt = _CP0_GET_COUNT(); 
    waitCnt = usDelay * us_SCALE; 

    while( _CP0_GET_COUNT() - startCnt < waitCnt ); 
} 
 
static inline void __attribute__((always_inline)) delay_ms(UINT32 msDelay ) 
{ 
    startCnt = _CP0_GET_COUNT(); 
    waitCnt = msDelay * ms_SCALE; 

    while( _CP0_GET_COUNT() - startCnt < waitCnt ); 
} 
/* Delay routine to provide application level delays*/

void delay_nop(unsigned int delay);


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

