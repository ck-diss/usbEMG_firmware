
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

#include "app.h"

/*******************************************************************************
  This source file has NOT been generated by the MHC
 *******************************************************************************/

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
extern RUNTIME_DATA runtimeData;
unsigned int SamplingRate = 8000;
int overflow_cnt = 0;
int runtimeDiff = 0;
unsigned char write1;
unsigned char write2;
unsigned int cnt_samples;
unsigned char temp;

// *****************************************************************************
// *****************************************************************************
// Section: User Functions
// *****************************************************************************
// *****************************************************************************
int dma_Initialize(void) {
    SYS_DMA_Channel0Setup((1<<4)); //SYS_DMA_CHANNEL_OP_MODE_AUTO
    SYS_DMA_Channel0TransferAdd();
    
    SYS_DMA_Channel1Setup((1<<4)); //SYS_DMA_CHANNEL_OP_MODE_AUTO
    SYS_DMA_Channel1TransferAdd();
    
    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_0, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    
    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DMA_CHANNEL_1, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DMA_CHANNEL_1, DMA_INT_BLOCK_TRANSFER_COMPLETE);
        
    return 1;
}

void dma_Buffer_Init(void) {
    // ******** ring buffer ***************************
    rb_read = 0;
	rb_write = 0;
	rb_size = 0;
	samples_cnt = 0;
    write1=0;
    write2=0;
    cnt_samples=0;
    // ADD!! Just comment for Debugging
	flushBuffer(&rb_buffer[0], RB_LENGTH);
	flushBuffer(&dumb_buffer[0], SAMPLE_LENGTH);       
}
    


/*******************************************************************
 * Interrupt service routine: dma channel 0 (receive)              *
 *******************************************************************/
void DMA0CallBack(void)
{
    cnt_samples++;
    //runtimeData.previous = runtimeData.current;
//    if (TRIGGER_FLAG) {
//        TRIGGER_FLAG = false;
//        DRV_TMR1_CounterValueSet(0);
//    }
    runtimeData.current = DRV_TMR2_CounterValueGet(); //TMR2 oder TMR1?? Beachte TriggerCallback in app.c
//    if (samples_cnt%64==0){
//        asm("nop");
//    }
    // if timer isn't resetted yet
    /*runtimeDiff = runtimeData.current - runtimeData.previous;
    if (runtimeDiff > 0) {
        runtimeData.toSave += runtimeDiff;
    } else {
        runtimeData.toSave += runtimeData.current;
    }
    //ensure 24bit value
    if (runtimeData.toSave > 0xFFFFFF) {
        runtimeData.toSave=0;
    }
        
    */
    
    //unsigned char temp;
    SYS_DMA_Channel0Disable();

    //STATUS BITS SETZEN
	//rb_buffer[rb_write+1] = (unsigned char) (samples_cnt >> 8);
	//rb_buffer[rb_write+2] = (unsigned char) samples_cnt;
	
    
    //DEBUG
//    for (i=0; i<3; i++) {
//        status_buffer[rb_write/9+i] = (unsigned char)rb_buffer[rb_write+i];
//        ch3_buffer[rb_write/9+i] = (unsigned char)rb_buffer[rb_write+9+i];
//        ch4_buffer[rb_write/9+i] = (unsigned char)rb_buffer[rb_write+12+i];
//        ch5_buffer[rb_write/9+i] = (unsigned char)rb_buffer[rb_write+15+i];
//        ch6_buffer[rb_write/9+i] = (unsigned char)rb_buffer[rb_write+18+i];        
//        ch7_buffer[rb_write/9+i] = (unsigned char)rb_buffer[rb_write+21+i];
//        ch8_buffer[rb_write/9+i] = (unsigned char)rb_buffer[rb_write+24+i];
//    }
//    status_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+0] << 16) | (rb_buffer[rb_write+1] << 8) | (rb_buffer[rb_write+2]);
//    ch1_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+3] << 16) | (rb_buffer[rb_write+4] << 8) | (rb_buffer[rb_write+5]);
//    ch2_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+6] << 16) | (rb_buffer[rb_write+7] << 8) | (rb_buffer[rb_write+8]);
//    ch3_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+9] << 16) | (rb_buffer[rb_write+10] << 8) | (rb_buffer[rb_write+11]);
//    ch4_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+12] << 16) | (rb_buffer[rb_write+13] << 8) | (rb_buffer[rb_write+14]);
//    ch5_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+15] << 16) | (rb_buffer[rb_write+16] << 8) | (rb_buffer[rb_write+17]);
//    ch6_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+18] << 16) | (rb_buffer[rb_write+19] << 8) | (rb_buffer[rb_write+20]);       
//    ch7_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+21] << 16) | (rb_buffer[rb_write+22] << 8) | (rb_buffer[rb_write+23]);
//    ch8_buffer[rb_write/SAMPLE_LENGTH] = (rb_buffer[rb_write+24] << 16) | (rb_buffer[rb_write+25] << 8) | (rb_buffer[rb_write+26]);
    
    // Setting of timestamp to the first three bytes of the rb_buffer
    // This should be taken from runtimeData.current: transform from DEC to HEX
    
    //runtimeData.current += (1 / SamplingRate) * (1000000);
    //runtimeData.current += 0x01;

    
    /*if (runtimeData.current >= 0xFFFFFF)
    {
        runtimeData.current = 0x000000;
    }*/
    
    // Prescaler 64 -> 1.333us = 4/3us
    //runtimeData.current = runtimeData.current;
    //runtimeData.current = runtimeData.current << 2;
    //runtimeData.current = runtimeData.current / 3;
//    write1 = rb_write+1;//deubg
//    write2 = rb_write+2;//debug
    
    // turnLEDs LeadOff Detection
    // enter only every 8000 sample (1sec if 8khz)
   if ((cnt_samples % 8000) == 0) {
       cnt_samples = 0;
        // channel 1
        if (((rb_buffer[rb_write+1] & 0x10) == 0x00) && ((rb_buffer[rb_write+2] & 0x10) == 0x00)) {ledVals[0]=I2C_LED_ON;} else {ledVals[0]=I2C_LED_OFF;}
        // channel 2
        if (((rb_buffer[rb_write+1] & 0x20) == 0x00) && ((rb_buffer[rb_write+2] & 0x20) == 0x00)) {ledVals[1]=I2C_LED_ON;} else {ledVals[1]=I2C_LED_OFF;}
        // channel 3
        if (((rb_buffer[rb_write+1] & 0x40) == 0x00) && ((rb_buffer[rb_write+2] & 0x40) == 0x00)) {ledVals[2]=I2C_LED_ON;} else {ledVals[2]=I2C_LED_OFF;}
        // channel 4
        if (((rb_buffer[rb_write+1] & 0x80) == 0x00) && ((rb_buffer[rb_write+2] & 0x80) == 0x00)) {ledVals[3]=I2C_LED_ON;} else {ledVals[3]=I2C_LED_OFF;}
        // channel 5
        if (((rb_buffer[rb_write] & 0x01) == 0x00) && ((rb_buffer[rb_write+1] & 0x01) == 0x00)) {ledVals[4]=I2C_LED_ON;} else {ledVals[4]=I2C_LED_OFF;}
        // channel 6
        if (((rb_buffer[rb_write] & 0x02) == 0x00) && ((rb_buffer[rb_write+1] & 0x02) == 0x00)) {ledVals[5]=I2C_LED_ON;} else {ledVals[5]=I2C_LED_OFF;}
        // channel 7
        if (((rb_buffer[rb_write] & 0x04) == 0x00) && ((rb_buffer[rb_write+1] & 0x04) == 0x00)) {ledVals[6]=I2C_LED_ON;} else {ledVals[6]=I2C_LED_OFF;}
        // channel 8
        if (((rb_buffer[rb_write] & 0x08) == 0x00) && ((rb_buffer[rb_write+1] & 0x08) == 0x00)) {ledVals[7]=I2C_LED_ON;} else {ledVals[7]=I2C_LED_OFF;}
    }
    //runtimeData.current = 0x00123456;
    //temp = runtimeData.current & 0xFF; 
    rb_buffer[rb_write+2] = runtimeData.current & 0xFF;
    //debug
    //rb_buffer[rb_write+17] = temp;
    //temp = (runtimeData.current >> 8) & 0xFF;
    rb_buffer[rb_write+1] = (runtimeData.current >> 8) & 0xFF;
    //debug
    //rb_buffer[rb_write+16] = temp;
    //temp = (runtimeData.current >> 16) & 0xFF;
    rb_buffer[rb_write+0] = 0x00;
    //debug
    //rb_buffer[rb_write+15] = temp;
    
    
    // rb_buffer writing pointer handler.
	rb_write+=SAMPLE_LENGTH;
	if (rb_write==RB_LENGTH)		// adjust write pointer	
    {
//        Nop();
        rb_write = 0;
//        overflow_cnt++;
//        if (overflow_cnt>10) {
//                Nop();
//        }
    }
	rb_size += SAMPLE_LENGTH;		// adjust available bytes

    //APP_SPI_CS_DESELECT();

	DCH0INT &= 0xFFFFFF00;			// clear DMA0 event flags
}
/*******************************************************************
 * Interrupt service routine: dma channel 1 (send)                 *
 *******************************************************************/
void DMA1CallBack(void)
{
    SYS_DMA_Channel1Disable();
	DCH1INT &= 0xFFFFFF00;			// clear DMA1 event flags
}
   