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
#ifndef _I2C_STATE_H
#define	_I2C_STATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

#define EEPROM_WRITE_ADDRESS  0b10100000 /* EEPROM Write Opcode */
#define EEPROM_READ_ADDRESS   0b10100001  /* EEPROM Read Opcode */

#define PCA9634_ADDRESS         0x82

#define PCA9634_REG_MODE1		0x00		// Mode register 1 
#define PCA9634_REG_MODE2		0x01		// Mode register 2
#define PCA9634_REG_PWM0		0x02		// brightness control LED0
#define PCA9634_REG_PWM1		0x03		// brightness control LED1
#define PCA9634_REG_PWM2		0x04		// brightness control LED2
#define PCA9634_REG_PWM3		0x05		// brightness control LED3
#define PCA9634_REG_PWM4		0x06		// brightness control LED4
#define PCA9634_REG_PWM5		0x07		// brightness control LED5
#define PCA9634_REG_PWM6		0x08		// brightness control LED6
#define PCA9634_REG_PWM7		0x09		// brightness control LED7
#define PCA9634_REG_GRPPWM		0x0A		// group duty cycle
#define PCA9634_REG_GRPFREQ		0x0B		// group frequency
#define PCA9634_REG_LEDOUT0		0x0C		// LED output state 0
#define PCA9634_REG_LEDOUT1		0x0D		// LED output state 1
#define PCA9634_REG_SUBADR1		0x0E		// I2C-bus subaddress 1
#define PCA9634_REG_SUBADR2		0x0F		// I2C-bus subaddress 2
#define PCA9634_REG_SUBADR3		0x10		// I2C-bus subaddress 3
#define PCA9634_REG_ALLCALLADR	0x11		// LED All I2C-bus address

#define I2C_LED1                0x02        // PCA9634 control register
#define I2C_LED2                0x03
#define I2C_LED3                0x04
#define I2C_LED4                0x05
#define I2C_LED5                0x06
#define I2C_LED6                0x07
#define I2C_LED7                0x08
#define I2C_LED8                0x09

#define I2C_LED_ON              0x25        // PWM value
#define I2C_LED_OFF             0x00        // PWM value



#define PCA9634_ALL_LED_TO_OFF				0x00
#define PCA9634_ALL_LED_TO_ON				0x55
#define PCA9634_ALL_LED_TO_PWM				0xAA
#define PCA9634_ALL_LED_TO_GRPPWM			0xFF

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation
int i2c_Initialize ( void );
char i2cTransmitArray(unsigned char, unsigned char, unsigned char *, unsigned char);
char i2cService(void);
void i2cTurnLED(unsigned char, unsigned char);

unsigned char eeprom_reg_address;
unsigned char max_len_of_readEEPromBytes;
unsigned char eeprom_data[1];
unsigned char eeprom_read_complete;
unsigned char rxBuffer_i2c[27];





typedef struct sI2CTransmission tI2CTransmission;

typedef enum
{
    /* Master initiates a stop condition */
    I2C_STATE_IDLE = 0,

    /* Master initiates a start condition */
    I2C_STATE_SEND_START_CONDITION,

    /* Master sends out an address byte */
    I2C_STATE_SEND_ADDRESS_BYTE,

    /* Master sends data to a slave */
    I2C_STATE_SEND_DATA,

    /* Master Send Start condition again to Read */
    I2C_SEND_PAGE_MARKER_START_CONDITON,

    /* Master Send Start Address */
    I2C_SEND_PAGE_MARKER_DEVICE_ADDRESS,

    /* Send Page address to mark for read */
    I2C_SEND_PAGE_ADDRESS,

    /* Master Send Start condition again to Read */
    I2C_READ_SEND_START_CONDITON,

    /*Master sends Read address byte to slave */
    I2C_STATE_SEND_READ_ADDRESS_BYTE,

   /*Master Reads Data from Slave*/
    I2C_STATE_READ_DATA,

   /* Stop condition for Read */
    I2C_STATE_READ_STOP_CONDITION,

    /* Master initiates a stop condition */
    I2C_STATE_SEND_STOP_CONDITION,
            
    I2C_READ_EEPROM_ID,        
            
    I2C_READ_EEPROM,
            
    I2C_WRITE_EEPROM,
            
    I2C_WRITE_LED,

    I2C_OPERATION_COMPLETE        

} I2C_STATES;

struct sI2CTransmission
{
    unsigned char bus_adr;
    unsigned char int_adr;
    unsigned char data_len;
    unsigned char * data;
    
    char pos;
};

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

