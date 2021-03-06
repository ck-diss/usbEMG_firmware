#ifndef _SPI_STATE_H
#define	_SPI_STATE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "C:\microchip\harmony\v1_06\framework\driver\spi\drv_spi.h"

#define PERIPHERAL_BUS_CLK   48000000
#define SPI_BAUD_RATE        125000

/// To allow the Chip Select Pin to be controlled by RC0 : 
#define SPI_CS_PORT_ID         PORT_CHANNEL_C
#define SPI_CS_PORT_PIN        PORTS_BIT_POS_0
 
#define APP_SPI_CS_SELECT() SYS_PORTS_PinClear(PORTS_ID_0,SPI_CS_PORT_ID,SPI_CS_PORT_PIN)
  
#define APP_SPI_CS_DESELECT() SYS_PORTS_PinSet(PORTS_ID_0,SPI_CS_PORT_ID,SPI_CS_PORT_PIN)

#define ADS1299_CS				LATCbits.LATC0
#define ADS1299_PWDN			LATCbits.LATC2
#define ADS1299_RESET			LATCbits.LATC1
#define ADS1299_START			LATAbits.LATA0
#define ADS1299_DRDY			PORTBbits.RB15

#define WAKEUP          0x02
#define STANDBY         0x04
#define RESET           0x06
#define START           0x08
#define STOP            0x0A
#define RDATAC          0x10
#define SDATAC          0x11
#define RDATA           0x12
#define RREG            0x20
#define WREG            0x40
#define RESET_SETTINGS	0xC6

#define ID			0x00
#define CONFIG1		0x01
#define CONFIG2		0x02
#define CONFIG3		0x03
#define LOFF		0x04
#define CH1SET		0x05
#define CH2SET		0x06
#define CH3SET		0x07
#define CH4SET		0x08
#define CH5SET		0x09
#define CH6SET		0x0A
#define CH7SET		0x0B
#define CH8SET		0x0C
#define RLD_SENSP	0x0D
#define RLD_SENSN	0x0E
#define LOFF_SENSP	0x0F
#define LOFF_SENSN	0x10
#define LOFF_FLIP	0x11
#define LOFF_STATP	0x12
#define LOFF_STATN	0x13
#define GPIO		0x14
#define MISC1		0x15
#define MISC2		0x16
#define CONFIG4		0x17
#define WCT1		0x18
#define WCT2		0x19
    
/// see http://microchip.wikidot.com/harmony:spi-dynamic
/* SPI Driver Handle */
DRV_HANDLE SPIHandle;
/* Write buffer handle */
DRV_SPI_BUFFER_HANDLE Write_Buffer_Handle;
/* Read buffer handle */
DRV_SPI_BUFFER_HANDLE Read_Buffer_Handle;
 
typedef unsigned char SPI_DATA_TYPE;

/* SPI Driver TX buffer */
SPI_DATA_TYPE txBuffer[6];
/* SPI Driver RX buffer */
SPI_DATA_TYPE rxBuffer[6];

typedef enum
{
    SPI_ENABLE,
            
    SPI_ADS_INIT,
            
    SPI_ADS_INIT_FROM_EEPROM,
            
    SPI_ADS_WAIT_FOR_EEPROM,
            
    SPI_START_WRITE_REG,
            
    SPI_WRITE_REG,

    SPI_WRITE_CMD,

    SPI_WAIT_FOR_REPLY,

    SPI_READ_ADS_ID,

    SPI_GET_DATA,
            
    SPI_WAIT_FOR_DATA,
            
    SPI_START_CONVERSION,
            
    SPI_STOP_CONVERSION,
            
    SPI_READ_EEPROM,
           
    SPI_COMPLETE,
            
    SPI_ERROR,

} SPI_STATES;

typedef struct
{
    SYS_MODULE_OBJ sysDevconObject;

    /* Current application state */
    SPI_STATES  state;
    
    SPI_STATES  stateBefore;

} SPI_DATA;


void GetEEPROMadsConfig (void);
void ADS1299Init (void);
void ADS_WriteREG (unsigned char, unsigned char);
void ADS_WriteREGsettings (unsigned char *);
void ADS_WriteCOM (unsigned char);
void ADS_ReadREG (unsigned char, unsigned char *, unsigned char);


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

