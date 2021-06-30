#include "app.h"

DEVICE_DATA deviceAppData;
I2C_STATES i2cAppState;

unsigned char deviceCode;
unsigned int deviceID;
unsigned char deviceIdentification[20];	


int device_Initialize ( void )
{

    return 1;
}

void deviceService ( void )
{
    switch(deviceAppData.state)
    {
        case DEVICE_STATE_INIT:
        {
            
            break;
        }
        case DEVICE_STATE_ID_READ_COMPLETE:
        {
            deviceAppData.state = DEVICE_STATE_EEPROM_ID_READ;
            break;
        }
        case DEVICE_STATE_EEPROM_ID_READ:
        {
            eeprom_reg_address = 0xFA;
            max_len_of_readEEPromBytes = 6;
            i2cAppState = I2C_READ_EEPROM;
            deviceAppData.state = DEVICE_STATE_EEPROM_ID_READ_WAIT;
            break;
        }        
        case DEVICE_STATE_EEPROM_ID_READ_WAIT:
        {
            if (eeprom_read_complete==2) {
                deviceAppData.state = DEVICE_STATE_EEPROM_ID_READ_COMPLETE;
                i2c_LED_Initialize();

            }
            break;
        }
        case DEVICE_STATE_EEPROM_ID_READ_COMPLETE:
        {
            if (rxBuffer_i2c[0]==0x00 && rxBuffer_i2c[1]==0x04 && rxBuffer_i2c[2]==0xA3 || rxBuffer_i2c[0]==0xd8 && rxBuffer_i2c[1]==0x80 && rxBuffer_i2c[2]==0x39) {//if its the EUI-48 EEPROM Adress
                for (i=0; i<6;i++) {
                    deviceIdentification[i] = rxBuffer_i2c[i];
                }
                deviceIdentification[6] = usbEMG; //usbEMG 0x01
                deviceIdentification[7]=deviceCode;
                for (i=0;i<12;i++) {
                    if (i<4)
                        deviceIdentification[i+8] = TSTP_HW_usbEMGv2_3[i];
                    else if (i<6)
                        deviceIdentification[i+8] = HW_Version_usbEMG[i-4];
                        else if (i<10)
                            deviceIdentification[i+8] = TSTP_SW_usbEMGv2_3[i-6];
                            else if (i<12)
                                deviceIdentification[i+8] = SW_Version_usbEMG[i-10];
                }            
            }
            deviceAppData.state = DEVICE_STATE_MAIN_TASK;
            break;
        }
        case DEVICE_STATE_MAIN_TASK:
        {
            break;
        }
        default:
            break;
    }
}