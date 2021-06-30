#include "app.h"

SPI_DATA spiAppData;
I2C_STATES i2cAppState;

unsigned char deviceCode = -1;

void GetEEPROMadsConfig(void) {
    eeprom_reg_address = 0x00;
    //max_len_of_readEEPromBytes = 24;
    max_len_of_readEEPromBytes = 27; //ADS Config + 3VsupplyByte
    i2cAppState = I2C_READ_EEPROM;
}

unsigned long CalcRuntimeInc (int samplingRate)
{
    float inc; ///??? 1/8000 = 0.000125
    inc = (1 / samplingRate) * (1000000);
    return (unsigned long) inc;
}


/***********************************************************************************
 * Function:        void ADS1299Init (void)
 *
 * Overview:        This function initializes the ADS1298 with fixed data.
 **********************************************************************************/
void ADS1299Init(void)
{
    ADS1299_START = 0;
    //delay_ms(6);                       // wait for stabilization of power supplys - at least 5.5ms
	ADS1299_RESET = 1;
	ADS1299_PWDN = 1;					// initial pin settings

	delay_ms(100);						// let the ADS start for sure
    
    ADS1299_RESET = 0;
	delay_us(1);
	ADS1299_RESET = 1;
	delay_us(10);						// reset the device

	ADS1299_PWDN = 1;					
	delay_ms(10);						// power up device

	ADS_WriteCOM(WAKEUP);
	delay_ms(10);						// wake the device up

	ADS_WriteCOM(SDATAC);
	delay_us(2);						// stop reading ads data continuously
    
    
   // rxBuffer_i2c[0] should contain 0xBB if it was already initialized with values
   // to Reset to initial Values set != 0xBB
   if (!USE_EEPROM_SAVED_CONFIG) {
        rxBuffer_i2c[0] = 0xEE; // !!! NO EEPROM READING !!!
   }
   if (rxBuffer_i2c[0] == 0xBB) {
        ADS_WriteREG(CONFIG1, rxBuffer_i2c[1]);		// High-Resolution - 8 kSPS
                                                // 7:1
                                                // 6: Daisy_en 0=enabled 
                                                // 5: CLK_EN 1=oscillator clock output
                                                // 4:3: must be '10'
                                                // 2:0 data rate 001=8kSPS
        ADS_WriteREG(CONFIG2, rxBuffer_i2c[2]);		
                                            // 7:5: must be '110'
                                            // 4: test source 0=extern
                                            // 3: must be '0'
                                            // 2: test signal amp -> 0 2.4mV (default)
                                            // 1:0 testsign. frequency '00' fclk/2^21 (default)
        ADS_WriteREG(CONFIG3, rxBuffer_i2c[3]);	// 	7: PD_REFBUF ..... 0 ... power down internal reference buffer
        //						76543210       	6:5 always 1
        //										4: BIAS_MEAS ...... 1 ... Bias_in is routed for measurement
        //										3: BIASREF_INT .... 0 ... reference fed externally
        //										2: BIAS buffer power ........ 0 ... power down RLD buffer
        //										1: BIAS_LOFF_SENS . 0 ... RLD sense function is disabled
        //										0: BIAS_STAT ...... 0 ... RLD lead off status - connected
        delay_us(9);
        ADS_WriteREG(LOFF, rxBuffer_i2c[4]);			// disable lead off

        ADS_WriteREG(CH1SET, rxBuffer_i2c[5]);				// 	7: PD ..... 0 ... power down normal operation
        //                 0b00010000
        //					 76543210       	6:4 gain -> set to 2 
        //										3: Source, reference bias channel .... 0 ... open default
        //										2:0 MUXn[2:0]: Channel input........ 000 ... normal electrod input
        ADS_WriteREG(CH2SET, rxBuffer_i2c[6]);			// gain 2
        ADS_WriteREG(CH3SET, rxBuffer_i2c[7]);			// gain 2
        ADS_WriteREG(CH4SET, rxBuffer_i2c[8]);			// gain 2
        ADS_WriteREG(CH5SET, rxBuffer_i2c[9]);			// gain 2
        ADS_WriteREG(CH6SET, rxBuffer_i2c[10]);			// gain 2
        ADS_WriteREG(CH7SET, rxBuffer_i2c[11]);			// gain 2
        ADS_WriteREG(CH8SET, rxBuffer_i2c[12]);			// gain 2

        ADS_WriteREG(RLD_SENSP, rxBuffer_i2c[13]);		// disable all channels
        ADS_WriteREG(RLD_SENSN, rxBuffer_i2c[14]);		// disable all channels
        ADS_WriteREG(LOFF_SENSP, rxBuffer_i2c[15]);		// disable all channels
        ADS_WriteREG(LOFF_SENSN, rxBuffer_i2c[16]);		// disable all channels
        ADS_WriteREG(LOFF_FLIP, rxBuffer_i2c[17]);

        ADS_WriteREG(GPIO, rxBuffer_i2c[20]);
        ADS_WriteREG(MISC1, rxBuffer_i2c[21]);
        ADS_WriteREG(MISC2, rxBuffer_i2c[22]);
        delay_us(9);
        ADS_WriteREG(CONFIG4, rxBuffer_i2c[23]);		//
        //ADS_WriteREG(WCT1, 0x00); 
        //ADS_WriteREG(WCT2, 0x00);	

        //ADS_ReadREG(CONFIG1,&RXbuffer[0],1);
    } else {
        if (DEVICE_IS_ADS_1299) {
            ADS_WriteREG(CONFIG1, 0b10110001);		// High-Resolution - 8 kSPS (001) 250SPS(110) 4k (010)
                                                // 7:1
                                                // 6: Daisy_en 0=enabled 
                                                // 5: CLK_EN 1=oscillator clock output
                                                // 4:3: must be '10'
                                                // 2:0 data rate 001=8kSPS
            ADS_WriteREG(CONFIG2, 0b11000000);		
                                                // 7:5: must be '110'
                                                // 4: test source 0=extern
                                                // 3: must be '0'
                                                // 2: test signal amp -> 0 2.4mV (default)
                                                // 1:0 testsign. frequency '00' fclk/2^21 (default)
            ADS_WriteREG(CONFIG3, 0b11101100);	// 	7: PD_REFBUF ..... 1 ... power on internal reference buffer
            //						76543210       	6: reserved = 1
            //                                      5: always 1
            //										4: BIAS_MEAS ...... 0 ... Bias_in is NOT routed for measurement (open)
            //										3: BIASREF_INT .... 1 ... reference fed internally
            //										2: BIAS buffer power ........ 1 ... power on RLD buffer
            //										1: BIAS_LOFF_SENS . 0 ... RLD sense function is disabled
            //										0: BIAS_STAT ...... 0 ... RLD lead off status - connected
           // delay_us(9);
            ADS_WriteREG(LOFF, 0x00);			//  lead off settings DC - default
            //ADS_WriteREG(LOFF, 0x03);			//  AC, 6nA, Fdr/4
            ADS_WriteREG(CH1SET, 0x30);				// 	7: PD ..... 0 ... normal operation
            //                   0b00110000
            //					   76543210       	6:4 gain -> set to 6 
            //										3: SRB2 Source, reference bias channel .... 0 ... open (off) default // 1 -> electrode chosen as reference
            //										2:0 MUXn[2:0]: Channel input........ 000 ... normal electrod input // 010 bias measurement
            ADS_WriteREG(CH2SET, 0x30);			// gain 2 00010000 //gain 6 00110000
            ADS_WriteREG(CH3SET, 0x30);			// gain 
            ADS_WriteREG(CH4SET, 0x30);			// gain 
            ADS_WriteREG(CH5SET, 0x30);			// gain 
            ADS_WriteREG(CH6SET, 0x30);			// gain 
            ADS_WriteREG(CH7SET, 0x30);			// gain 
            ADS_WriteREG(CH8SET, 0x30);			// gain 
            delay_us(9);
            ADS_WriteREG(RLD_SENSP, 0x00);		// disable all channels //1 = channel 1
            ADS_WriteREG(RLD_SENSN, 0x00);		// disable all channels
            ADS_WriteREG(LOFF_SENSP, 0xff);		// ff enable // disable all channels
            ADS_WriteREG(LOFF_SENSN, 0xff);		// ff enable // disable all channels
            ADS_WriteREG(LOFF_FLIP, 0x00);

            ADS_WriteREG(GPIO, 0x0F);           // 0F = default
            ADS_WriteREG(MISC1, 0x00);          // Bit5: SRB1: 0 = open (default) // SRB1 closed 1
            ADS_WriteREG(MISC2, 0x00);          // always 0
            delay_us(9);
            ADS_WriteREG(CONFIG4, 0x02);		// 02 turn on lead off // 00 continous conversion, lead-off comperator disabled (default)
        //}
        } else { // DEVICE IS ADS1298
            ADS_WriteREG(CONFIG1, 0b10100010);		// High-Resolution - 8 kSPS (001) 250SPS(110) 4k (010)
                                                // 7: HR 1= High Resolution
                                                // 6: Daisy_en 0=enabled 
                                                // 5: CLK_EN 1=oscillator clock output
                                                // 4:3: must be '00'
                                                // 2:0 data rate 010=8kSPS
             ADS_WriteREG(CONFIG2, 0b00000000);		
                                                // 7:6: must be '00'
                                                // 6: WCT chopping scheme 0 = frequency varies
                                                // 4: test source 0=extern
                                                // 3: must be '0'
                                                // 2: test signal amp -> 0 2.4mV (default)
                                                // 1:0 testsign. frequency '00' fclk/2^21 (default)
              ADS_WriteREG(CONFIG3, 0b11001100);	// 	7: PD_REFBUF ..... 1 ... power internal reference buffer
            //						  76543210      6: reserved = 1
            //                                      5: Ref Voltage P 0 = 2.4V
            //										4: BIAS_MEAS ...... 0 ... Bias_in is NOT routed for measurement
            //										3: BIASREF_INT .... 1 ... reference fed internally
            //										2: BIAS buffer power ........ 1 ... power on RLD buffer
            //										1: BIAS_LOFF_SENS . 0 ... RLD sense function is disabled
            //										0: BIAS_STAT ...... 0 ... RLD lead off status - connected
            delay_us(9);
            ADS_WriteREG(LOFF, 0x00);			// default lead off - DC
            ADS_WriteREG(CH1SET, 0x00);				// 	7: PD ..... 0 ... normal operation
            //                   0b00000000
            //					   76543210       	6:4 gain -> set to 6 
            //										3: Source, reference bias channel .... 0 ... open (off) default
            //										2:0 MUXn[2:0]: Channel input........ 000 ... normal electrod input
            ADS_WriteREG(CH2SET, 0x00);			// gain 2 00010000 //gain 6 00110000
            ADS_WriteREG(CH3SET, 0x00);			// gain 
            ADS_WriteREG(CH4SET, 0x00);			// gain 
            ADS_WriteREG(CH5SET, 0x00);			// gain 
            ADS_WriteREG(CH6SET, 0x00);			// gain 
            ADS_WriteREG(CH7SET, 0x00);			// gain 
            ADS_WriteREG(CH8SET, 0x00);			// gain 
            delay_us(9);
            ADS_WriteREG(RLD_SENSP, 0x00);		// disable all channels
            ADS_WriteREG(RLD_SENSN, 0x00);		// disable all channels
            ADS_WriteREG(LOFF_SENSP, 0x00);		// disable all channels
            ADS_WriteREG(LOFF_SENSN, 0x00);		// disable all channels
            ADS_WriteREG(LOFF_FLIP, 0x00);

            ADS_WriteREG(GPIO, 0x0F);           // 0F = default
            ADS_WriteREG(MISC1, 0x00);          // SRB1: 0 = open (default)
            ADS_WriteREG(MISC2, 0x00);          // 00 default
            delay_us(9);
            ADS_WriteREG(CONFIG4, 0x00);		// continous conversion, lead-off comperator disabled (default)            
            ADS_WriteREG(WCT1, 0x00); 
            ADS_WriteREG(WCT2, 0x00);	
        }
        //write 0xBB to eeprom at address 0x00 to know, the ADS config is written successfully to EEPROM
        eeprom_reg_address = 0x00;
        eeprom_data[0]=0xBB; 
        i2cAppState = I2C_WRITE_EEPROM;
        while (i2cAppState != I2C_STATE_IDLE && i2cAppState != I2C_OPERATION_COMPLETE){
            i2cService();
        }
        //ADS_ReadREG(CONFIG1,&RXbuffer[0],1);
    
    }
	delay_ms(10);

}	
/***********************************************************************************
 * Function:	void ADS_ReadREG (unsigned char reg, unsigned char *data, unsigned char len)
 *
 * Overview:	This function reads one specific register of the ADS1299.
 **********************************************************************************/
void ADS_ReadREG (unsigned char reg, unsigned char *data, unsigned char len)
{
	unsigned char temp [2];
	temp [0] = RREG | (reg & 0x1F);		// write register op-code
	temp [1] = (len-1) & 0x1F;			// register to read - 1
    
	ADS_WriteCOM(SDATAC);				// issue SDATAC command before reading registers
	delay_us(2);

	APP_SPI_CS_SELECT();
	delay_us(1);

    DRV_SPI0_BufferAddWriteRead(&temp, &data[0], 2);
    
    delay_us(1);
 
    DRV_SPI0_BufferAddWriteRead(&temp, &data[0], len);
    
    if (reg==ID) {
        deviceCode = data[0];
    }
	delay_us(2);
	APP_SPI_CS_DESELECT();
	delay_us(1);
}
/***********************************************************************************
 * Function:       void ADS_WriteREG (unsigned char reg, unsigned char data)
 *
 * Overview:        This function writes one specific register to the ADS1298.
 **********************************************************************************/
void ADS_WriteREG (unsigned char reg, unsigned char rxBuffer)
{
	unsigned char temp [3];
	temp [0] = WREG | (reg & 0x1F);		// write register op-code
	temp [1] = 0x00;					// write one register
	temp [2] = rxBuffer;

	ADS_WriteCOM(SDATAC);				// issue SDATAC command before reading registers
	delay_us(2);

	delay_us(2);
	APP_SPI_CS_SELECT();
	delay_us(1);

    DRV_SPI0_BufferAddWriteRead(&temp[0], &rxBuffer, 3);
    
    delay_us(2);
	APP_SPI_CS_DESELECT();
	delay_us(1);
    
    eeprom_reg_address = reg;
    eeprom_data[0]=temp[2];
    i2cAppState = I2C_WRITE_EEPROM;
    while (i2cAppState != I2C_STATE_IDLE && i2cAppState != I2C_OPERATION_COMPLETE){
       i2cService();
    }
}	

/***********************************************************************************
 * Function:		void ADS_WriteREGsettings (unsigned char data)
 *
 * Overview:		This function writes the whole register setting on the ADS1298.
 **********************************************************************************/
void ADS_WriteREGsettings (unsigned char *data)
{
	unsigned char i, j;
	unsigned char temp [27];

	// we need to skip the read only registers
	// first part starts at CONFIG1 and ends with LOFF_FLIP 
	temp [0] = WREG | (CONFIG1 & 0x1F);		// write register op-code
	temp [1] = (LOFF_FLIP-ID-1) & 0x1F;		// register to write - 1

	for (i=2, j=0; j<LOFF_FLIP; i++, j++)
		temp[i] = data[j];

    
    ADS_WriteCOM(SDATAC);				// issue SDATAC command before reading registers
	delay_us(2);

	APP_SPI_CS_SELECT();
	delay_us(1);

    DRV_SPI0_BufferAddWriteRead(&temp[0], &rxBuffer, i);


	delay_us(2);
	APP_SPI_CS_DESELECT();
	delay_us(1);


	// second part starts at GPIO and ends with WCT2
	temp [0] = WREG | (GPIO & 0x1F);		// write register op-code
	temp [1] = (CONFIG4-LOFF_STATN-1) & 0x1F;	// register to write - 1

	for (i=2, j=19; j<CONFIG4; i++, j++)
		temp[i] = data[j];

	delay_us(2);
	APP_SPI_CS_SELECT();
	delay_us(1);

    DRV_SPI0_BufferAddWriteRead(&temp[0], &rxBuffer, i);

	delay_us(2);
	APP_SPI_CS_DESELECT();
	delay_us(1);
}
/***********************************************************************************
 * Function:       void ADS_WriteCOM (unsigned char data)
 *
 * Overview:       This function writes a command to the ADS1298.
 **********************************************************************************/
void ADS_WriteCOM (unsigned char data)
{
	unsigned char temp;
	temp = data;
	delay_us(2);
	APP_SPI_CS_SELECT();
	delay_us(1);

    DRV_SPI0_BufferAddWriteRead(&temp, &rxBuffer, 1);

	delay_us(2);
	APP_SPI_CS_DESELECT();
	delay_us(1);	
}

/*******************************************************************
 * Interrupt service routine: external Interrupt - ADS Data Ready  *
 *******************************************************************/
void DRDYCallBack () {
    
    APP_SPI_CS_SELECT();

	samples_cnt++;
    DCH0DSA = (volatile unsigned long)&rb_buffer[rb_write] & 0x1FFFFFFF;	// destination address to receive

    SYS_DMA_Channel0Enable();
    SYS_DMA_Channel1Enable();

    SYS_DMA_Channel1ForceStart();
}