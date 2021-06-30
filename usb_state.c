/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "usb_state.h"
#include "app.h"
#include <stdbool.h>

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */
/** Descriptive Data Item Name

  @Summary
    Brief one-line summary of the data item.
    
  @Description
    Full description, explaining the purpose and usage of data item.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
  @Remarks
    Any additional remarks
 */
extern SPI_DATA spiAppData;
extern APP_DATA appData;
USB_DATA usbAppData;
I2C_STATES i2cAppState;

extern unsigned int rb_size;
extern unsigned char deviceIdentification[];

/* Receive data buffer */
uint8_t receivedDataBuffer[USB_READ_BUFFER_SIZE] USB_MAKE_BUFFER_DMA_READY;

/* Transmit data buffer */
uint8_t  transmitDataBuffer[USB_READ_BUFFER_SIZE] USB_MAKE_BUFFER_DMA_READY;

/* The endpoint size is 64 for FS and 512 for HS */
uint16_t endpointSize;

/* Extended Compatible ID Feature Descriptor */ 
const uint8_t microsoftOsCompatDescriptor[] = 
{
    0x28, 0x00, 0x00, 0x00, /* dwLength Length of this descriptor */ 
    0x00, 0x01,             /* bcdVersion = Version 1.0 */ 
    0x04, 0x00,             /* wIndex = 0x0004 */ 
    0x01,                   /* bCount = 1 */ 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Reserved */ 
    0x00,                   /* Interface number = 0 */ 
    0x01,                   /* Reserved */ 
    0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, /* compatibleID */ 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  /* subCompatibleID */ 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* Reserved */   
};

///* Extended Properties OS Feature Descriptor */
//const uint8_t microsoftOsPropDescriptor[] = 
//{
////     0x92, 0x00, 0x00, 0x00, /* dwLength Length of this descriptor */ 
////    0x00, 0x01,             /* bcdVersion = Version 1.0 */ 
////    0x05, 0x00,             /* wIndex = 0x0005 */ 
////    0x01, 0x00,                   /* bCount = 1 */
////    0x88, 0x00, 0x00, 0x00, /* Length of custom property section (132 bytes) */
////    0x07, 0x00, 0x00, 0x00, /*String format (UTF-16LE Unicode) */
////    0x2A, 0x00,             /* Length of Property name (40 bytes) */
////    /* Property name: DeviceInterfaceGUID */
////    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00,
////    0x49, 0x00, 0x6E, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00,
////    0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00, 0x55, 0x00, 0x49, 0x00,
////    0x44, 0x00, 0x73, 0x00, 0x00, 0x00,
////    
////    0x50, 0x00, 0x00, 0x00, /*Length of property data (78 bytes)*/
////    
////    /* Vendor-defined property data: {1dfd1ca0-1e3e-4ece-beef-95e7c7b49cac} */
////    0x7B, 0x00, 0x31, 0x00, 0x64, 0x00, 0x66, 0x00, 0x64, 0x00, 0x31, 0x00, 
////    0x63, 0x00, 0x61, 0x00, 0x30, 0x00, 0x2D, 0x00, 0x31, 0x00, 0x65, 0x00, 
////    0x33, 0x00, 0x65, 0x00, 0x2d, 0x00, 0x34, 0x00, 0x65, 0x00, 0x63, 0x00, 
////    0x65, 0x00, 0x2d, 0x00, 0x62, 0x00, 0x65, 0x00, 0x65, 0x00, 0x66, 0x00, 
////    0x2d, 0x00, 0x39, 0x00, 0x35, 0x00, 0x65, 0x00, 0x37, 0x00, 0x63, 0x00, 
////    0x37, 0x00, 0x62, 0x00, 0x34, 0x00, 0x39, 0x00, 0x63, 0x00, 0x61, 0x00, 
////    0x63, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00
//    
//        sizeof(ExtPropertyFeatureDescriptor),   //dwLength
//        0x0100,                                 //bcdVersion = 1.00
//        EXTENDED_PROPERTIES,                    //wIndex
//        0x0001,                                 //wCount - 0x0001 "Property Sections" implemented in this descriptor
//        //----------Property Section 1----------
//        132,                                    //dwSize - 132 bytes in this Property Section
//        0x00000001,                             //dwPropertyDataType (Unicode string)
//        40,                                     //wPropertyNameLength - 40 bytes in the bPropertyName field
//        {'D','e','v','i','c','e','I','n','t','e','r','f','a','c','e','G','U','I','D', 0x0000},  //bPropertyName - "DeviceInterfaceGUID"
//        78,                                     //dwPropertyDataLength - 78 bytes in the bPropertyData field (GUID value in UNICODE formatted string, with braces and dashes)
//        //The below value is the Device Interface GUID (a 128-bit long "globally unique identifier")
//        //Please modify the GUID value in your application before moving to production.
//        //When you change the GUID, you must also change the PC application software
//        //that connects to this device, as the software looks for the device based on 
//        //VID, PID, and GUID.  All three values in the PC application must match 
//        //the values in this firmware.
//        //The GUID value can be a randomly generated 128-bit hexadecimal number, 
//        //formatted like the below value.  The actual value is not important,
//        //so long as it is almost certain to be globally unique, and matches the
//        //PC software that communicates with this firmware.
//        {'{','5','8','d','0','7','2','1','0','-','2','7','c','1','-','1','1','d','d','-','b','d','0','b','-','0','8','0','0','2','0','0','c','9','a','6','6','}',0x0000} 

#define USB_ISOLATOR_ACTIVATE   LATAbits.LATA10   


// LED parameters
//#define LED_GREEN   LATBbits.LATB3      // for device status
//#define LED_ORANGE  LATAbits.LATA9      // for ADS-1299 Information
//#define LED_ON      0
//#define LED_OFF     1

#define USB_COMMAND_ENDPOINT_RX 0x01
#define USB_COMMAND_ENDPOINT_TX 0x81
#define USB_DATA_ENDPOINT_TX    0x82


bool flgDatasend = false;

// CRC16-CCIT parameters 
typedef uint16_t        ushort; 
typedef unsigned char  Byte; 
 
ushort CheckSum = 0xFFFF; 
int CRC_Packet_length; 
bool flgPacketCheckResult = false; 

char temp3VsupplyValue = -1;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/*********************************************
 * Application USB Device Layer Event Handler
 *********************************************/

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    uint8_t * configurationValue;
    uint16_t length = 0; 
    USB_SETUP_PACKET * setupPacket;
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device is reset or de-configured. Provide LED indication.*/
            usbAppData.deviceIsConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration */
            configurationValue = (uint8_t *)eventData;
            if(*configurationValue == 1 )
            {
                /* The device is in configured state. Update LED indication */
                /* Reset endpoint data send & receive flag  */
                usbAppData.deviceIsConfigured = true;
            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Device is suspended. Update LED indication */
            //LEDOn(LED_status, PORT_A);
            //LEDOn(LED_power, PORT_B);
            break;


        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS is detected. Attach the device */
            USB_ISOLATOR_ACTIVATE = 1;  //activate USB isolator with pin RA10
            USB_DEVICE_Attach(usbAppData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is removed. Detach the device */
            USB_DEVICE_Detach (usbAppData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
            /* This means we have received a setup packet */
            setupPacket = (USB_SETUP_PACKET *)eventData;
            if(setupPacket->bRequest == USB_REQUEST_SET_INTERFACE)
            {
                /* If we have got the SET_INTERFACE request, we just acknowledge
                 for now. This demo has only one alternate setting which is already
                 active. */
                USB_DEVICE_ControlStatus(usbAppData.usbDevHandle,USB_DEVICE_CONTROL_STATUS_OK);
            }
            else if(setupPacket->bRequest == USB_REQUEST_GET_INTERFACE)
            {
                /* We have only one alternate setting and this setting 0. So
                 * we send this information to the host. */
                USB_DEVICE_ControlSend(usbAppData.usbDevHandle, &usbAppData.altSetting, 1);
            }
            // Sending the Microsoft OS Extended Descriptor if the OS asked for
            else if (setupPacket->bRequest == APP_USB_MICROSOFT_OS_DESCRIPTOR_VENDOR_CODE)
            {
                if (setupPacket->bmRequestType == 0xc0)
                {
                    length = setupPacket->wLength; 
                    if (length > sizeof(microsoftOsCompatDescriptor))
                    {
                       length =  sizeof(microsoftOsCompatDescriptor); 
                    }
                    USB_DEVICE_ControlSend(usbAppData.usbDevHandle, &microsoftOsCompatDescriptor, length);
                }
                
//                else if (setupPacket->bmRequestType == 0xc1)
//                {
//                    length = setupPacket->wLength; 
//                    if (length > sizeof(microsoftOsPropDescriptor))
//                    {
//                       length =  sizeof(microsoftOsPropDescriptor); 
//                    }
//
//                    USB_DEVICE_ControlSend(usbAppData.usbDevHandle, &microsoftOsPropDescriptor, length);
//                }  
            }
            else
            {
                /* We have received a request that we cannot handle. Stall it*/
                USB_DEVICE_ControlStatus(usbAppData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            }
            break;

        case USB_DEVICE_EVENT_ENDPOINT_READ_COMPLETE:
           /* Endpoint read is complete */
            usbAppData.epDataReadPending = false;
            break;

        case USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE:
            /* Endpoint write is complete */
            usbAppData.epDataWritePending = false;
            break;

        /* These events are not used in this demo. */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Application Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

void USB_allEndpointEnable()
{
    if (USB_DEVICE_EndpointIsEnabled(usbAppData.usbDevHandle, usbAppData.endpointRx) == false )
    {
        /* Enable Read EP1Rx Command Endpoint */
        USB_DEVICE_EndpointEnable(usbAppData.usbDevHandle, 0, usbAppData.endpointRx,
                USB_TRANSFER_TYPE_BULK, endpointSize);
    }
    if (USB_DEVICE_EndpointIsEnabled(usbAppData.usbDevHandle, usbAppData.endpointTx) == false )
    {
        /* Enable Write EP1Tx Command Endpoint */
        USB_DEVICE_EndpointEnable(usbAppData.usbDevHandle, 0, usbAppData.endpointTx,
                USB_TRANSFER_TYPE_BULK, endpointSize);
    }

    if (USB_DEVICE_EndpointIsEnabled(usbAppData.usbDevHandle, usbAppData.dataendpointTx) == false )
    {
        /* Enable Write EP2Tx Data Endpoint */
        USB_DEVICE_EndpointEnable(usbAppData.usbDevHandle, 0, usbAppData.dataendpointTx,
                USB_TRANSFER_TYPE_BULK, endpointSize);
    }
}

// Endpoints disable
void USB_allEndpointDisable()
{
    // Disable all endpoints, EP1Rx, EP1Tx, EP2Tx respectively.
    USB_DEVICE_EndpointDisable(usbAppData.usbDevHandle, usbAppData.endpointRx);
    USB_DEVICE_EndpointDisable(usbAppData.usbDevHandle, usbAppData.endpointTx);
    USB_DEVICE_EndpointDisable(usbAppData.usbDevHandle, usbAppData.dataendpointTx);
}

void USB_EndpointDisable(USB_ENDPOINT_ADDRESS endpoint)
{
    USB_DEVICE_EndpointDisable(usbAppData.usbDevHandle, endpoint);
}

void USB_EndpointEnable(USB_ENDPOINT_ADDRESS endpoint)
{
    if (USB_DEVICE_EndpointIsEnabled(usbAppData.usbDevHandle, endpoint) == false )
    {
        /* Enable Read EP1Rx Command Endpoint */
        USB_DEVICE_EndpointEnable(usbAppData.usbDevHandle, 0, endpoint,
                USB_TRANSFER_TYPE_BULK, endpointSize);
    }  
}

void USB_EndpointRestart(USB_ENDPOINT_ADDRESS endpoint)
{
   USB_EndpointDisable(endpoint);
   USB_EndpointEnable(endpoint);
}
// Sending Data stream through EP2Tx Data Endpoint
bool USBSendData(unsigned char* rb_buffer, unsigned int* rb_read)
{
    unsigned int i;
    
    for (i=0; i<USB_PACKET_SIZE; i++)
    {
        transmitDataBuffer[i] = rb_buffer[(*rb_read)++];
    }

    usbAppData.epDataWritePending = true;

    /*--------- Writing data to EP2Tx for USB transmission to host-----------*/
    volatile USB_DEVICE_RESULT r = USB_DEVICE_EndpointWrite ( usbAppData.usbDevHandle, &usbAppData.writeTranferHandle,
            usbAppData.dataendpointTx, &transmitDataBuffer[0],
            sizeof(transmitDataBuffer),
            USB_DEVICE_TRANSFER_FLAGS_MORE_DATA_PENDING);
    
    if(r!=USB_DEVICE_RESULT_OK)
        return false;
    else    
        return true;
    
}

// Sending Data stream through EP1Tx Data Endpoint
bool USBSendCommand(unsigned char* rx_buffer)
{
    unsigned int i;
    
    for (i=0; i<USB_PACKET_SIZE; i++)
    {
        transmitDataBuffer[i] = rx_buffer[i];
    }

    // 
    //usbAppData.epDataWritePending = true;

    /*--------- Writing command to EP1Tx for USB transmission to host-----------*/ 
    volatile USB_DEVICE_RESULT r = USB_DEVICE_EndpointWrite ( usbAppData.usbDevHandle, &usbAppData.writeTranferHandle,
            usbAppData.endpointTx, &transmitDataBuffer[0],
            sizeof(transmitDataBuffer),
            USB_DEVICE_TRANSFER_FLAGS_MORE_DATA_PENDING);
    
    if(r!=USB_DEVICE_RESULT_OK)
        return false;
    else    
        return true;
    
}

// Check if the packet is correctly received with CRC16-CCIT decoding 
// the result must be equal to 0x0000 (correct), otherwise the packet is incorrectly received 
// Check if CheckSum is correct 
bool USBcheckPacket(Byte* Packet) 
{ 
    bool result = false; 
    Byte Buffer[USB_PACKET_SIZE]; 
    int i; 
        //---------- CRC16-CCIT Decoding ----------// 
    // Setup a buffer for CRC16-CCIT checking 
    for (i=0; i<USB_PACKET_SIZE; ++i) 
    { 
        Buffer[i] = Packet[i]; 
    }   
 
    //Decoding with CRC16-CCIT 
    CheckSum = CRC16CheckSum((int)CRC_ADS_COMMAND_HEADER_LENGTH  + Buffer[6] + CRC16_BYTES_LENGTH, Buffer); 
    if (CheckSum == 0x0000) 
        result = CRC16_CORRECT; 
    else 
        result = CRC16_INCORRECT; 
     
    //return result; 
    return true;
} 
 
 
// Selecting which task to work on from a received command packet 
bool usbCommandSelection(uint8_t UsbCommand[])
{
    /* 
     * USB command unpacking 
     */ 
    // Check if packet is a Control command 
    switch(UsbCommand[0])
    {
        // Check if packet is a Control command
        case USB_COMMAND_PACKET:    // 0x55
        
            // Check command type
            switch(UsbCommand[5])
            {
                // ----- ADS1299 Commands ----- //
                case USB_ADS_PACKET :                           // 0xAD
                {
                    //Check ADS1299 Instructions 
                    switch(UsbCommand[7])
                    {
                        // ----- LED Command for communication testing purpose ----- // 
                        case USB_TEST_LED_ON: // 0x81 : Test USB communication LED turned ON 
                            //LEDOn(LED_power); 
                            return true; 
                            break; 
                        case USB_TEST_LED_OFF: // 0x82 : Test USB communication LED turned OFF 
                            //LEDOff(LED_power); 
                            return true; 
                            break; 
                        // ----- SPI System Commands ----- //
                        case WAKEUP:    // 0x02 : Wake up from Standby mode
                            ADS_WriteCOM(WAKEUP);
                            return true;
                            break;
                        case STANDBY:   // 0x04 : Enter standby mode
                            ADS_WriteCOM(STANDBY);
                            return true;
                            break;
                        case RESET:     // 0x06 : Reset the device
                            ADS_WriteCOM(RESET);
                            return true;
                            break;
                        case START:     // 0x08 : Start & restart (synchronize) conversion
                            // Hard wired to RA0, no function in this implementation
                            return true;
                            break;
                        case STOP:      // 0x0A : Stop conversion
                            // Hard wired to RA0, no function in this implementation
                            return true;
                            break;
                            
                        // ----- Data Read Commands ----- //
                        case RDATAC:    // 0x10 : Enable Read Data Continuous mode
                            spiAppData.state = SPI_START_CONVERSION;
                            return true;
                            break;
                        case SDATAC:    // 0x11 : Stop Read Data Continuous mode
                            spiAppData.state = SPI_STOP_CONVERSION;
                            return true;
                            break;
                        case RDATA:     // 0x12 : Read data by command
                            //Reading a single data is not needed, no function in this implementation
                            return true;
                            break;                            
                        // ----- Register Read Commands ----- //    
                        case RREG:      // 0x20 : RREG: Read n nnnn registers starting at addr r rrrr
                            // (UsbCommand[7] & 0xE0)
                            flushBuffer (&data[0], USB_PACKET_SIZE);
                            ADS_ReadREG(ID,&data[0],27);
                            //ADD 3V supply Byte 
                            if (temp3VsupplyValue != -1) {
                                data[26] = temp3VsupplyValue;
                            } else {
                                data[26] = rxBuffer_i2c[26];
                            }
                            
                            USBSendCommand(&data[0]);
                            return true;
                            break;
                        case WREG:      // 0x40 : Write n nnnn registers starting at addr r rrrr
                            //ADS_WriteREGsettings(&UsbCommand[9]);
                            ADS_WriteREG(UsbCommand[8],UsbCommand[9]);
                            return true;
                            break;
                        case RESET_SETTINGS:     // 0xC6 : Reset ADS settings to default
                            rxBuffer_i2c[0] = 0xC6; // Value != 0xBB resets to default
                            temp3VsupplyValue = 0x00;
                            ADS1299Init();
                            return true;
                            break;
                    }
                    //break;
                }
                
                // ----- MCU Commands ----- //
                case USB_MCU_INFO_PACKET:                       // 0x00
                    USBSendCommand(&deviceIdentification[0]);
                    return true;
                    break;
                case USB_MCU_GET_UNIX_TIMESTAMP_32_BIT_PACKET:  // 0xF0
                    return true;
                    break;
                case USB_MCU_SET_UNIX_TIMESTAMP_32_BIT_PACKET:  // 0xF1
                    return true;
                    break;
                case USB_MCU_GET_DAQ_TIMER_24_BIT_PACKET:       // 0xF2
                    return true;
                    break;
                case USB_MCU_SET_DAQ_TIMER_24_BIT_PACKET:       // 0xF3
                    return true;
                    break;
                case USB_MCU_GET_3V_SUPPLY_PACKET:              // 0xF4
//                    if (rxBuffer_i2c[0] == 0xBB) { //if eeprom is written correctly
//                        USBSendCommand(rxBuffer_i2c[24]);
//                    }
                    break;
                case USB_MCU_SET_3V_SUPPLY_PACKET:              // 0xF5
                {
                    //Check ADS1299 Instructions 
                    switch(UsbCommand[7])
                    {
                        case USB_MCU_SET_3V_SUPPLY_OFF:
                            CH1_CH2_3V_SUPPLY_OFF();  
                            CH3_CH4_3V_SUPPLY_OFF();  
                            CH5_CH6_3V_SUPPLY_OFF();  
                            CH7_CH8_3V_SUPPLY_OFF();  
                            break;
                        case 0x01: 
                            CH1_CH2_3V_SUPPLY_ON();  
                            CH3_CH4_3V_SUPPLY_OFF();  
                            CH5_CH6_3V_SUPPLY_OFF();  
                            CH7_CH8_3V_SUPPLY_OFF(); 
                            break;
                        case 0x02: 
                            CH1_CH2_3V_SUPPLY_OFF();  
                            CH3_CH4_3V_SUPPLY_ON();  
                            CH5_CH6_3V_SUPPLY_OFF();  
                            CH7_CH8_3V_SUPPLY_OFF(); 
                            break;
                        case 0x03: 
                            CH1_CH2_3V_SUPPLY_ON(); 
                            CH3_CH4_3V_SUPPLY_ON(); 
                            CH5_CH6_3V_SUPPLY_OFF();  
                            CH7_CH8_3V_SUPPLY_OFF(); 
                            break;
                        case 0x04: 
                            CH1_CH2_3V_SUPPLY_OFF();  
                            CH3_CH4_3V_SUPPLY_OFF();
                            CH5_CH6_3V_SUPPLY_ON(); 
                            CH7_CH8_3V_SUPPLY_OFF(); 
                            break;
                        case 0x05: 
                            CH1_CH2_3V_SUPPLY_ON();  
                            CH3_CH4_3V_SUPPLY_OFF();
                            CH5_CH6_3V_SUPPLY_ON(); 
                            CH7_CH8_3V_SUPPLY_OFF(); 
                            break;
                        case 0x06: 
                            CH1_CH2_3V_SUPPLY_OFF();  
                            CH3_CH4_3V_SUPPLY_ON();                            
                            CH5_CH6_3V_SUPPLY_ON();     
                            CH7_CH8_3V_SUPPLY_OFF(); 
                            break;
                        case 0x07: 
                            CH1_CH2_3V_SUPPLY_ON();                            
                            CH3_CH4_3V_SUPPLY_ON();                            
                            CH5_CH6_3V_SUPPLY_ON();  
                            CH7_CH8_3V_SUPPLY_OFF(); 
                            break;
                        case 0x08: 
                            CH1_CH2_3V_SUPPLY_OFF();
                            CH3_CH4_3V_SUPPLY_OFF();
                            CH5_CH6_3V_SUPPLY_OFF();
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                        case 0x09: 
                            CH1_CH2_3V_SUPPLY_ON();
                            CH3_CH4_3V_SUPPLY_OFF();
                            CH5_CH6_3V_SUPPLY_OFF();
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                        case 0x0A: 
                            CH1_CH2_3V_SUPPLY_OFF();
                            CH3_CH4_3V_SUPPLY_ON();
                            CH5_CH6_3V_SUPPLY_OFF();
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                        case 0x0B: 
                            CH1_CH2_3V_SUPPLY_ON();                            
                            CH3_CH4_3V_SUPPLY_ON();
                            CH5_CH6_3V_SUPPLY_OFF();
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                        case 0x0C: 
                            CH1_CH2_3V_SUPPLY_OFF();
                            CH3_CH4_3V_SUPPLY_OFF();
                            CH5_CH6_3V_SUPPLY_ON(); 
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                        case 0x0D: 
                            CH1_CH2_3V_SUPPLY_ON();  
                            CH3_CH4_3V_SUPPLY_OFF();
                            CH5_CH6_3V_SUPPLY_ON(); 
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                        case 0x0E: 
                            CH1_CH2_3V_SUPPLY_OFF();
                            CH3_CH4_3V_SUPPLY_ON();                            
                            CH5_CH6_3V_SUPPLY_ON(); 
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                        case 0x0F: 
                            CH1_CH2_3V_SUPPLY_ON();                            
                            CH3_CH4_3V_SUPPLY_ON();                            
                            CH5_CH6_3V_SUPPLY_ON(); 
                            CH7_CH8_3V_SUPPLY_ON();   
                            break;
                    }
                    eeprom_reg_address = 0x1A;
                    eeprom_data[0]=UsbCommand[7];
                    temp3VsupplyValue = UsbCommand[7];
                    i2cAppState = I2C_WRITE_EEPROM;
                    while (i2cAppState != I2C_STATE_IDLE && i2cAppState != I2C_OPERATION_COMPLETE){
                       i2cService();
                    }
                    break;
                }
                case USB_MCU_GET_STATUS_LED_PACKET:             // 0xF6
                    return true;
                    break;
                case USB_MCU_SET_STATUS_LED_PACKET:             // 0xF7
                    return true;
                    break;
                case USB_MCU_SEND_ERROR_PACKET :                // 0xEE
                    return true;
                    break;
                
                
                // ----- LED Driver Command ----- //
                case USB_LED_DRIVER_PACKET :                    // 0xD0
                    return true;
                    break;
                
                
                // ----- EEPROM Command ----- //
                case USB_EEPROM_PACKET :                        // 0xD1
                    return true;
                    break;
            }
            //break;   

        case USB_DEVICE_RESET:  // 0x80 : Device reset (All modules re-initialized)
            appData.state = APP_STATE_INIT;
            return true;
            break;                        

        default:
            return true;
            break;
    }    
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void usb_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
int usb_Initialize ( void )
{
    /* Set all initial parameters of USB module. */
    usbAppData.state = USB_STATE_INIT;
    usbAppData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    usbAppData.deviceIsConfigured = false;
    
    usbAppData.endpointRx = USB_COMMAND_ENDPOINT_RX;
    usbAppData.endpointTx = USB_COMMAND_ENDPOINT_TX;
    usbAppData.dataendpointTx = USB_DATA_ENDPOINT_TX;
    
    usbAppData.epDataReadPending = false;
    usbAppData.epDataWritePending = false;
    usbAppData.altSetting = 0;
    
    rb_usb_readout_overflow_cnt = 0;
   
    return 1;
}

/******************************************************************************
  Function:
    void usbService ( void )

  Remarks:
    See prototype in app.h.
 */

void usbService ( void )
{
    switch(usbAppData.state)
    {
        case USB_STATE_INIT:
            /* Open the device layer */
            usbAppData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(usbAppData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(usbAppData.usbDevHandle,  APP_USBDeviceEventHandler, 0);

                usbAppData.state = USB_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case USB_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured */
            if(usbAppData.deviceIsConfigured == true)
            {
                if (USB_DEVICE_ActiveSpeedGet(usbAppData.usbDevHandle) == USB_SPEED_FULL)
                {
                    endpointSize = 64;
                }
                else if (USB_DEVICE_ActiveSpeedGet(usbAppData.usbDevHandle) == USB_SPEED_HIGH)
                {
                    endpointSize = 512;
                }
                
                //--------------------- Endpoints Enable --------------------//
                USB_allEndpointEnable();
                
                /* Indicate that we are waiting for read */
                usbAppData.epDataReadPending = true;

                /* Place a new read request. */
                USB_DEVICE_EndpointRead(usbAppData.usbDevHandle, &usbAppData.readTranferHandle,
                        usbAppData.endpointRx, &receivedDataBuffer[0], sizeof(receivedDataBuffer) );

                /* Device is ready to run the main task */
                usbAppData.state = USB_STATE_MAIN_TASK;
            }
            break;

        case USB_STATE_MAIN_TASK:

            if(!usbAppData.deviceIsConfigured)
            {
                /* This means the device got de-configured. Change the
                 * application state back to waiting for configuration. */
                usbAppData.state = USB_STATE_WAIT_FOR_CONFIGURATION;

                /* Disable the endpoint*/
                USB_allEndpointDisable();
                usbAppData.epDataReadPending = false;
                usbAppData.epDataWritePending = false;
            }
            
            // Reading Command via EP1Rx via USB port, choose which task to be operated
            else if (usbAppData.epDataReadPending == false)
            {
                /* Look at the data sent from the host, to see what kind of
                 * application specific command it sent. */
 
                // Check the incoming command packet with CRC16-CCIT 
                flgPacketCheckResult = USBcheckPacket(receivedDataBuffer); 
                
                // Choose which task to do with usbCommandSelection()
                // if CRC16_CORRECT and a Command has been selected 
                if(flgPacketCheckResult && usbCommandSelection(receivedDataBuffer)) 
                { 
                    usbAppData.epDataReadPending = true ; 
                    flgPacketCheckResult = false; 
                }           
                
                /* Place a new read request. */
                USB_DEVICE_EndpointRead ( usbAppData.usbDevHandle, &usbAppData.readTranferHandle,
                            usbAppData.endpointRx, &receivedDataBuffer[0], sizeof(receivedDataBuffer) );
            }
            
            // Sending Data from rb_buffer to EP2Tx via USB port, if data are ready to be sent
            if (usbAppData.epDataWritePending == false)
            {
                   if (rb_size > USB_PACKET_SIZE) // checking if rb_buffer is filled
                    {
                        if (USBSendData(&rb_buffer , &rb_read)) // Data is sent here
                        {
                            rb_size -= USB_PACKET_SIZE;
                            if (rb_size >= RB_LENGTH)
                            {
                                rb_usb_readout_overflow_cnt++;
                            }

                            //for debugging
                           //usbAppData.epDataWritePending = false;

                            if (rb_read==RB_LENGTH)		// adjust write pointer
                                rb_read = 0;	
                        }
                    }
            }
            
            break;

        case USB_STATE_ERROR:
            break;

        default:
            break;
    } 
}
/* *****************************************************************************
 End of File
 */
