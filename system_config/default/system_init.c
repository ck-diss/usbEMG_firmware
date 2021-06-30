/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits, 
    and allocates any necessary global system resources, such as the 
    sysObj structure that contains the object handles to all the MPLAB Harmony 
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include "system_config.h"
#include "system_definitions.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     ON
#pragma config ICESEL =     ICS_PGx1
#pragma config PWP =        OFF
#pragma config BWP =        OFF
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      PRIPLL
#pragma config FSOSCEN =    OFF
#pragma config IESO =       ON
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FPBDIV =     DIV_1
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     OFF
#pragma config FWDTWINSZ =  WINSZ_25

/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_4
#pragma config FPLLMUL =    MUL_24
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLIDIV =   DIV_4
#pragma config UPLLEN =     ON

/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   OFF
#pragma config FVBUSONIO =  OFF
// </editor-fold>


// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************

//<editor-fold defaultstate="collapsed" desc="USB Stack Initialization Data">



/**************************************************
 * USB Device Function Driver Init Data
 **************************************************/
/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1] =
{
    /* Function 1 */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 0,       /* First interfaceNumber of this function */ 
        .speed = USB_SPEED_FULL,    /* Function Speed */ 
        .numberOfInterfaces = 1,    /* Number of interfaces */
        .funcDriverIndex = 0,  /* Index of Vendor Driver */
        .driver = NULL,            /* No Function Driver data */ 
        .funcDriverInit = NULL     /* No Function Driver Init data */
    },
};

/*******************************************
 * USB Device Layer Descriptors
 *******************************************/
/*******************************************
 *  USB Device Descriptor 
 *******************************************/
const USB_DEVICE_DESCRIPTOR deviceDescriptor =
{
    0x12,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,          // DEVICE descriptor type
    0x0200,                         // USB Spec Release Number in BCD format
    0x00,                           // Class Code
    0x00,                           // Subclass code
    0x00,                           // Protocol code
    USB_DEVICE_EP0_BUFFER_SIZE,     // Max packet size for EP0, see system_config.h
    0x04D8,                         // Vendor ID
    0xBBFF,                         // Product ID
    0x0100,                         // Device release number in BCD format
    0x01,                           // Manufacturer string index
    0x02,                           // Product string index
    0x03,                           // Device serial number string index // default: 0x03
    0x01                            // Number of possible configurations
};


/*******************************************
 *  USB Full Speed Configuration Descriptor
 *******************************************/
const uint8_t fullSpeedConfigurationDescriptor[]=
{
    /* Configuration Descriptor */

    0x09,                                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                       // Descriptor Type
    39,0,                //(32 Bytes)Size of the Config descriptor.e
    1,                                               // Number of interfaces in this cfg
    0x01,                                               // Index value of this configuration
    0x00,                                               // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED, // Attributes
    50,                                                 // Max power consumption (2X mA)
    /* Descriptor for Function 1 - Vendor     */ 
    
    /* Interface Descriptor */

    0x09,                       // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,   // INTERFACE descriptor type
    0,                          // Interface Number
    0,                          // Alternate Setting Number
    3,                          // Number of endpoints in this intf
    0xFF,                       // Class code
    0x00,                       // Subclass code
    0x00,                       // Protocol code
    0,                          // Interface string index

    /* Endpoint Descriptor 1 */

    0x07,                       // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,    // Endpoint Descriptor
    1 |USB_EP_DIRECTION_OUT,   // EndpointAddress
    USB_TRANSFER_TYPE_BULK,     // Attributes
    0x40,0x00,                  // Max packet size of this EP
    1,                          // Interval

    /* Endpoint Descriptor 2 */

    0x07,                       // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,    // Endpoint Descriptor
    1 |USB_EP_DIRECTION_IN,    // EndpointAddress
    USB_TRANSFER_TYPE_BULK,     // Attributes
    0x40,0x00,                  // Max packet size of this EP
    1,                           // Interval
    
    /* Endpoint Descriptor 4 */

    0x07,                       // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,    // Endpoint Descriptor
    2 |USB_EP_DIRECTION_IN,    // EndpointAddress
    USB_TRANSFER_TYPE_BULK,     // Attributes
    0x40,0x00,                  // Max packet size of this EP
    1,                           // Interval

};

/*******************************************
 * Array of Full speed config descriptors
 *******************************************/
USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    fullSpeedConfigurationDescriptor
};


/**************************************
 *  String descriptors.
 *************************************/

 /*******************************************
 *  Language code string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;   // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type 
        uint16_t string[1];     // String
    }
    sd000 =
    {
        0, // Index of this string is 0
        0, // This field is always blank for String Index 0
        sizeof(sd000)-sizeof(sd000.stringIndex)-sizeof(sd000.languageID),
        USB_DESCRIPTOR_STRING,
        {0x0409}                // Language ID
    };  
/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;    // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[25];    // String
    }
    sd001 =
    {
        1,      // Index of this string descriptor is 1. 
        0x0409, // Language ID of this string descriptor is 0x0409 (English)
        sizeof(sd001)-sizeof(sd001.stringIndex)-sizeof(sd001.languageID),
        USB_DESCRIPTOR_STRING,
        {'M','i','c','r','o','c','h','i','p',' ','T','e','c','h','n','o','l','o','g','y',' ','I','n','c','.'}
    };

/*******************************************
 *  Product string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;   // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type 
        uint16_t string[11];    // String
    }
    sd002 =
    {
        2,       // Index of this string descriptor is 2. 
        0x0409,  // Language ID of this string descriptor is 0x0409 (English)
        sizeof(sd002)-sizeof(sd002.stringIndex)-sizeof(sd002.languageID),
        USB_DESCRIPTOR_STRING,
        {'u','s','b','E','M','G',' ','V','2','.','3'} 
    }; 
/*******************************************
 *  Serial Number string descriptor
 *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;   // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[7];    // String
    }
    sd003 =
    {
        3,       // Index of this string descriptor is 3. 
        0x0409,  // Language ID of this string descriptor is 0x0409 (English)
        sizeof(sd003)-sizeof(sd003.stringIndex)-sizeof(sd003.languageID),
        USB_DESCRIPTOR_STRING,
        USB_DESCRIPTOR_DEVICE_NAME
        //{'E','M','G','-','0','0','3'}
    };
    
///*******************************************
// * OS string descriptor (WinUSB, index 238)
// *******************************************/
    const struct __attribute__ ((packed))
    {
        uint8_t stringIndex;    //Index of the string descriptor
        uint16_t languageID ;   // Language ID of this string.
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type 
        uint16_t string[18];    // String
    }
    microSoftOsDescriptor =
    {
        0xEE,       /* This value is per Microsoft OS Descriptor documentation */  
        0x0000,     /* Language ID is 0x0000 as per Microsoft Documentation */  
        0x18,       /* Size is 18 Bytes as Microsoft documentation */  
        USB_DESCRIPTOR_STRING,
        /* Vendor code to retrieve OS feature descriptors.  */ 
        /* qwSignature = MSFT100 */ 
        /* Vendor Code = 0xFF */
        {'M','S','F','T','1','0','0',0x0100 | APP_USB_MICROSOFT_OS_DESCRIPTOR_VENDOR_CODE} 
    };
    
/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[5]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002,
    (const uint8_t *const)&sd003,
    (const uint8_t *const)&microSoftOsDescriptor     
};


/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    &deviceDescriptor,          /* Full speed descriptor */
    1,                          /* Total number of full speed configurations available */
    fullSpeedConfigDescSet,     /* Pointer to array of full speed configurations descriptors*/
    NULL, 
    0, 
    NULL, 
    5,                          // Total number of string descriptors available.
    stringDescriptors,          // Pointer to array of string descriptors.
    NULL, 
    NULL
};


/****************************************************
 * USB Device Layer Initialization Data
 ****************************************************/
const USB_DEVICE_INIT usbDevInitData =
{
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Number of function drivers registered to this instance of the
       USB device layer */
    .registeredFuncCount = 1,
    
    /* Function driver table registered to this instance of the USB device layer*/
    .registeredFunctions = (USB_DEVICE_FUNCTION_REGISTRATION_TABLE*)funcRegistrationTable,

    /* Pointer to USB Descriptor structure */
    .usbMasterDescriptor = (USB_DEVICE_MASTER_DESCRIPTOR*)&usbMasterDescriptor,

    /* USB Device Speed */
    .deviceSpeed = USB_SPEED_FULL,
    
    /* Index of the USB Driver to be used by this Device Layer Instance */
    .driverIndex = DRV_USBFS_INDEX_0,

    /* Pointer to the USB Driver Functions. */
    .usbDriverInterface = DRV_USBFS_DEVICE_INTERFACE,
    
    /* Specify queue size for vendor endpoint read */
    .queueSizeEndpointRead = 4,
    
    /* Specify queue size for vendor endpoint write */
    .queueSizeEndpointWrite= 4,
};

// </editor-fold>



// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************

//<editor-fold defaultstate="collapsed" desc="DRV_Timer Initialization Data">

// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="DRV_SPI Initialization Data">
 
 /*** SPI Driver Initialization Data ***/
// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="DRV_I2C Initialization Data">
// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">

/******************************************************
 * USB Driver Initialization
 ******************************************************/
/****************************************************
 * Endpoint Table needed by the Device Layer.
 ****************************************************/
uint8_t __attribute__((aligned(512))) endPointTable[DRV_USBFS_ENDPOINTS_NUMBER * 32];
const DRV_USBFS_INIT drvUSBInit =
{
    /* Assign the endpoint table */
    .endpointTable= endPointTable,

    /* Interrupt Source for USB module */
    .interruptSource = INT_SOURCE_USB_1,
    
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    .operationMode = DRV_USBFS_OPMODE_DEVICE,
    
    .operationSpeed = USB_SPEED_FULL,
    
    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USB_ID_1
};

// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;


// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Device Control System Service Initialization Data
  
  <editor-fold defaultstate="collapsed" 
  desc="Device Control System Service Initialization Data">
*/

const SYS_DEVCON_INIT sysDevconInit =
{
    .moduleInit = {0},
};

// </editor-fold>


// *****************************************************************************
// *****************************************************************************
// Section: Static Initialization Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( SYS_INIT_DATA *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&sysDevconInit);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_DEVCON_JTAGDisable();
    SYS_PORTS_Initialize();
    SYS_DMA_Initialize();
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA0, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA0, INT_SUBPRIORITY_LEVEL3);
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA1, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA1, INT_SUBPRIORITY_LEVEL1);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_0);
    SYS_INT_SourceEnable(INT_SOURCE_DMA_1);



    /* Initialize Drivers */
    /*Initialize TMR0 */
    DRV_TMR0_Initialize();
    /*Initialize TMR1 */
    DRV_TMR1_Initialize();
    /*Initialize TMR2 */
    DRV_TMR2_Initialize();
    /*Initialize TMR3 */
    DRV_TMR3_Initialize();
 
    DRV_SPI0_Initialize();
    DRV_I2C0_Initialize();



    /*Initialize OC0 */
    DRV_OC0_Initialize();

    /*Initialize OC1 */
    DRV_OC1_Initialize();
    /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBFS_Initialize(DRV_USBFS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBInit);

    /* Initialize System Services */
    SYS_INT_Initialize();  
	
    /*Setup the INT_SOURCE_EXTERNAL_4 and Enable it*/
    SYS_INT_VectorPrioritySet(INT_VECTOR_INT4, INT_PRIORITY_LEVEL4);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_INT4, INT_SUBPRIORITY_LEVEL1);
    SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE4,INT_EDGE_TRIGGER_FALLING);
    SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_4);

    /*Setup the INT_SOURCE_EXTERNAL_0 and Enable it*/
    SYS_INT_VectorPrioritySet(INT_VECTOR_INT0, INT_PRIORITY_LEVEL5);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_INT0, INT_SUBPRIORITY_LEVEL2);
    SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE0,INT_EDGE_TRIGGER_RISING);
    SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_0);





    /* Initialize Middleware */
    /* Set priority of USB interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL4);

    /* Set Sub-priority of USB interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);


    /* Initialize the USB device layer */
    sysObj.usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);
    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

