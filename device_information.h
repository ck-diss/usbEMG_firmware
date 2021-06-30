/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    /* ************************************************************************** */
    /** Descriptive Constant Name

      @Summary
        Brief one-line summary of the constant.
    
      @Description
        Full description, explaining the purpose and usage of the constant.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
      @Remarks
        Any additional remarks
     */
/* TIMESTAMP: */
//1439596800 //Saturday, August 15th 2015, 00:00:00 (GMT) 
// byteTimestamp {0x55 0xCE 0x81 0x00} 
#define usbCLK  0x00
#define usbEMG  0x01
#define usbMOT  0x02
/* ************************************************************************** */
/* Define if Device has a ADS1298 or ADS1299                                  */
/* ************************************************************************** */
    
#define DEVICE_IS_ADS_1299      true
#define AUTOSTART_ADS           false
#define USE_EEPROM_SAVED_CONFIG true
#define USB_DESCRIPTOR_DEVICE_NAME  {'E','M','G','-','0','0','9'}
    
/* ************************************************************************** */
/*                                  */
/* ************************************************************************** */
//TODO 
//getTimestamp of compilation
    //https://www.epochconverter.com/
    //https://tools.keycdn.com/hex-converter?value=1528814450&input=dec

static char TSTP_HW_usbEMGv2_3 [4]  = {0x5A, 0x97, 0x42, 0x80};  //03/18 5A974280
static char HW_Version_usbEMG [2] = {0x02, 0x04};  //Hardwareversion 2.4

static char TSTP_SW_usbEMGv2_3 [4]={0x59, 0xCA, 0x15, 0xAA};  
static char SW_Version_usbEMG [2]={0x01, 0x04};  //Softwareversion 1.4
    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

    /** Descriptive Data Type Name

      @Summary
        Brief one-line summary of the data type.
    
      @Description
        Full description, explaining the purpose and usage of the data type.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Remarks
        Any additional remarks
        <p>
        Describe enumeration elements and structure and union members above each 
        element or member.
     */

typedef enum
{
    /* Application's state machine's initial state. */
    DEVICE_STATE_INIT=0,
            
    DEVICE_STATE_ID_READ_COMPLETE,
            
            DEVICE_STATE_EEPROM_ID_READ,
            
            DEVICE_STATE_EEPROM_ID_READ_WAIT,
            
            DEVICE_STATE_EEPROM_ID_READ_COMPLETE,
            
            DEVICE_STATE_SET_HWVersion,
            
            DEVICE_STATE_SET_SWVersion,
            
    DEVICE_STATE_MAIN_TASK,

            

} DEVICE_STATES;


typedef struct
{
    /* Application state*/
    DEVICE_STATES state;
    
} DEVICE_DATA;


extern unsigned char deviceCode;


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    int device_Initialize ( void );
    void deviceService ( void );


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
