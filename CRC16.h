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

#ifndef _CRC16_H    /* Guard against multiple inclusion */
#define _CRC16_H


#include "app.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

 /*
 * The width of the CRC calculator and result.
 * Modify the typedef for an 8 or 32-bit CRC standard.
 */
typedef uint16_t ushort;
typedef unsigned char	Byte;

#define CRC16_BYTES_LENGTH                          2
#define CRC_ADS_COMMAND_HEADER_LENGTH               7
#define CRC_ADS_COMMAND_ENCODE_LENGTH               8
#define CRC_ADS_COMMAND_DECODE_LENGTH               10

#define CRC16_INCORRECT     false
#define CRC16_CORRECT       true
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Interface Functions
// *****************************************************************************
// *****************************************************************************
int CRC16_Initialize( void );

void CRC16GenerateTable();

ushort CRC16CheckSum(int len, Byte* data);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
