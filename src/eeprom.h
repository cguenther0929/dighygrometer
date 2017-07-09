/******************************************************************************
*   FILE: eeprom.h
*
*   PURPOSE:  eeprom.c header file.  This file handles defines and declares functions
*               related to the EEPROM module
*
*
*   DEVICE: PIC18F25K80
*
*   COMPILER: Microchip XC8 v1.32
*
*   IDE: MPLAB X v1.60
*
*   TODO:  
*
*   NOTE:
*
******************************************************************************/
#ifndef __EEPROM_H_
#define __EEPROM_H_

#include <xc.h>         //Part specific header file
#include <stdint.h>

#include "isr.h"
#include "config.h"     //Project specific header file
#include "main.h"

/********************************************************
*FUNCTION: void CleanEEPROM( void )
*PURPOSE: Write all NULLS to EEPROM
*PRECONDITION: NA
*POSTCONDITION: EEPROM has been refreshed
*RETURN: Nothing
********************************************************/
void CleanEEPROM( void );

/********************************************************
*FUNCTION: void EEPROMWriteByte(uint8_t DataIn, uint8_t StartAddress)
*PURPOSE: Write a BYTE of data to the EEPROM at the location
        specified by StartAddress
*PRECONDITION: NA
*POSTCONDITION: BYTE of data written to EEPROM
*RETURN: Nothing
********************************************************/
void EEPROMWriteByte(uint8_t DataIn, uint8_t StartAddress);

/********************************************************
*FUNCTION: uint8_t EEPROMReadByte(uint8_t StartAddress)
*PURPOSE: Read a byte form the EEPROM at location specified
    by StartAddress
*PRECONDITION: NA
*POSTCONDITION: BYTE of data read from EEPROM
*RETURN: BYTE of data
********************************************************/
uint8_t EEPROMReadByte(uint8_t StartAddress);

/********************************************************
*FUNCTION: void EEPROMWriteWord(uint8_t DataArray[], uint8_t StartAddress)
*           DataArray shall always be 4 elements large
*PURPOSE: Write a double word value to the eeprom
*PRECONDITION: NA
*POSTCONDITION: Four bytes written to EEPROM memory
*RETURN: Nothing
********************************************************/
void EEPROMWriteWord(uint8_t DataArray[], uint8_t StartAddress);

/********************************************************
*FUNCTION: void EEPROMReadWord(uint8_t DataArray[], uint8_t StartAddress)
*           DataArray shall always be 4 elements large
*PURPOSE: Read four bytes from the EEPROM
*PRECONDITION: NA
*POSTCONDITION: Four bytes read from EEPROM memory starting
        at address defined by StartAddress
*RETURN: Nothing
********************************************************/
void EEPROMReadWord(uint8_t StartAddress);

/********************************************************
*FUNCTION: BrakeDown(float FloatVal, BTYE DataArray[])
*PURPOSE: Break 32 bit float into 4 individual bytes
*PRECONDITION: FloatVal shall be a valid float value
*POSTCONDITION: DataArray filled with four BYTES that
       represent the float value
*RETURN: Nothing
********************************************************/
void BrakeDown(float FloatVal, uint8_t DataArray[]);

#endif