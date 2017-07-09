/******************************************************************************
*   FILE: eeprom.c
*
*   PURPOSE: EEPROM module source file.  Contains Algorithms for all eeprom module
*           routines.
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

#include"eeprom.h"

void CleanEEPROM( void ){
    UINT i;
    for(i = 0; i < 0x3FF; i++) {
        EEPROMWriteByte(0x00,i);            //Write 0 to the memory location
    }
}

void EEPROMWriteByte(uint8_t DataIn, uint8_t StartAddress) {
    uint8_t i = 0;
    DisableInterrupts();        //TODO make sure this function is defined
    EEIF = 0;                  //This flag must be cleared to write to the EEPROM -- DO NOT REMOVE
    EEPGD = 0;                  //Tells device we wish to access the EEPROM
    CFGS = 0;                   //Tells device we wish to access the EEPROM
    WREN = 1;                   //Enable writes to the EEPROM

    EEADRH = 0x00;                      //Write 0 to the high address
    EEADR = StartAddress;               //Write the low address of the memory
    EEDATA = DataIn;                    //Write the character a to the register
    EECON2 = 0x55;                      //Need to write 0x55 to this register as part of the write sequence
    EECON2 = 0xAA;                      //Need to write 0xAA to this register as part of the write sequence


    EECON1bits.WR = 1;                      //Initiate the write sequence
    while (EEIF == 0);                  //Wait until the write is complete
    EEIF = 0;                           //This flag needs to be cleared in software!

    WREN = 0;                               //It is a good idea to keep this bit cleared during normal operation
    EEIF = 0;                           //Just to be safe, clear this flag
    EnableInterrupts();         //TODO make sure this function is defined
}

uint8_t EEPROMReadByte(uint8_t StartAddress) {
    uint8_t temp = 0;
    DisableInterrupts();

    EEADRH = 0x00;              //Write 0 to the high address
    EEADR = StartAddress;   //Write the low address of the memory
    EEPGD = 0;                  //Clear this bit as part of the sequence
    CFGS = 0;                   //Clear this bit as part of the sequence
    EECON1bits.RD = 1;                     //Set this bit so that the data become available at the EEDATA register
    temp = EEDATA;              //Grab the BYTE of information
    EnableInterrupts();

    return temp;
}

void EEPROMWriteWord(uint8_t DataArray[], uint8_t StartAddress) {
    uint8_t i = 0;
    DisableInterrupts();
    EEIF = 0;                  //This flag must be cleared to write to the EEPROM -- DO NOT REMOVE
    EEPGD = 0;              //Tells device we wish to access the EEPROM
    CFGS = 0;               //Tells device we wish to access the EEPROM
    WREN = 1;               //Enable writes to the EEPROM

    for(i = 0; i < 4; i++) {
        EEADRH = 0x00;                      //Write 0 to the high address
        EEADR = StartAddress + i;           //Write the low address of the memory
        EEDATA = DataArray[i];              //Write the character a to the register
        EECON2 = 0x55;                      //Need to write 0x55 to this register as part of the write sequence
        EECON2 = 0xAA;                      //Need to write 0xAA to this register as part of the write sequence


        EECON1bits.WR = 1;                             //Initiate the write sequence
        while (EEIF == 0);                  //Wait until the write is complete
        EEIF = 0;                           //This flag needs to be cleared in software!
    }

    WREN = 0;               //It is a good idea to keep this bit cleared during normal operation
    EEIF = 0;                           //Just to be safe, clear this flag

    EnableInterrupts();
}

//void EEPROMReadWord(uint8_t StartAddress) {
//    uint8_t i = 0;             //Use this as a counter
//    DisableInterrupts();
//    for(i = 0; i < 4; i++) {
//        EEADRH = 0x00;              //Write 0 to the high address
//        EEADR = StartAddress + i;   //Write the low address of the memory
//        EEPGD = 0;                  //Clear this bit as part of the sequence
//        CFGS = 0;                   //Clear this bit as part of the sequence
//
//        EECON1bits.RD = 1;                     //Set this bit so that the data become available at the EEDATA register
//        convert.buff[i] = EEDATA;               //Grab the BYTE of information
//    }
//    EnableInterrupts();
//
//}

void BrakeDown(float FloatVal, uint8_t DataArray[]) {
    uint32_t d = *(uint32_t *)&FloatVal;

    DataArray[0] = d & 0x00FF;
    DataArray[1] = (d & 0xFF00) >> 8;
    DataArray[2] = (d & 0xFF0000) >> 16;
    DataArray[3] = (d & 0xFF000000) >> 24;
}
