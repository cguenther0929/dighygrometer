/******************************************************************************
*   FILE: i2c.h
*
*   PURPOSE: Header file for i2c.c
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
#ifndef __I2C_H_
#define __I2C_H_

/* PULL IN OTHER H FILES AS NECESSARY */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* MASTER MODE CLOCK RATE */
#define Oscillator      8000000                     //Device oscillator value
#define I2CCLOCK        100000                      //I2C Clock Value   
#define BaudValue       (Oscillator/(4*I2CCLOCK)) - 1     //Value for SSPADD register (master-mode only)          
#define i2cdelay        50

/* DEFINE SPECIAL SFR NAMES FOR I2CxSTAT REGISTER */
#define I2CACKBit       SSPCON2bits.ACKDT       //Intelligent mnemonic for defining the ACK/NACK bit.
#define I2CGenACK       SSPCON2bits.ACKEN       //Intelligent mnemonic for generating an ACK/NACK
#define I2CGenStart     SSPCON2bits.SEN         //Intelligent mnemonic for generating a start condition
#define I2CGenStop      SSPCON2bits.PEN         //Intelligent mnemonic for generating a stop condition
#define I2CRepStart     SSPCON2bits.RSEN        //Intelligent mnemonic for generating a repeated start condition
#define I2CRecEnable    SSPCON2bits.RCEN        //Intelligent mnemonic for defining we wish to receive data
#define I2CACKStat      SSPCON2bits.ACKSTAT     //Intelligent mnemonic for determined if the slave sent an ACK/NACK

#define MSSP_Active     (SSPCON2bits.SEN | SSPCON2bits.RSEN | SSPCON2bits.PEN | SSPCON2bits.RCEN | SSPCON2bits.ACKEN) //Indicates when module is active -- Not tested

#define I2CStartActive  SSPCON2bits.SEN 		//Intelligent mnemonic indicating start condition is active 
#define I2CTXBusy       SSPSTATbits.READ_WRITE  //Intelligent mnemonic indicating a transmission is busy
#define I2CBF   	    SSPSTATbits.BF		    //Define special SFR name for Transmit Buffer Full Status Bit
#define I2CStopDetect   SSPSTATbits.P           //Intelligent mnemonic indicating a stop condition was detected
#define I2CStartDetect  SSPSTATbits.S           //Intelligent mnemonic indicating a start condition was detected

/* DEFINE THE READ WRITE BITS */
#define I2CREAD			1					    //Define the read bit to be high
#define I2CWRITE		0					    //Define the write bit to be low
#define ACK             0                       //An I2C ACK is active low
#define NACK            1                       //An I2C NACK is active high




/* DECLARE ALL FUNCTIONS */
/********************************************************
*FUNCTION: void I2Cinit( void )
*PURPOSE: To initalize the i2c bus module on the PIC
*PRECONDITION: NONE 
*POSTCONDITION: i2c module is ready to use as Master Mode
*				7 bit address mode
*RETURN: NOTHING
********************************************************/
void I2Cinit( void );

/********************************************************  //TODO fix comment 
*FUNCTION: uint8_t ReadI2C(uint8_t addr)
*PURPOSE: Send 7 bit addr To read 8 bits from the A/D
*PRECONDITION: Bus needs to be idle. Pull in address
*POSTCONDITION: 16 Data bits will be returned
*RETURN: 16 Bits of binary data (VDD/1024)*VALUE
********************************************************/
uint8_t I2CRead(uint8_t baseaddress, uint8_t subaddress);     //TODO add comment

void I2CWrite(uint8_t baseaddress, uint8_t subaddress, uint8_t senddata);


/********************************************************
*FUNCTION: 
*PURPOSE: 
*PRECONDITION: 
*POSTCONDITION: 
*RETURN: 
********************************************************/

#endif