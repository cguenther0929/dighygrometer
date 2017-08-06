/******************************************************************************
*   FILE: i2c.h
*
*   PURPOSE: Header file for i2c.c
*
*   DEVICE: PIC18F66K22
*
*   COMPILER: Microchip XC8 v1.32
*
*   IDE: MPLAB X v3.45
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
#define Oscillator      500000                    //Device oscillator value
#define I2CCLOCK        50000                      //I2C Clock Value   
#define BaudValue       (Oscillator/(4*I2CCLOCK)) - 1     //Value for SSPADD register (master-mode only)          
#define i2cdelay        10

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

/********************************************************  
*FUNCTION: uint8_t ReadI2C(uint8_t addr)
*PURPOSE: To read a byte of data over the I2C bus.  
*PRECONDITION: I2C module must be configured.
*       Bus needs to be idle. Must pass functions proper
*       addresses.  
*POSTCONDITION: BYTE of data will be returned to application
*RETURN: The BYTE that was read over the bus
********************************************************/
uint8_t I2CRead(uint8_t baseaddress, uint8_t subaddress);

/********************************************************  
*FUNCTION: void I2CWrite(uint8_t baseaddress, uint8_t subaddress, uint8_t senddata)
*PURPOSE: Write a BYTE of data over the I2C bus.  
*PRECONDITION: I2C module must be configured.
*       Bus needs to be idle. Must pass in proper
*       addresses and data to be sent
*POSTCONDITION: BYTE of data written over bus
*RETURN: Nothing
********************************************************/
void I2CWrite(uint8_t baseaddress, uint8_t subaddress, uint8_t senddata);  

#endif