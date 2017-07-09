/******************************************************************************
*   FILE: main.h
*
*   PURPOSE: Header file for main.c
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
#ifndef __MAIN_H_
#define __MAIN_H_

#include <xc.h>         //Part specific header file
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "eeprom.h"
#include "i2c.h"
#include "struct.h"
#include "isr.h"
#include "config.h"     //Project specific header file
#include "timer.h"
#include "can.h"

#define TEMP 65577

/********************************************************
*FUNCTION: void tick100mDelay( uint16_t tick100ms )
*PURPOSE: Much more accurate timer that works off interrupts
            User must define how many 1/10s of a tick1000mond he/she
            wishes to pass
*PRECONDITION: Timers must be set up and running in order for this to work
*POSTCONDITION: tick100ms * 1/10s of a tick1000mond have passed.
*RETURN: Nothing
********************************************************/
void tick100mDelay( uint16_t tick100ms );

/********************************************************
*FUNCTION: void tick1mDelay( uint16_t tick1ms )
*PURPOSE: Much more accurate timer that works off interrupts
            User passes in how many 1/50s he/she wishes to pass 
*PRECONDITION: Timer0 set up and running and set to interrupt
*POSTCONDITION: Blocking delay inserted
*RETURN: Nothing
********************************************************/
void tick1mDelay( uint16_t tick1ms );

/********************************************************
*FUNCTION: void SetUp( void );
*PURPOSE: Set up the PIC I/O and etc...
*PRECONDITION: PIC not configured
*POSTCONDITION: PIC I/O Configured
*RETURN: Nothing
********************************************************/
void SetUp( void );

void TaskUpdate (void);         //TODO need to comment

void TempHumidityInitialize(void);      //TODO need to comment 

void AccelTest(void);       //TODO this was only put in for testing

void THtest(void);       //TODO this was only put in for testing

void HandleNewAccelData( void );    //TODO comment

void BcastAccelRollingAverage( void ); //TODO comment 

void ReportLevelData( void );

void GrabHWid(void);  //TODO comment

void StoreCalibrationValues(void); //TODO comment

//               1 = Enable; 0 = Disable
//                  |       1 = Rising edge 0 = Falling edge
//                  |           |
void INT0Setup(uint8_t State, uint8_t Edge);  //--Fix comment


#endif