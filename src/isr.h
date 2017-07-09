/******************************************************************************
*   FILE: isr.h
*
*   PURPOSE: Header file for isr.c
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

#ifndef __ISR_H_
#define __ISR_H_

#include <xc.h>                 //Part specific header file
#include <stdint.h>
#include <stdbool.h>
//#include "type.h"
#include "config.h"             //Project specific header file
#include "main.h"
#include "struct.h"

/********************************************************
*FUNCTION: void Init_Interrupts( void )
*PURPOSE: This function is responsible for initializing
    interrupt features
*PRECONDITION: Desired interrupts not yet supports
*POSTCONDITION: Desired interrupts now enabled
*RETURN: Nothing
********************************************************/
void Init_Interrupts( void );

/********************************************************
*FUNCTION: void interrupt low_priority main_isr( void )
*PURPOSE: This is the low priority interrupt that will handle
    the majority of the interrupts generated.  Examples would
    be the UART and timer.  These are non-critical interrupts
*PRECONDITION: Interrupt flag not set.
*POSTCONDITION: Interrupt respond to and flag cleared
*RETURN: Nothing
********************************************************/
void interrupt low_priority main_isr( void );

/********************************************************
*FUNCTION: void interrupt high_priority edges_isr( void )
*PURPOSE: This is where the vector will point when we detect
    and edge on one of the TACH signal inputs
*PRECONDITION: Interrupt flag not set.
*POSTCONDITION: Interrupt respond to and flag cleared
*RETURN: Nothing
********************************************************/
void interrupt high_priority edges_isr( void );

/********************************************************
*FUNCTION: void DisableInterrupts( void )
*PURPOSE: Will prevent interrupts from firing
*PRECONDITION: Interrupts may be enabled.
*POSTCONDITION: Interrupts now disabled.
*RETURN: Nothing
********************************************************/
void DisableInterrupts( void );

/********************************************************
*FUNCTION: void EnableInterrupts( void )
*PURPOSE: Enable high and low priority interrupts
*PRECONDITION: Interrupts may be disabled.
*POSTCONDITION: Interrupts now enabled.
*RETURN: Nothing
********************************************************/
void EnableInterrupts( void );

void Events1ms(void);  //TODO need to comment

void Events100ms(void);     //TODO need to comment

void Events1000ms(void);        //TODO need to comment


#endif
/* END OF FILE */