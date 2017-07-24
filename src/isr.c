/******************************************************************************
*   FILE: isr.c
*
*   PURPOSE: Interrupt service routines are contained in this file.  This file contains
*           algorithms that pertain to the interrupt service routine.
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

#include "isr.h"                //Include the header file for this module.

struct GlobalInformation gblinfo;

void Init_Interrupts( void ) {
    IPEN = 1;           //This allows interrupts to be defined a priority (either high or low) -- see page 103/380
    GIEH = 1;           //Enable all interrupts with high priority
    GIEL = 1;           //Enable all interrupts with low priority

}

void PORTBINTSetup( void ){
	
    /* SETUP FOR INT 0*/
    INT0IF = 0;     //Clear this flag before enabling interrupt 
    INT0IE = 1;     //Enable the INT0 interrupt
    
    /* SETUP FOR INT 1*/
    //INT1IF = 0;     //Clear this flag before enabling interrupt 
    //INT1IP = 0;     //Define this to be low priority
    //INT1IE = 1;     //Enable the INT0 interrupt
    
    /* SETUP FOR INT 2*/
    //INT2IF = 0;     //Clear this flag before enabling interrupt 
    //INT2IP = 0;     //Define this to be low priority
    //INT2IE = 1;     //Enable the INT0 interrupt

}


void interrupt high_priority edges_isr( void ) {     

    // if(RXB0IF){         //CAN Message Buffer contains message
    //     gblinfo.canmsgrxed = TRUE;      
    //     PIE5bits.RXB0IE = 0;     //Interrupt disabled until message is processed.  p.406 in datasheet for more info
    //     PIR5bits.RXB0IF = 0;     //Clear this just to be safe
    // }
    
    // if(TXB0IF){
    //     TXB0IF = 0;         //Clear the interrupt flag
    // }
    
    // if(INT0IF){
    //     INT0IF = 0;                             
    // }
}

void interrupt low_priority main_isr( void ) {
    // if(TMR0IF) {                            //Timer 0 interrupt flag has been set
    //     TMR0IF = 0;                         //Software is responsible for clearing this flag
    // }
    
    if(TMR0IF){                                     //Timer 1 interrupt
        TMR0H = TMR0HIGH;                        //Load the high register for the timer -- looking for 1/100 of a tick1000ms
        TMR0L = TMR0LOW;                        //Load the low register for the timer
        
        Events10ms();
        
        if(gblinfo.tick10ms == 9) {
            gblinfo.tick10ms = 0;               //Reset centi-tick1000monds
            Events100ms();
            if(gblinfo.tick100ms == 9) {         //Once Second Reached
                gblinfo.tick100ms = 0;           //Reset 100 milliseconds ounter
                Events1000ms();                 //Look at events that are to happen every 1s
                if(gblinfo.tick1000ms == 59)                     //We've ticked away one minute, so reset
                    gblinfo.tick1000ms = 0;                      //Reset seconds counter
                else
                    gblinfo.tick1000ms += 1;                     //Increment seconds counter
            }
            else {
                 gblinfo.tick100ms += 1;                         //Increment 100 millisecond timer 
            }
        }
        else {
            gblinfo.tick10ms += 1;                               //Increment 1 millisecond timer
        }

        TMR0IF = 0;                         //Software is responsible for clearing this flag
    }   /* END IF FOR TMR1IF */
    
    if(PIR2bits.TMR3IF){
        PIR2bits.TMR3IF = 0;                     //Clear the interrupt flag  
    } /* END IF FOR TMR3IF */
    
    if(INT0IF){
        INT0IF = 0;                             
    }
    
    if(INT1IF){
        INT1IF = 0;     //Clear the interrupt flag
    }
    
    if(INT2IF){
        INT2IF = 0;     //Clear the interrupt flag
    }
    
} /* END void interrupt low_priority main_isr( void ) */

void Events10ms(void) {                  //Keep routine slim!
}

void Events100ms(void) {                //Keep routine slim!
    
}

void Events1000ms(void) {
    PULSEOUT = ~PULSEOUT;
}


void DisableInterrupts( void ) {
    GIEH = 0;           //Disable high priority interrupts
    GIEL = 0;           //Disable low priority interrupts
}

void EnableInterrupts( void ) {
    GIEH = 1;           //Enable high priority interrupts
    GIEL = 1;           //Enable low priority interrupts
}