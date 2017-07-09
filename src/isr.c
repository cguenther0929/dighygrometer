/******************************************************************************
*   FILE: isr.c
*
*   PURPOSE: Interrupt service routines are contained in this file.  This file contains
*           algorithms that pertain to the interrupt service routine.
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

#include "isr.h"                //Include the header file for this module.

struct GlobalInformation gblinfo;

void Init_Interrupts( void ) {
    IPEN = 1;           //This allows interrupts to be defined a priority (either high or low) -- see page 103/380
    GIEH = 1;           //Enable all interrupts with high priority
    GIEL = 1;           //Enable all interrupts with low priority

}

void interrupt high_priority edges_isr( void ) {     //TODO place interrupts from sensors in here

    if(RXB0IF){         //CAN Message Buffer contains message
        gblinfo.canmsgrxed = TRUE;      
        PIE5bits.RXB0IE = 0;     //Interrupt disables until message is processed.  p.406 in datasheet for more info
        PIR5bits.RXB0IF = 0;     //Clear this just to be safe
    }
    
    if(TXB0IF){
    }
    TXB0IF = 0;         //Clear the interrupt flag
}

void Events1ms(void) {                  //Keep routine slim!

}

void Events100ms(void) {                //Keep routine slim!
    if(!gblinfo.ledlock && gblinfo.fastblink){     
        HBLED = ~HBLED;                  //Fast blink occurs every 100mili seconds
    }
    //gblinfo.reportTH == FALSE ? (gblinfo.reportTH = TRUE):(gblinfo.systemerror = TRUE);                    //TODO put these lines in!!! Removed for debugging
    //gblinfo.accel_bcast_100ms == FALSE ? (gblinfo.accel_bcast_100ms = TRUE):(gblinfo.systemerror = TRUE);  //TODO put these lines in!!! Removed for debugging
    
    
    /*FOLLOWING CODE IS IN FOR DEBUGGING ONLY */
    //gblinfo.run_accel_test = TRUE;
    gblinfo.run_accel_test = TRUE;
}

void Events1000ms(void) {
    if(!gblinfo.ledlock && !gblinfo.fastblink){     
        HBLED = ~HBLED;                  //Fast blink occurs every 100mili seconds
    }
    
    /* TODO the following block of code is in for debugging only  */
    
    //if(gblinfo.tick1000m % 10 == 0)
    //    gblinfo.reportTH == FALSE ? (gblinfo.reportTH = TRUE):(gblinfo.systemerror = TRUE);
    //    
    //if(gblinfo.tick1000m % 20 == 0)
    //    gblinfo.accel_bcast_100ms == FALSE ? (gblinfo.accel_bcast_100ms = TRUE):(gblinfo.systemerror = TRUE);
    //
    //if(gblinfo.tick1000m % 30 == 0)
    //    gblinfo.bcast_lvl_data == TRUE ? (gblinfo.lvl_data_send = TRUE):(gblinfo.lvl_data_send = FALSE);
    //
    /* TODO This is the end of debug code */
    
    //gblinfo.bcast_lvl_data == TRUE ? (gblinfo.lvl_data_send = TRUE):(gblinfo.lvl_data_send = FALSE);  //TODO this line needs to be in
}

void interrupt low_priority main_isr( void ) {
    if(INTCONbits.TMR0IF) {                            //Timer 0 interrupt flag has been set
        // TMR0H = TMR0HIGH;                        //Load the high register for the timer -- looking for 1/100 of a tick1000m
        // TMR0L = TMR0LOW;                        //Load the low register for the timer
        
        // if(gblinfo.tick1m == 100) {
            // gblinfo.tick1m = 0;               //Reset centi-tick1000monds
            // Events100ms();
            // if(gblinfo.tick100m == 9) {         //Once Second Reached
                // gblinfo.tick100m = 0;           //Reset 100mili second counter
                // Events1000ms();                 //Look at events that are to happen every 1s
                // if(gblinfo.tick1000m == 59)                     //We've ticked away one minute, so reset
                    // gblinfo.tick1000m = 0;                      //Reset seconds counter
                // else
                    // gblinfo.tick1000m += 1;                     //Increment seconds counter
            // }
            // else {
                 // gblinfo.tick100m += 1;                         //Increment 100mili second timer 
            // }
        // }
        // else {
            // gblinfo.tick1m += 1;                               //Increment 50mili second timer
        // }

        INTCONbits.TMR0IF = 0;                         //Software is responsible for clearing this flag
    }
    
    if(PIR1bits.TMR1IF){                                     //Timer 1 interrupt
        TMR1H = TMR1HIGH;                        //Load the high register for the timer -- looking for 1/100 of a tick1000m
        TMR1L = TMR1LOW;                        //Load the low register for the timer
        
        if(gblinfo.tick1m == 100) {
            gblinfo.tick1m = 0;               //Reset centi-tick1000monds
            Events100ms();
            if(gblinfo.tick100m == 9) {         //Once Second Reached
                gblinfo.tick100m = 0;           //Reset 100mili second counter
                Events1000ms();                 //Look at events that are to happen every 1s
                if(gblinfo.tick1000m == 59)                     //We've ticked away one minute, so reset
                    gblinfo.tick1000m = 0;                      //Reset seconds counter
                else
                    gblinfo.tick1000m += 1;                     //Increment seconds counter
            }
            else {
                 gblinfo.tick100m += 1;                         //Increment 100mili second timer 
            }
        }
        else {
            gblinfo.tick1m += 1;                               //Increment 50mili second timer
        }

        PIR1bits.TMR1IF = 0;                         //Software is responsible for clearing this flag
    }   /* END IF FOR TMR1IF */
    
    if(PIR2bits.TMR3IF){
        PIR2bits.TMR3IF = 0;                     //Clear the interrupt flag  
    } /* END IF FOR TMR3IF */
    
} /* END void interrupt low_priority main_isr( void ) */

void DisableInterrupts( void ) {
    GIEH = 0;           //Disable high priority interrupts
    GIEL = 0;           //Disable low priority interrupts
}

void EnableInterrupts( void ) {
    GIEH = 1;           //Enable high priority interrupts
    GIEL = 1;           //Enable low priority interrupts
}