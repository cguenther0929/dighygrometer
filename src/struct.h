/******************************************************************************
*   FILE: struct.h
*
*   PURPOSE: Structures used throughout the code base are defined here.  
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
#ifndef __STRUCT_H_
#define __STRUCT_H_

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"             //Project specific header file

extern struct GlobalInformation         //Structure to pass information that is shared among modules
{
    uint16_t tick1m;           //Increases every 50mili tick1000monds 
    uint16_t tick100m;              //Used to keep track of passing deci-tick1000monds
    uint16_t tick1000m;               //Use this to tick tick1000monds -- might not be used for audio interface board
    
    bool ledlock;               //bool value put in place mainly for testing
    bool fastblink;
    bool canmsgrxed;            //Set this flag when we receive a can message
    bool bcast_lvl_data;            //This flag will be set to true when we are to broadcast level data every 1000ms
    bool lvl_data_send;             //Set when it's time to broadcast level data
    bool reportTH;                  //Set when we need to report Temp and Humidity data
    bool systemerror;               //Set for any error.  Very generic. May not be used
    bool accel_bcast_100ms;         //Broadcast the accel rolling average (trigger every 100ms)
    bool accel_data_ready;          //Triggered via accelerometer interrupt on pin
    bool run_accel_test;            //TODO for debugging only 
    
    uint16_t AccelXAvg100ms;       //Used for 100ms rolling average on X-axis
    uint16_t AccelXAvg500ms;      //Used for 500ms rolling average on X-axis
    uint16_t AccelXAvg1000ms;     //Used for 1000ms rolling average on X-axis
    
    uint16_t AccelYAvg100ms;        //Used for 100ms rolling average on Y-axis
    uint16_t AccelYAvg500ms;         //Used for 500ms rolling average on Y-axis
    uint16_t AccelYAvg1000ms;       //Used for 1000ms rolling average on Y-axis
    
    uint16_t AccelZAvg100ms;        //Used for 100ms rolling average on Z-axis
    uint16_t AccelZAvg500ms;         //Used for 500ms rolling average on Z-axis
    uint16_t AccelZAvg1000ms;       //Used for 1000ms rolling average on Z-axis
    
    uint8_t MaxAmplitude100ms;       //Keeps track of which axis has the largest magnitude for this rolling average
    uint8_t MaxAmplitude500ms;       //Keeps track of which axis has the largest magnitude for this rolling average
    uint8_t MaxAmplitude1000ms;      //Keeps track of which axis has the largest magnitude for this rolling average
    
    uint8_t HW_ID;                  //Will contain the address of this device (4 bits)
    
    float TempSlope;                //Used for linear interpolation of temperature (Line Slope)
    float TempInt;                  //Used for linear interpolation of temperature  (Line Intercept)
    float HumSlope;                 //Used for linear interpolation of humidity (Line Slope)
    float HumInt;                   //Used for linear interpolation of humidity (Line Intercept)
}GlobalInformation;


#endif