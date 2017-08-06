/******************************************************************************
*   FILE: config.h
*
*   PURPOSE: Configuration file specific to the processor being used and the 
*           underlying hardware. 
*
*   DEVICE: PPIC18F66K22
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
#ifndef __CONFIG_H_
#define __CONFIG_H_

//#include "type.h"       //Needed for defined functions
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>

/* DEFINE CODE VERSION NUMBER */
#define MAJVER              0x00
#define MINVER              0x00
#define BUGVER              0x01

/* DEFINES FOR STATUS GPIO */
#define PULSEOUT            LATBbits.LATB5

/* DEFINES FOR POWER CONTROL*/  
#define DISP_PWR_EN_n       LATEbits.LATE7      //Assert low to enable power to the display
#define PIEZ_KILL           LATEbits.LATE6      //Assert high to kill stored power keeping power to the MCU.
#define MCU_PWR_LATCH       LATEbits.LATE5      //Assert high to keep power to the MCU    

/* DEFINES FOR LCD PINS*/
#define DISP_BYTE           LATD                //Port that connects to dispaly
#define DISP_RS             LATEbits.LATE2      //Register Select signal. RS=0=Command, RS=1=Data
#define DISP_RW             LATEbits.LATE1      //Read/Write. R/W=1=Read; R/W=0=Write 
#define DISP_E              LATEbits.LATE0      //Operation enable (falling edge to trigger)
#define disp_write          0
#define disp_read           1
#define disp_command        0
#define disp_data           1

/* REGISTER VALUES FOR TIME BASE */         
#define TMR0HIGH            251                 //Defined assuming 500kHz internal oscillator and prescaler of 1 interrupt every 10ms 
#define TMR0LOW             29                  //Defined assuming 500kHz internal oscillator and prescaler of 1 interrupt every 10ms

/* I2C ADDDRESSES FOR HUMIDITY SENSOR */
#define HumBaseAddr         0x5F
#define HumIDReg_r          0x0F
#define HumStatReg_r        0x27
#define HumHuLo_r           0x28
#define HumHuHi_r           0x29
#define HumTmpLo_r          0x2A
#define HumTmpHi_r          0x2B
#define HumAvConf_rw        0x10
#define HumCtrl1_rw         0x20
#define HumCtrl2_rw         0x21
#define HumCtrl3_rw         0x22

#define HumT0DegCx8_rw      0x32        //Temperature-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumT1DegCx8_rw      0x33        //Temperature-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumT0T1msb_rw       0x35        //Temperature-related calibration values used to construct transfer function that allows conversion of digital value read
#define HumT0Lo_rw          0x3C        //Temperature-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumT0Hi_rw          0x3D        //Temperature-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumT1Lo_rw          0x3E        //Temperature-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumT1Hi_rw          0x3F        //Temperature-related calibration values used to construct transfer function that allows conversion of digital value read    

#define HumH0rhx2_rw        0x30        //Humidity-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumH1rhx2_rw        0x31        //Humidity-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumH0T0Lo_rw        0x36        //Humidity-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumH0T0Hi_rw        0x37        //Humidity-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumH1T0Lo_rw        0x3A        //Humidity-related calibration values used to construct transfer function that allows conversion of digital value read    
#define HumH1T0Hi_rw        0x3B        //Humidity-related calibration values used to construct transfer function that allows conversion of digital value read    

/* DEFINE VARIOUS PIN FUNCITONS */
#define output              0           //Define the output pin direction setting
#define input               1

/* BATTERY STATUS DEFINES */
#define BAT_VOLT_MIN        2.5         //Battery voltage at or below this value will trip the alert message          

#endif