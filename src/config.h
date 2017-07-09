/******************************************************************************
*   FILE: config.h
*
*   PURPOSE: Configuration file specific to the processor being used and the 
*           underlying hardware. 
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
#ifndef __CONFIG_H_
#define __CONFIG_H_

//#include "type.h"       //Needed for defined functions
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>

/* DEFINE CODE VERSION NUMBER */
#define MAJVER              0x00
#define MINVER              0x00
#define BUGVER              0x02


/* DEFINES FOR STATUS LEDS */
#define HBLED               LATCbits.LATC0
#define REDLED              LATCbits.LATC1
#define GRNLED              LATCbits.LATC5
#define YELLED              LATCbits.LATC2

/* CAN ADDRESS BITS */
#define b0                  PORTAbits.RA3
#define b1                  PORTAbits.RA5
#define b2                  PORTAbits.RA7
#define b3                  PORTAbits.RA6

/* REGISTER VALUES FOR TIME BASE */
#define TMR0HIGH            248        //Defined assuming 16MHz internal oscillator and prescaler of 64 interrupt every 1ms
#define TMR0LOW             47         //Defined assuming 16MHz internal oscillator and prescaler of 64 interrupt every 1ms


#define TMR1HIGH            252        //Defined assuming 16MHz internal oscillator and prescaler of 64 interrupt every 1ms
#define TMR1LOW             23         //Defined assuming 16MHz internal oscillator and prescaler of 64 interrupt every 1ms

/* I2C ADDDRESSES FOR ACCELEROMETER */
#define AccBaseAddr         0x19        //Base (7 bit) address of accelerometer
#define AccIDReg_r          0x0F
#define AccStatusReg_r      0x27        
#define AccXLo_r            0x28
#define AccXHi_r            0x29
#define AccYLo_r            0x2A
#define AccYHi_r            0x2B
#define AccZLo_r            0x2C
#define AccZHi_r            0x2D
#define AccIntSrc_r         0x31
#define AccClickSrc_r       0x39
#define AccStatusAux_rw     0x07       
#define AccTempCfg_rw       0x1F       //Will always return 0x33
#define AccCtrl1_rw         0x20       //Data rate and axis enables defined here
#define AccCtrl2_rw         0x21       //Define data filters
#define AccCtrl3_rw         0x22       //For manipulated interrupts
#define AccCtrl4_rw         0x23       //Set sensitivity via this register
#define AccCtrl5_rw         0x24
#define AccCtrl6_rw         0x25
#define AccFIFOCtrl_rw      0x2E
#define AccIntCfg_rw        0x30
#define AccIntThs_rw        0x32
#define AccIntDura_rw       0x33
#define AccClickCfg_rw      0x38
#define AccClickThs_rw      0x3A
#define AccTimeLimit_rw     0x3B
#define AccTimeLatency_rw   0x3C
#define AccTimeWindow_rw    0x3D

/* ACCELEROMETER-RELATED PARAMETERS*/
#define AVG100MS_ACCEL      0.03        //300Hz sample rate, so sample every 3ms.  100ms/3ms ~= 33 samples in window.  So 1/33 = 0.03
#define AVG500MS_ACCEL      0.006       //300Hz sample rate, so sample every 3ms.  500ms/3ms ~= 166 samples in window.  So 1/166 = 0.006
#define AVG1000MS_ACCEL     0.003       //300Hz sample rate, so sample every 3ms.  500ms/3ms ~= 333 samples in window.  So 1/333 = 0.003

#define X_AXIS              1           //Mnemonic for identifying which axis has the largest magnitude 
#define Y_AXIS              2           //Mnemonic for identifying which axis has the largest magnitude 
#define Z_AXIS              3           //Mnemonic for identifying which axis has the largest magnitude 


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

#define ledon               0           //LEDs are active low
#define ledoff              1           //LEDs are active low

/* DEFINES FOR EEPROM */
#define CALID              	0x00 		//Start Address in EEPROM where the calibration identifier is located
#define XMSB              	0x01        //Start Address in EEPROM where the X-axis MSB calibration nibble is stored
#define XLSB              	0x02        //Start Address in EEPROM where the X-axis LSB calibration nibble is stored
#define YMSB                0x03        //Start Address in EEPROM where the Y-axis MSB calibration nibble is stored
#define YLSB                0x04        //Start Address in EEPROM where the Y-axis LSB calibration nibble is stored
#define ZMSB                0x05        //Start Address in EEPROM where the Z-axis MSB calibration nibble is stored
#define ZLSB                0x06        //Start Address in EEPROM where the Z-axis LSB calibration nibble is stored

 #endif