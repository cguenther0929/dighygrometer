/******************************************************************************
*   FILE: main.c
*
*   PURPOSE: Main source
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

#include "main.h"               //Include header file associated with main.c

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO1    // Oscillator (Internal RC oscillator, CLKOUT function on OSC2)
#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor (Enabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2L
#pragma config PWRTEN = ON     // Power Up Timer (Enabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 1         // Brown-out Reset Voltage bits (2.7V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1024     // Watchdog Postscaler (1:1024)

// CONFIG3H
#pragma config CANMX = PORTC    // ECAN Mux bit (ECAN TX and RX pins are located on RC6 and RC7, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-01FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 02000-03FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 04000-05FFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 06000-07FFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)


struct GlobalInformation gblinfo;

//union floatval convert;                 //Union to handle float conversions

void main() {
    uint16_t led_counter1 = 0;
    uint16_t led_counter2 = 0;
    uint8_t I2CdataL = 0;
	uint8_t I2CdataH = 0;
	uint16_t AccelDat = 0;
    SetUp();                    //Initialize hardware
    gblinfo.ledlock = TRUE;		//TODO for debugging purposes we don't allow the LED to blink inside the interrupt routine 
	/* BEGIN SUPER LOOP */
    while(1){
        //TaskUpdate();
        CLRWDT();           //Kick the dog
        //AccelTest();
        //GRNLED = ~GRNLED;
		
        led_counter1++;			//TODO this is in for debugging only
	if(led_counter1 >= 1000){
            led_counter1 = 0;
            led_counter2++;
            if(led_counter2 >= 50 ){
                led_counter2 = 0;
                //AccelTest();
                //I2CWrite(AccBaseAddr,AccCtrl1_rw,0x77);     //400Hz, Normal Power Mode, All three axes are enabled
                //I2CdataL = I2CRead(AccBaseAddr, AccXLo_r);
                //GRNLED = ~GRNLED;
				I2CdataL = I2CRead(AccBaseAddr, AccXLo_r);
				I2CdataH = I2CRead(AccBaseAddr, AccXHi_r);
				AccelDat = (uint16_t)((I2CdataH << 8) | I2CdataL);
            }
	}
        
    }  //END while (1) SUPER LOOP
}

void SetUp( void ) {
    uint16_t i;
    uint16_t j;
    
    SWDTEN = 1;     //SW enable WDT
    //OSCCONbits.IRCF = 7;                //Define HF-INTOSC at 16MHz
    //OSCTUNEbits.PLLEN = 1;

    for(i=0;i<1000;i++){
        for(j=0;j<100;j++);
    }
    
    /*PIN DIRECTION FOR LEDS */
    TRISC0 = output;                    //Port pin attached to the Heartbeat LED
	TRISC1 = output;					//Port pin attached to the RED LED
	TRISC2 = output;					//Port pin attached to the YELLOW LED
	TRISC5 = output;					//Port pin attached to the GREEN LED
    
    /* PIN DIRECTION FOR HW ID INPUTS */
    TRISA3 = input;                     //Hardware id bit 0
    TRISA5 = input;                     //Hardware id bit 1
    TRISA7 = input;                     //Hardware id bit 2
    TRISA6 = input;                     //Hardware id bit 3
    
    /* PIN DIRECTIONS FOR INTERRUPTS */ //TODO NEED TO DEFINE THESE AND MAKE THEM INPUTS
    
    /* PIN DIRECTIONS FOR I2C */
    I2Cinit();
    TRISC3 = input;                     //I2C SCK. Must be defined as input
    TRISC4 = input;                     //I2C SDA. Must be defined as input
    
    /* PIN DIRECTIONS FOR CAN*/
    TRISC7 = input;                     //Tied to CAN RX pin    
    TRISC6 = output;                    //Tied to CAN TX pin
	
    /* INITIAL LED STATES */
    HBLED = ledoff;
    REDLED = ledoff;
    YELLED = ledoff;
    GRNLED = ledoff;
	
	gblinfo.tick1m = 0;                   //Initialize 100s of a tick1000mond counter to 0
	gblinfo.tick100m = 0;               //Initialize 100s of a tick1000mond counter to 0
    gblinfo.tick1000m = 0;              //Seconds counter
    
	gblinfo.ledlock             = FALSE;    //Do not lock the LED blinking at startup
    gblinfo.fastblink           = FALSE;    //Normal blink rate
    gblinfo.canmsgrxed          = FALSE;    //Set upon CAN message RX interrupt
    gblinfo.bcast_lvl_data      = FALSE;    //Default for broadcasting level data is false
    gblinfo.reportTH            = FALSE;    //Flag is set when it is time to broadcast 
    gblinfo.systemerror         = FALSE;    //Any errors in the system, and this flag is set.  May not be used at first
    gblinfo.accel_bcast_100ms   = FALSE;    //Accel rolling average broadcast event flags
    gblinfo.accel_data_ready    = FALSE;    //Interrupt has triggered indicating more accel data is ready

    Init_Interrupts();                  //Set up interrupts
    //CanInit();
    
    //TODO not yet supported !! //INT0Setup(1,1);					//Set up INT0 interrupt
    
    //Timer0Init(1,2,0);                 //ARGS: interrupts = yes, prescaler = 1, clksource = FOSC/4
    //Timer0On();                         //Turn the timer one
    Timer1Init(1,2,0);                 //ARGS: interrupts = yes, prescaler = 2, clksource = FOSC/4
    Timer1On(TMR1HIGH,TMR1LOW);
    
    /* FUNCTIONS/FEATURES JUST PRIOR TO MAIN LOOP */
    HBLED  = ledon;
    REDLED = ledon;                         //To make sure all leds work  //>> TODO this debugging routine will likely be removed
    YELLED = ledon;
    GRNLED = ledon;
    tick100mDelay(7);
    
    HBLED  = ledoff;
    REDLED = ledoff;
    YELLED = ledoff;
    GRNLED = ledoff;
    tick100mDelay(7);
    
    HBLED  = ledon;
    REDLED = ledon;                         //To make sure all leds work  //>> TODO this debugging routine will likely be removed
    YELLED = ledon;
    GRNLED = ledon;
    tick100mDelay(7);
    
    HBLED  = ledoff;
    REDLED = ledoff;
    YELLED = ledoff;
    GRNLED = ledoff;
    
    gblinfo.fastblink = TRUE;
    
    /* TEMP/HUMIDITY SENSOR CONFIG*/
    I2CWrite(HumBaseAddr,HumCtrl1_rw,0x88);     //[7]=Enable Device,[6:3]=Reserved,[2]=High and low register read before update,[1:0]=ODR
    I2CWrite(HumBaseAddr,HumAvConf_rw,0x1B);    //[7:6]=Reserved,[5:3]=Temp Average,[2:0]=Humidity Average
    I2CWrite(HumBaseAddr,HumCtrl3_rw,0x04);        //[7]=DRDY Active State,[6]DRDY Push/Pull,[5:3]=Reserved,[2]=DRDY En,[1:0]=Reserved
    //TempHumidityInitialize();                   //Gather transfer function parameters from stored calibration values
    
    /* ACCELEROMETER SENSOR CONFIG*/
    I2CWrite(AccBaseAddr,AccCtrl3_rw,0x08);     //Define DRDY 1 on INT1
    //tick100mDelay(1);                           //Must have a small delay when back-to-back writes to registers have to occur
    I2CWrite(AccBaseAddr,AccCtrl4_rw,0x08);     //TODO comment Define that we read MSB and LSB prior to updating accel registers
    //tick100mDelay(1);
    I2CWrite(AccBaseAddr,AccTempCfg_rw,0x80);   //Enable the ADC
    //tick100mDelay(1);                           //Must have a small delay when back-to-back writes to registers have to occur
    I2CWrite(AccBaseAddr,AccCtrl1_rw,0x77);     //400Hz, Normal Power Mode, All three axes are enabled 
    
    
    
}

void TaskUpdate (void){
    //GRNLED = ~GRNLED;
    
    //if(gblinfo.canmsgrxed)
        //ProcessCanMessage();
        
    //if(gblinfo.accel_bcast_100ms) {     //Time to send out rolling averages
        //BcastAccelRollingAverage();
        //gblinfo.accel_bcast_100ms = FALSE;
    //}
    
    //if(gblinfo.lvl_data_send) {             //Will only be set true if bcast_lvl_data is set (via command bit)
      //  ReportLevelData();
        //gblinfo.lvl_data_send = FALSE;
    //}
    
    //if(gblinfo.systemerror)
        //REDLED = ledon;                     //Turn on the "bad things are happening LED"  
        
    //if(gblinfo.accel_data_ready) {
        //HandleNewAccelData();         //TODO we want this line in
      //  gblinfo.accel_data_ready = FALSE;
   // }
    
    //if(gblinfo.run_accel_test) {
      //  gblinfo.run_accel_test = FALSE;
        //AccelTest();
        //GRNLED = ~GRNLED;
    //}
    
}


void INT0Setup(uint8_t State, uint8_t Edge){
	State = 1 ? (INT0IE = 1):(INT0IE = 0);			//If 1, turn on interrupt
	Edge = 1 ? (INTEDG0 = 1):(INTEDG0 = 0);			//1 to interrupt on rising edge. 
}


void TempHumidityInitialize(void) {
    uint8_t T0degCx8Lo = 0;
    uint8_t T1degCx8Lo = 0;
    uint8_t T1T0degmsb = 0;
    uint16_t T0degCx8 = 0;
    uint16_t T1degCx8 = 0;
    
    uint16_t T0out = 0;
    uint16_t T1out = 0;
    uint8_t I2Cdata = 0;
    
    uint8_t H0rhx2 = 0;
    uint8_t H1rhx2 = 0;
    uint16_t H0T0out = 0;
    uint16_t H1T0out = 0;
    
    /* DERIVE LINE EQUATIONS FOR TEMPERATURE SENSOR */
    T0degCx8Lo = I2CRead(HumBaseAddr,HumT0DegCx8_rw);
    T1degCx8Lo = I2CRead(HumBaseAddr,HumT1DegCx8_rw);
    T1T0degmsb = I2CRead(HumBaseAddr,HumT0T1msb_rw);
    
    T0degCx8 = (uint16_t)((T1T0degmsb & 0x03) | T0degCx8Lo);
    T1degCx8 = (uint16_t)(((T1T0degmsb >> 2) & 0x03) | T1degCx8Lo);
    
    I2Cdata = I2CRead(HumBaseAddr,HumT0Hi_rw);
    T0out = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr,HumT0Lo_rw);
    T0out = (uint16_t)(T0out | I2Cdata);
    
    I2Cdata = I2CRead(HumBaseAddr,HumT1Hi_rw);
    T1out = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr,HumT1Lo_rw);
    T1out = (uint16_t)(T1out | I2Cdata);
    
    gblinfo.TempSlope = (T1degCx8 - T0degCx8)/((T1out-T0out)*8);        //Slope of temperature interpolation line
    gblinfo.TempInt = (T0degCx8/8) - (gblinfo.TempSlope*T0out);         //Intercept of temperature interpolation line
    
    
    /* DERIVE LINE EQUATIONS FOR HUMIDITY SENSOR */
    H0rhx2 = I2CRead(HumBaseAddr,HumH0rhx2_rw);
    H1rhx2 = I2CRead(HumBaseAddr,HumH1rhx2_rw);
    
    I2Cdata = I2CRead(HumBaseAddr,HumH0T0Hi_rw);
    H0T0out = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr,HumH0T0Lo_rw);
    H0T0out = (uint16_t)(H0T0out | I2Cdata);
    
    I2Cdata = I2CRead(HumBaseAddr,HumH1T0Hi_rw);
    H1T0out = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr,HumH1T0Lo_rw);
    H1T0out = (uint16_t)(H0T0out | I2Cdata);
    
    gblinfo.HumSlope = (H1rhx2 - H0rhx2)/((H1T0out - H0T0out) * 8);  //Slope for humidity transfer function 
    gblinfo.HumInt = (H0rhx2/2) - (gblinfo.HumSlope * H0T0out);         //Intercept for humidity transfer function
}

void HandleNewAccelData( void ) {  //>>  TODO need to cap the number at 14bits, or if above report 0b AA11-1111-1111-1111
    uint16_t I2Cdata = 0;
    uint16_t AccelDat = 0;
    
    I2Cdata = I2CRead(AccBaseAddr, AccXHi_r);       //Get the X-axis data from the accelerometer
    AccelDat = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(AccBaseAddr, AccXLo_r);
    AccelDat = (uint16_t)(AccelDat | I2Cdata);
    
    //if (AccelDat >= 32768){                         //Just looking for the absolute value so convert to positive magnitude
    //    AccelDat += 32768;
    //}
    
    gblinfo.AccelXAvg100ms = (uint16_t)((AccelDat * (1-AVG100MS_ACCEL)) + (gblinfo.AccelXAvg100ms * AVG100MS_ACCEL));            //Update X 100ms rolling average (approximation)
    gblinfo.AccelXAvg500ms = (uint16_t)((AccelDat * (1-AVG500MS_ACCEL)) + (gblinfo.AccelXAvg500ms * AVG500MS_ACCEL));            //Update X 500ms rolling average (approximation)
    gblinfo.AccelXAvg1000ms = (uint16_t)((AccelDat * (1-AVG1000MS_ACCEL)) + (gblinfo.AccelXAvg1000ms * AVG1000MS_ACCEL));        //Update X 1000ms rolling average (approximation)
    
    I2Cdata = I2CRead(AccBaseAddr, AccYHi_r);       //Get the Y-axis data from the accelerometer
    AccelDat = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(AccBaseAddr, AccYLo_r);
    AccelDat = (uint16_t)(AccelDat | I2Cdata);
    
    if (AccelDat >= 32768){                         //Just looking for the absolute value so convert to positive magnitude
        AccelDat += 32768;                          
    }
    
    gblinfo.AccelYAvg100ms = (uint16_t)((AccelDat * (1-AVG100MS_ACCEL)) + (gblinfo.AccelYAvg100ms * AVG100MS_ACCEL));            //Update Y 100ms rolling average (approximation)
    gblinfo.AccelYAvg500ms = (uint16_t)((AccelDat * (1-AVG500MS_ACCEL)) + (gblinfo.AccelYAvg500ms * AVG500MS_ACCEL));            //Update Y 500ms rolling average (approximation)
    gblinfo.AccelYAvg1000ms = (uint16_t)((AccelDat * (1-AVG1000MS_ACCEL)) + (gblinfo.AccelYAvg1000ms * AVG1000MS_ACCEL));        //Update Y 1000ms rolling average (approximation)
    
    I2Cdata = I2CRead(AccBaseAddr, AccZHi_r);       //Get the Z-axis data from the accelerometer
    AccelDat = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(AccBaseAddr, AccZLo_r);
    AccelDat = (uint16_t)(AccelDat | I2Cdata);
    
    if (AccelDat >= 32768){                         //Just looking for the absolute value so convert to positive magnitude
        AccelDat += 32768;                          
    }
    
    gblinfo.AccelZAvg100ms = (uint16_t)((AccelDat * (1-AVG100MS_ACCEL)) + (gblinfo.AccelZAvg100ms * AVG100MS_ACCEL));            //Update Z 100ms rolling average (approximation)
    gblinfo.AccelZAvg500ms = (uint16_t)((AccelDat * (1-AVG500MS_ACCEL)) + (gblinfo.AccelZAvg500ms * AVG500MS_ACCEL));            //Update Z 500ms rolling average (approximation)
    gblinfo.AccelZAvg1000ms = (uint16_t)((AccelDat * (1-AVG1000MS_ACCEL)) + (gblinfo.AccelZAvg1000ms * AVG1000MS_ACCEL));        //Update Z 1000ms rolling average (approximation)
    
    if(gblinfo.AccelXAvg100ms > gblinfo.AccelYAvg100ms && gblinfo.AccelXAvg100ms > gblinfo.AccelZAvg100ms)              //For the 100ms rolling average, determine which axis has the largest magnitude
        gblinfo.MaxAmplitude100ms = X_AXIS;
    else if(gblinfo.AccelXAvg100ms > gblinfo.AccelYAvg100ms && gblinfo.AccelXAvg100ms < gblinfo.AccelZAvg100ms)
        gblinfo.MaxAmplitude100ms = Z_AXIS;
    else 
        gblinfo.MaxAmplitude100ms = Y_AXIS;
        
    if(gblinfo.AccelXAvg500ms > gblinfo.AccelYAvg500ms && gblinfo.AccelXAvg500ms > gblinfo.AccelZAvg500ms)              //For the 500ms rolling average, determine which axis has the largest magnitude
        gblinfo.MaxAmplitude500ms = X_AXIS;
    else if(gblinfo.AccelXAvg500ms > gblinfo.AccelYAvg500ms && gblinfo.AccelXAvg500ms < gblinfo.AccelZAvg500ms)
        gblinfo.MaxAmplitude500ms = Z_AXIS;
    else 
        gblinfo.MaxAmplitude500ms = Y_AXIS;
        
    if(gblinfo.AccelXAvg1000ms > gblinfo.AccelYAvg1000ms && gblinfo.AccelXAvg1000ms > gblinfo.AccelZAvg1000ms)          //For the 1000ms rolling average, determine which axis has the largest magnitude
        gblinfo.MaxAmplitude1000ms = X_AXIS;
    else if(gblinfo.AccelXAvg1000ms > gblinfo.AccelYAvg1000ms && gblinfo.AccelXAvg1000ms < gblinfo.AccelZAvg1000ms)
        gblinfo.MaxAmplitude1000ms = Z_AXIS;
    else 
        gblinfo.MaxAmplitude1000ms = Y_AXIS;
    
    
}

void BcastAccelRollingAverage( void ){  //TODO need to put code here

}

void ReportLevelData( void ) {      //TODO add code 

}

void StoreCalibrationValues(void) {
    //Store the unique calibration identifier passed down from above
    EEPROMWriteByte(RXB0D0, CALID);
    
    //Store X-axis data
    EEPROMWriteByte(RXB0D1,XMSB);
    EEPROMWriteByte(RXB0D2,XLSB);
    
    //Store Y-axis data
    EEPROMWriteByte(RXB0D3,YMSB);
    EEPROMWriteByte(RXB0D4,YLSB);
    
    //Store Z-axis data 
    EEPROMWriteByte(RXB0D5,ZMSB);
    EEPROMWriteByte(RXB0D6,ZLSB);
    
}

void GrabHWid(void) {
    can_id_t id;
    id.src_addr = (uint8_t)((b3 << 3) | (b2 << 2) | (b1 << 1) | b0);  //Hardware address of this module
}


void AccelTest(void) {              //>> TODO  This is only in for bring up! 
    uint8_t I2CdataL = 0x55;
    uint8_t I2CdataH = 0x55;
    int16_t AccelDat = 0;
    
    I2CdataL = I2CRead(AccBaseAddr, AccXLo_r);
    I2CdataH = I2CRead(AccBaseAddr, AccXHi_r);
    AccelDat = (uint16_t)((I2CdataH << 8) | I2CdataL);
    
    
    I2CdataL = I2CRead(AccBaseAddr, AccYLo_r);
    I2CdataH = I2CRead(AccBaseAddr, AccYHi_r);
    AccelDat = (uint16_t)((I2CdataH << 8) | I2CdataL);
    
    I2CdataL = I2CRead(AccBaseAddr, AccZLo_r);
    I2CdataH = I2CRead(AccBaseAddr, AccZHi_r);
    AccelDat = (uint16_t)((I2CdataH << 8) | I2CdataL);
    
    if(AccelDat >= 57365){                            // >= 90% of 1G (16383*.9)
        REDLED = YELLED = GRNLED = ledon;
    }
    else if(AccelDat >= 7372 && AccelDat <= 9010){    //Between 45 and 55% of full scale
        REDLED = YELLED = ledon;
        GRNLED = ledoff;
    }
    else if(AccelDat >= 4095 && AccelDat <= 5734){    //Between 25% and 35% of full scale
        //REDLED = ledon;
        YELLED = GRNLED = ledoff;
    }
    else
        REDLED = YELLED = GRNLED = ledoff;          //Below 25%
    
    //TestCanSend();      //Test sending a can message
    
    
    //GRNLED = ~GRNLED;
    I2CdataL=I2CRead(AccBaseAddr,AccIDReg_r);        //
    if(I2CdataL == 0x33)
        HBLED = ~HBLED;
}

void tick100mDelay( uint16_t tick100ms ) {
    uint16_t i = 0;
    uint16_t tick = 0;  //Used to lock time value
    for(i = tick100ms; i > 0; i--) {
        tick = gblinfo.tick100m;
        while (tick == gblinfo.tick100m);  //Wait for time to wrap around (in one half tick1000mond)
    }
}

void tick1mDelay( uint16_t tick1ms ) {
    uint16_t i = 0;
    uint16_t tick = 0;  //Used to lock time value
    for(i = tick1ms; i > 0; i--) {
        tick = gblinfo.tick1m;
        while (tick == gblinfo.tick1m);  //Wait for time to wrap around (in one half tick1000mond)
    }

}
/* END OF FILE */