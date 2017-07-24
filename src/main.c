/******************************************************************************
*   FILE: main.c
*
*   PURPOSE: Main source
*
*   DEVICE: PIC18F66K22
*
*   COMPILER: Microchip XC8 v1.32
*
*   IDE: MPLAB X v3.45
*
*   TODO:  TODO need to setup INT 1 for HUM_DRDY
*
*   NOTE:
*
******************************************************************************/

#include "main.h" //Include header file associated with main.c


// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Disabled - Controlled by SRETEN bit)
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = INTIO1    // Oscillator (Internal RC oscillator) with output on OSC2.  p45/550.  default IRCF<2:0> = 110 = 8MHz (on pin 8MHz/4 = 2MHz)
#pragma config PLLCFG = OFF     // PLL x4 Enable bit (Disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power Up Timer (Enabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 1         // Brown-out Reset Voltage bits (2.7V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer (WDT controlled by SWDTEN bit setting)
#pragma config WDTPS = 1024     // Watchdog Postscaler (1:1024)

// CONFIG3L
#pragma config RTCOSC = SOSCREF // RTCC Clock Select (RTCC uses SOSC)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 Mux (RC1)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RG5 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-03FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 04000-07FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 08000-0BFFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 0C000-0FFFF (Disabled)

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
#pragma config EBRT0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBRT1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBRT2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBRT3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBRTB = OFF      // Table Read Protect Boot (Disabled)

// //OLD starts here
// // CONFIG1L
// #pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
// #pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
// #pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
// #pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// // CONFIG1H
// #pragma config FOSC = INTIO1 // Oscillator (Internal RC oscillator, CLKOUT function on OSC2). p45/550.  default IRCF<2:0> = 110 = 8MHz
// #pragma config PLLCFG = OFF  // PLL x4 Enable bit (Disabled).  We do not need to increase clock speed
// #pragma config FCMEN = ON    // Fail-Safe Clock Monitor (Enabled)
// #pragma config IESO = ON     // Internal External Oscillator Switch Over Mode (Enabled)

// // CONFIG2L
// #pragma config PWRTEN = ON      // Power Up Timer (Enabled)
// #pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
// #pragma config BORV = 1         // Brown-out Reset Voltage bits (2.7V)
// #pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// // CONFIG2H
// #pragma config WDTEN = ON   // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
// #pragma config WDTPS = 1024 // Watchdog Postscaler (1:1024)

// // CONFIG3H
// #pragma config CANMX = PORTC  // ECAN Mux bit (ECAN TX and RX pins are located on RC6 and RC7, respectively)
// #pragma config MSSPMSK = MSK7 // MSSP address masking (7 Bit address masking mode)
// #pragma config MCLRE = ON     // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// // CONFIG4L
// #pragma config STVREN = ON  // Stack Overflow Reset (Enabled)
// #pragma config BBSIZ = BB2K // Boot Block Size (2K word Boot Block size)

// // CONFIG5L
// #pragma config CP0 = OFF // Code Protect 00800-01FFF (Disabled)
// #pragma config CP1 = OFF // Code Protect 02000-03FFF (Disabled)
// #pragma config CP2 = OFF // Code Protect 04000-05FFF (Disabled)
// #pragma config CP3 = OFF // Code Protect 06000-07FFF (Disabled)

// // CONFIG5H
// #pragma config CPB = OFF // Code Protect Boot (Disabled)
// #pragma config CPD = OFF // Data EE Read Protect (Disabled)

// // CONFIG6L
// #pragma config WRT0 = OFF // Table Write Protect 00800-03FFF (Disabled)
// #pragma config WRT1 = OFF // Table Write Protect 04000-07FFF (Disabled)
// #pragma config WRT2 = OFF // Table Write Protect 08000-0BFFF (Disabled)
// #pragma config WRT3 = OFF // Table Write Protect 0C000-0FFFF (Disabled)

// // CONFIG6H
// #pragma config WRTC = OFF // Config. Write Protect (Disabled)
// #pragma config WRTB = OFF // Table Write Protect Boot (Disabled)
// #pragma config WRTD = OFF // Data EE Write Protect (Disabled)

// // CONFIG7L
// #pragma config EBTR0 = OFF // Table Read Protect 00800-03FFF (Disabled)
// #pragma config EBTR1 = OFF // Table Read Protect 04000-07FFF (Disabled)
// #pragma config EBTR2 = OFF // Table Read Protect 08000-0BFFF (Disabled)
// #pragma config EBTR3 = OFF // Table Read Protect 0C000-0FFFF (Disabled)

// // CONFIG7H
// #pragma config EBTRB = OFF // Table Read Protect Boot (Disabled)

struct GlobalInformation gblinfo;

void main()
{
    uint16_t i;      //TODO remove this line
    SetUp(); //Initialize hardware

    DISP_PWR_EN_n = 0;      //Turn the display on
    tick10msDelay(2);
    DisplayInit();
    
    /* BEGIN SUPER LOOP */
    while (1)
    {
        // UpdateTH();             //Report temp and humidity data
        // ClearDisp();
        // CursorHome();
        // PrintUnsignedDecimal(gblinfo.int_temp_val); 
        // DispSendString("F ");
        
        // PrintUnsignedDecimal(gblinfo.int_hum_val);
        // DispSendChar('%',false);
        // tick100mDelay(20);

        /* THE FOLLOWING IS THE BATTERY TEST */
        BatteryStatus();        //Determine Battery Voltage
        CursorHome();
        ClearDisp();
        PrintFloat(gblinfo.battery_voltage);
        tick100mDelay(20);

        // CLRWDT();            //Kick the dog   //TODO I don't believe we'll use the WDT, so remove this line?


        // UpdateDisplay();        //Diaply temperature and humidity values on display
    }                           //END while(1) SUPER LOOP
} //END Main()

void SetUp(void)
{
    uint16_t i;

    /* PIN DIRECTION FOR DBG GPIO */
    TRISB5 = output; //Port pin attached to the Heartbeat LED

    /* PIN DIRECTIONS FOR I2C */
    TRISC3 = input; //I2C SCK. Must be defined as input
    TRISC4 = input; //I2C SDA. Must be defined as input

    /* PIN DIRECTION FOR POWER CONTROL SIGNALS */
    TRISE7 = output; //Active low signal to enable display power
    TRISE6 = output; //Active high signal to kill stored energy from piezo activation
    TRISE5 = output; //Active high signal to latch MCU power on

    /* PIN DIRECTIONS FOR LCD */
    TRISD = output; //The entire port shall be an output  #TODO if this doesn't work, try TRISDbits = output;
    TRISE = output; //The entire port can be an output.
   
    DISP_PWR_EN_n = 1;                  //Display off by default //TODO we need corrections here, this just for testing
    // MCU_PWR_LATCH = 0;                  //Turn latch off
    MCU_PWR_LATCH = 1;                  //Turn latch ON
    // PIEZ_KILL = 1;                      //Make sure energy is out of Piezo element

    Init_Interrupts();                  //Set up interrupts  TODO not sure if we're going to require interrupts

    AnalogRefSel(REF2D048, EXTREF); //User internal 2.048V reference and External VREF pin for negative reference -- page 216/380
    InitA2D(1, 16, 0);              //Set up AD (Justification, Acq Time (TAD), Prescaler) ==> (Right, 16 TAD, RC Oscillator) -- page 361/550

    /* DIGITAL PULSE INITIAL VALUE */
    PULSEOUT = 0;

    gblinfo.tick10ms = 0;    //Initialize 100s of a tick1000mond counter to 0
    gblinfo.tick100ms = 0;  //Initialize 100s of a tick1000mond counter to 0
    gblinfo.tick1000ms = 0; //Seconds counter

    /* DISABLE ANALOG CHANNELS */
    ANCON0 = 0x00; //Analog channels 7-0 are configured for digital inputs. p.363     //TODO need to enable AN0
    ANCON1 = 0x00; //Analog channel 10-8 are configred for digital inputs. p.364
    EnableAnalogCh(0);

    Timer0Init(1, 64, 0); //ARGS: interrupts = yes, prescaler = 64, clksource = FOSC/4 (8MHz / 4 in this application)
    Timer0On();

    /* I2C START UP*/   //TODO Uncomment
    I2Cinit();
    tick100mDelay(2);

    // // /* TEMP/HUMIDITY SENSOR CONFIG*/   //TODO need to uncomment all of this stuff
    I2CWrite(HumBaseAddr, HumCtrl1_rw, 0x87); //[7]=Enable Device,[6:3]=Reserved,[2]=High and low register read before update,[1:0]=ODR
    tick100mDelay(1);

    I2CWrite(HumBaseAddr, HumAvConf_rw, 0x77); //[7:6]=Reserved,[5:3]=Temp Average,[2:0]=Humidity Average  -- Writing default values
    tick100mDelay(1);

    I2CWrite(HumBaseAddr, HumCtrl3_rw, 0x04); //[7]=DRDY Active State,[6]DRDY Push/Pull,[5:3]=Reserved,[2]=DRDY En,[1:0]=Reserved
    tick100mDelay(1);

    TempHumidityInitialize(); //Gather transfer function parameters from stored calibration values

    // SWDTEN = 1;     //SW enable WDT   TODO we want to enable this, or just remove it completely
}

void UpdateTH(void)
{
    int16_t SenseorData;
    int16_t TempIntVal;
    int16_t HumIntVal;
    float TempFloatVal;
    float HumFloatVal;
    uint8_t I2CData;

    /* GRAB TEMPERATURE DATA AND CONVERT TO UINT 8 */
    I2CData = I2CRead(HumBaseAddr, HumTmpHi_r);
    SenseorData = (uint16_t)((I2CData << 8) & 0xFF00);
    I2CData = I2CRead(HumBaseAddr, HumTmpLo_r);
    SenseorData = (uint16_t)(SenseorData | I2CData);

    TempFloatVal = (float)(gblinfo.TempSlope * SenseorData + gblinfo.TempInt);
    TempFloatVal = TempFloatVal * 1.8 + 32;               //Convert number to deg F

    gblinfo.int_temp_val = (uint8_t)(TempFloatVal);         // Convert to uint 8.  Number will never be negative

    /* GRAB HUMIDITY DATA AND CONVERT TO UINT 8 */
    I2CData = I2CRead(HumBaseAddr, HumHuHi_r);
    SenseorData = (uint16_t)((I2CData << 8) & 0xFF00);
    I2CData = I2CRead(HumBaseAddr, HumHuLo_r);
    SenseorData = (uint16_t)(SenseorData | I2CData);

    HumFloatVal = (float)(gblinfo.HumSlope * SenseorData + gblinfo.HumInt);

    gblinfo.int_hum_val = (uint8_t)(HumFloatVal);           // Convert to uint 8.  Number will never be negative
}

void TempHumidityInitialize(void)
{
    uint8_t T0degCx8Lo = 0;
    uint8_t T1degCx8Lo = 0;
    uint8_t T1T0degmsb = 0;
    uint16_t T0degCx8 = 0;
    uint16_t T1degCx8 = 0;

    int16_t T0out = 0;
    int16_t T1out = 0;
    uint8_t I2Cdata = 0;

    uint8_t H0rhx2 = 0;
    uint8_t H1rhx2 = 0;
    int16_t H0T0out = 0;
    int16_t H1T0out = 0;

    /* DERIVE LINE EQUATIONS FOR TEMPERATURE SENSOR */
    T0degCx8Lo = I2CRead(HumBaseAddr, HumT0DegCx8_rw);
    T1degCx8Lo = I2CRead(HumBaseAddr, HumT1DegCx8_rw);
    T1T0degmsb = I2CRead(HumBaseAddr, HumT0T1msb_rw);

    T0degCx8 = (uint16_t)(((T1T0degmsb & 0x03) << 8) | T0degCx8Lo);
    T1degCx8 = (uint16_t)((((T1T0degmsb >> 2) & 0x03) << 8) | T1degCx8Lo);

    I2Cdata = I2CRead(HumBaseAddr, HumT0Hi_rw);
    T0out = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr, HumT0Lo_rw);
    T0out = (uint16_t)(T0out | I2Cdata);

    I2Cdata = I2CRead(HumBaseAddr, HumT1Hi_rw);
    T1out = (uint16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr, HumT1Lo_rw);
    T1out = (uint16_t)(T1out | I2Cdata);

    gblinfo.TempSlope = (float)((T1degCx8 - T0degCx8) / ((T1out - T0out) * 8.00)); //Slope of temperature interpolation line
    gblinfo.TempInt = (float)((T0degCx8 / 8.00) - (gblinfo.TempSlope * T0out));    //Intercept of temperature interpolation line

    /* DERIVE LINE EQUATIONS FOR HUMIDITY SENSOR */
    H0rhx2 = I2CRead(HumBaseAddr, HumH0rhx2_rw);
    H1rhx2 = I2CRead(HumBaseAddr, HumH1rhx2_rw);

    I2Cdata = I2CRead(HumBaseAddr, HumH0T0Hi_rw);
    H0T0out = (int16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr, HumH0T0Lo_rw);
    H0T0out = (int16_t)(H0T0out | I2Cdata);

    I2Cdata = I2CRead(HumBaseAddr, HumH1T0Hi_rw);
    H1T0out = (int16_t)(I2Cdata << 8);
    I2Cdata = I2CRead(HumBaseAddr, HumH1T0Lo_rw);
    H1T0out = (int16_t)(H1T0out | I2Cdata);

    gblinfo.HumSlope = (float)((H1rhx2 - H0rhx2) / ((H1T0out - H0T0out) * 2.00)); //Slope for humidity transfer function
    gblinfo.HumInt = (float)((H0rhx2 / 2.00) - (gblinfo.HumSlope * H0T0out));     //Intercept for humidity transfer function
}

void BatteryStatus( void ) {
    uint16_t batvoltage = 0;
    
    batvoltage = ReadA2D(0,1);                          //Battery analog sense is on channel AN0
    batvoltage = (float)(batvoltage*(0.0005)); 		//Convert bits to volts. Showing all mathamatical steps here
    batvoltage *= 1.6329;                           //Scale up since BATSEN = BATV * (3.16e3/5.16e3)
    
    gblinfo.battery_voltage = batvoltage;
    (batvoltage <= BAT_VOLT_MIN) ? (gblinfo.bat_low = true):(gblinfo.bat_low = false);

}

void UpdateDisplay( void ) {
    
}

void tick100mDelay(uint16_t ticks)
{
    uint16_t i = 0;
    uint16_t tick = 0; //Used to lock time value
    for (i = ticks; i > 0; i--)
    {
        tick = gblinfo.tick100ms;
        while (tick == gblinfo.tick100ms)
            ; //Wait for time to wrap around (in one half tick1000mond)
    }
}

void tick10msDelay(uint16_t ticks)
{
    uint16_t i = 0;
    uint16_t tick = 0; //Used to lock time value
    for (i = ticks; i > 0; i--)
    {
        tick = gblinfo.tick10ms;
        while (tick == gblinfo.tick10ms); //Wait for time to wrap around (in one half tick1000mond)
    }
}
/* END OF FILE */