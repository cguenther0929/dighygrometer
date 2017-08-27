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
*   TODO:  
*
*   NOTE:
*
******************************************************************************/

#include "main.h" //Include header file associated with main.c


// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Disabled - Controlled by SRETEN bit)
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
// #pragma config INTOSCSEL = LOW  // LF-INTOSC Low-power Enable bit (LF-INTOSC in Low-power mode during Sleep)
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
// #pragma config WDTEN = ON       // Watchdog Timer (WDT controlled by SWDTEN bit setting)
#pragma config WDTEN = OFF       // Watchdog Timer (WDT controlled by SWDTEN bit setting)
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

struct GlobalInformation gblinfo;

void main()
{
    uint16_t i; 
    SetUp();                    //Initialize hardware

    for(i=0;i<5000;i++);        //Hardware delay for things to stabilize
    DisplayON();
    
    /* BEGIN SUPER LOOP */
    while (true)
    {
        UpdateTH();             //Report temp and humidity data
        ClearDisp();
        CursorHome();
        PrintUnsignedDecimal(gblinfo.int_temp_val); 
        DispSendString("F ");
        
        PrintUnsignedDecimal(gblinfo.int_hum_val);
        DispSendChar('%',false);
        tick100mDelay(20);
        
        ClearDisp(); CursorHome();
        DispSendString("BV "); PrintUnsignedDecimal(BaudValue);
        tick100mDelay(20);
        
        /* THE FOLLOWING IS THE BATTERY TEST */
        // BatteryStatus();        //Determine Battery Voltage
        // if(gblinfo.bat_low == true) {
        //     ClearDisp();
        //     CursorHome();
        //     PrintFloat(gblinfo.battery_voltage);
        //     DispSendChar('*',false);
        //     tick100mDelay(10);
        // }

        // DisplayOFF();                   //Sleep display for ultra low power draw
        
        // gblinfo.wakeedge = false;       //Reset this flag
        // while(!gblinfo.wakeedge);       //Wait for rising edge on interrupt pin
        // gblinfo.wakeedge = false;       //Reset this flag

        // DisplayON();                    //Dispaly on so we can print out info

    }                           //END while(1) SUPER LOOP
} //END Main()

void SetUp(void)
{
    uint16_t i;

    INTSRC = 0;                             //Set appropriate bits to select the 500kHz internal timer
    MFIOSEL = 1;
    IRCF2 = 0; IRCF1 = 1; IRCF0 = 0;
    
    for(i=0;i<500;i++);     //Give the clock some time to stablize
    for(i=0;i<500;i++);     //Give the clock some time to stablize
    for(i=0;i<500;i++);     //Give the clock some time to stablize

    /* PIN DIRECTION FOR DBG GPIO */
    TRISB5 = output; //Port pin attached to the Heartbeat LED

    /* PIN DIRECTIONS FOR I2C */
    TRISC3 = input; //I2C SCK. Must be defined as input
    TRISC4 = input; //I2C SDA. Must be defined as input

    /* PIN DIRECTION FOR POWER CONTROL SIGNALS */   
    DISP_PWR_EN_n = 1;                  //Display off by default 

    /* PIN DIRECTIONS FOR LCD */
    //Defined in display init function since function that removes power from display sets these to HI-Z each time
   
    gblinfo.wakeedge = false;    
    
    PORTBINTSetup(1, true, false);         //Turn on INT1, rising_edge = true, high priority = false (has no affect on channel 0, anyhow)

    Init_Interrupts();                  //Set up interrupts  
    
    AnalogRefSel(REF2D048, EXTREF); //User internal 2.048V reference and External VREF pin for negative reference -- page 216/380
    InitA2D(1, 16, 0);              //Set up AD (Justification, Acq Time (TAD), Prescaler) ==> (Right, 16 TAD, RC Oscillator) -- page 361/550

    /* DIGITAL PULSE INITIAL VALUE */
    PULSEOUT = 0;

    gblinfo.tick10ms = 0;       //Initialize 100s of a tick1000mond counter to 0
    gblinfo.tick100ms = 0;      //Initialize 100s of a tick1000mond counter to 0
    gblinfo.tick1000ms = 0;     //Seconds counter

    /* DISABLE ANALOG CHANNELS */
    ANCON0 = 0x00; //Analog channels 7-0 are configured for digital inputs. p.363     
    ANCON1 = 0x00; //Analog channel 10-8 are configred for digital inputs. p.364
    EnableAnalogCh(0);

    Timer0Init(1, 1, 0); //ARGS: interrupts = yes, prescaler = 1, clksource = FOSC/4 (8MHz / 4 in this application)
    Timer0On();

    /* I2C START UP*/   
    I2Cinit();
    tick100mDelay(2);

    /* TEMP/HUMIDITY SENSOR CONFIG*/   
    I2CWrite_16(THBaseAddr, THConfigReg, THConfigVal);  //See config.h to understand THConfigVal
    tick100mDelay(1);

}

void UpdateTH(void) {
    uint32_t SensorData;
    uint16_t TemperatureInt;
    uint16_t HumIntVal;
    float TempFloatVal;
    float HumFloatVal;
    uint8_t I2CData;

    I2CWrite_SetPointer(THBaseAddr,THValuePointer);
    tick10msDelay(1);                                           //Per page 5/30 of the HDC1080 datasheet, conversion time for 14bits is 6.5ms
    SensorData = I2CRead(THBaseAddr);                           //Reads MSB Temp | LSB Temp | MSB Hum | LSB Hum
    
    TemperatureInt = (uint16_t)((SensorData >> 16) & 0xFFFF);
    HumIntVal = (uint16_t)(SensorData & 0xFFFF);

    TempFloatVal = (float)((TemperatureInt/397.187878) - 40);       //Per HDC1080 datasheet page 14 of 30
    gblinfo.int_temp_val = (uint8_t)(TempFloatVal);

    HumFloatVal = (float)(HumIntVal/655.36);                      //Per HDC1080 datasheet page 14/30.  Value in %RH
    gblinfo.int_hum_val = (uint8_t)(HumFloatVal);
}

void BatteryStatus( void ) {
    uint16_t adcreading = 0;
    float Tempval = 0;
    
    adcreading = ReadA2D(0,1);                          //Battery analog sense is on channel AN0
    Tempval = (float)(adcreading*(0.000816456)); 		//Convert bits to volts. Internal Reference is 2.048 
    
    gblinfo.battery_voltage = Tempval;
    (Tempval <= BAT_VOLT_MIN) ? (gblinfo.bat_low = true):(gblinfo.bat_low = false);

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