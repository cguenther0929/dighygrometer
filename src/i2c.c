/******************************************************************************
*   FILE: i2c.c
*
*   PURPOSE: Source file which contains all I2C-related routines.  
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

#include "i2c.h"			//Pull in the bus header file

void I2Cinit( void ) {
	/* SET ALL THE CORRECT VALUES IN THE I2C CON REGISTER */
	SMP1 = 1;    //Slew rate control bit (0=Slew Rate Control enabled for high speed mode; 1=Disabled for standard speed)
    WCOL1 = 0;   //Write collision detect bit.  Must be cleared in SW.
    SSPOV1 = 0;  //Receiver overflow indicator bit. Must be cleared in SW.
	SSPCON1bits.SSPM = 8;   //I2C Master Mode. CLOCK RATE = FOSC/(4*(SSPADD+1)) p.298
	SSPCON1bits.SSPEN = 1;	//Enable the I2C module and configure the SDA and SCL Pins as serial port pins
    
    /* LOAD THE BAUD RATE GENERATOR WITH THE APPROPIATE VALUE */
	SSPADD = BaudValue;      //In master-mode, this is the BAUD rate reload value. (p.298) 
}

void I2CWrite(uint8_t baseaddress, uint8_t subaddress, uint8_t senddata) {  
	uint8_t tempaddr = 0;		//Use this as a way to and the R/W bit with the I2CADDR that can be found in the header file
    uint16_t i = 0;
    uint8_t rtndata = 0;		//This will be the returned 16 bit number
    
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
	
	tempaddr = (uint8_t)((baseaddress << 1) | I2CWRITE);		//LSP is Read/n_Write bit

    while((SSPCON2 & 0x1F) >= 1);       //Checking if ACKEN, RCEN, PEN, RSEN, or SEN is 1
        
    I2CGenStart = 1; 			        //Generate the start condition
    while(I2CGenStart == 1);        //Bit will automatically get cleared in HW
    if (WCOL1 == 1){             //Bus collision detected (p.320)
        WCOL1=0;
        return;
    }
    
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);

    SSPBUF = tempaddr;			//Load the address and r/w bits into the transmit buffer

    if (WCOL1 == 1){             //Bus collision detected (p.320)
        WCOL1=0;
        return;
    }

    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    
    if(I2CACKStat == NACK){     //Slave did not acknowledge transmission of base address 
        return;
    }
    
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);

    SSPBUF = subaddress;		    //Send the sub address to the slave
    
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    
    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        return;
    }
	
	/* SEND DATA TO SLAVE */
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    
    SSPBUF = senddata;		        //Send the sub address to the slave
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data

    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        return;
    }
    
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    
    while((SSPCON2 & 0x1F) >= 1);       //Checking if ACKEN, RCEN, PEN, RSEN, or SEN is 1
	
    I2CGenStop = 1;					//Create the STOP condition on the bus
    while(I2CGenStop == 1);         //Bit automatically cleared in HW
}

uint8_t I2CRead(uint8_t baseaddress, uint8_t subaddress) {
    uint8_t tempaddr = 0;		//Use this as a way to and the R/W bit with the I2CADDR that can be found in the header file
	uint8_t rtndata = 0;		//This will be the returned 16 bit number
    uint16_t i = 0;

    WCOL1 = 0;                           //Write collision detect bit.  Must be cleared in SW.
    SSPOV1 = 0;                          //Receiver overflow indicator bit. Must be cleared in SW.
    
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
	
    tempaddr = (uint8_t)((baseaddress << 1) | I2CWRITE);		//LSB is Read/n_Write bit
    
    while((SSPCON2 & 0x1F) >= 1);       //Checking if ACKEN, RCEN, PEN, RSEN, or SEN is 1 aka is MSSP active?
    
    SEN1 = 1; 			        //Generate the start condition
    while(SEN1 == 1);        //Bit will automatically get cleared in HW
    
    if (WCOL1 == 1){                     //Bus collision detected (p.320)
        WCOL1=0;
        return 0;
    }

    SSPBUF = tempaddr;			//First send SAD + W
    
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    for(i=0;i<i2cdelay;i++);    //Some delay for safety
    
    if (WCOL1 == 1){             //Bus collision detected (p.320)
        WCOL1=0;
        return 0;
    }
        
    if(I2CACKStat == NACK){     //Slave did not acknowledge transmission of base address 
        return 0;
    }

    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    
	SSPBUF = subaddress;		//Send the sub address to the slave

    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    for(i=0;i<i2cdelay;i++);    //Some delay to allow for ACK/NACK bit
    
    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        return 0;
    }
    
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    
    tempaddr = (uint8_t)((baseaddress << 1) | I2CREAD);		//LSB is Read/n_Write bit
    while((SSPCON2 & 0x1F) >= 1);       //Checking if ACKEN, RCEN, PEN, RSEN, or SEN is 1 aka is MSSP active?

    I2CRepStart = 1;                    //Generate repeated start condition 
    while(I2CRepStart == 1);            //Bit automatically cleared in HW
    for(i=0;i<i2cdelay;i++);            //Some delay to allow for collision detect 
    
    
    if (WCOL1 == 1){             //Bus collision detected (p.320)
        WCOL1=0;
        return 0;
    }
	
    SSPBUF = tempaddr;
    while(I2CTXBusy == 1){}		//Wait for the 8 clock cycles to transmit the data
    for(i=0;i<i2cdelay;i++);    //Some delay for ACK/NACK setup
    
    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        return 0;
    }
	
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);

    while((SSPCON2 & 0x1F) >= 1);       //Receive enable bit ignored if the module is active!!
	
    I2CRecEnable = 1;               //Enable I2C Receiver
    while(I2CRecEnable == 1){}
    
    I2CACKBit = NACK;               //Set the ACK/NACK bit to NACK to end transmission
    SSPIF = 0;
    I2CGenACK = 1;                  //Assert acknowledge on I2C bus for slave to see
	for(i=0;i<i2cdelay;i++);
    
    i=0;
    while(I2CGenACK == 1){
        i++;
        if(i==10)
            break;
    }
 
    rtndata = SSPBUF;               //Should automatically clear BF flag (defined by I2CBF)

    I2CRecEnable = 0;
    
    SSPIF = 0;
    I2CGenStop = 1;					//Create the STOP condition on the bus
	for(i=0;i<i2cdelay;i++);
    
    i=0;
    while(SSPIF == 0){
        i++;
        if(i==10)
            break;
    }
    
    return (rtndata);               //Return data in form xxxxxx(b9)(b8)(b7)(b6)(b5)(b4)(b3)(b2)(b1)(b0)
}