/******************************************************************************
*   FILE: i2c.c
*
*   PURPOSE: Source file which contains all I2C-related routines.  
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

#include "i2c.h"			//Pull in the bus header file

void I2Cinit( void ) {
	/* SET ALL THE CORRECT VALUES IN THE I2C CON REGISTER */
	SMP = 1;    //Slew rate control bit (0=Slew Rate Control enabled for high speed mode; 1=Disabled for standard speed)	
    WCOL = 0;   //Write collision detect bit.  Must be cleared in SW.
    SSPOV = 0;  //Receiver overflow indicator bit. Must be cleared in SW.
	SSPCON1bits.SSPM = 8;   //I2C Master Mode. CLOCK RATE = FOSC/(4*(SSPADD+1)) p.298
	SSPEN = 1;	//Enable the I2C module and configure the SDA and SCL Pins as serial port pins
    
    /* LOAD THE BAUD RATE GENERATOR WITH THE APPROPIATE VALUE */
	SSPADD = BaudValue;      //In master-mode, this is the BAUD rate reload value. (p.298) //TODO this is the line we want in
    //SSPADD = 159;      //In master-mode, this is the BAUD rate reload value. (p.298)
}

void I2CWrite(uint8_t baseaddress, uint8_t subaddress, uint8_t senddata) {  //TODO this function needs an overhaul 
	uint8_t tempaddr = 0x00;		//Use this as a way to and the R/W bit with the I2CADDR that can be found in the header file
    
    WCOL = 0;                           //Write collision detect bit.  Must be cleared in SW.
    SSPOV = 0;                          //Receiver overflow indicator bit. Must be cleared in
	
	tempaddr = (uint8_t)((baseaddress << 1) | I2CWRITE);		//LSP is Read/n_Write bit
    //tempaddr = 0x32; //(uint8_t)((baseaddress << 1) | I2CWRITE);		//LSP is Read/n_Write bit  TODO clean up this routine !!

    while((SSPCON2 & 0x1F) >= 1);       //Checking if ACKEN, RCEN, PEN, RSEN, or SEN is 1
    
    
    I2CGenStart = 1; 			        //Generate the start condition
    if (WCOL == 1){             //Bus collision detected (p.320)
        REDLED = ledon;     //TODO: This is in for debugging only
        WCOL=0;
        //return;
    }
    
    while(I2CGenStart == 1);        //Bit will automatically get cleared in HW
    
    SSPBUF = tempaddr;			//Load the address and r/w bits into the transmit buffer

    if (WCOL == 1){             //Bus collision detected (p.320)
        REDLED = ledon;             //TODO: This is in for debugging only
        WCOL=0;
        //return;
    }

    //while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    
    if(I2CACKStat == NACK){     //Slave did not acknowledge transmission of base address 
        GRNLED = ledon;                 //TODO: This is in for debugging only
        //return;
    }

    SSPBUF = subaddress;		    //Send the sub address to the slave
    
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    
    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        YELLED = ledon;                 //TODO: This is in for debugging only
        //return;
    }
	
	/* SEND DATA TO SLAVE */
    SSPBUF = senddata;		        //Send the sub address to the slave
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data

    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        REDLED = ledon;             //TODO: This is in for debugging only
        //return;
    }
	
    I2CGenStop = 1;					//Create the STOP condition on the bus
    while(I2CGenStop == 1);         //Bit automatically cleared in HW
}

uint8_t I2CRead(uint8_t baseaddress, uint8_t subaddress) {
    uint8_t tempaddr = 0x00;		//Use this as a way to and the R/W bit with the I2CADDR that can be found in the header file
	uint8_t rtndata = 0x00;		//This will be the returned 16 bit number
    uint16_t i = 0;
	
	rtndata = SSPBUF;	//Read this so we clar all buffer full flags (if set)
    
    SMP = 1;    //Slew rate control bit (0=Slew Rate Control enabled for high speed mode; 1=Disabled for standard speed)	
    WCOL = 0;   //Write collision detect bit.  Must be cleared in SW.
    SSPOV = 0;  //Receiver overflow indicator bit. Must be cleared in SW.
	SSPCON1bits.SSPM = 8;   //I2C Master Mode. CLOCK RATE = FOSC/(4*(SSPADD+1)) p.298
	SSPEN = 1;	//Enable the I2C module and configure the SDA and SCL Pins as serial port pins
    
    /* SSPBUF HOUSE CLEANING */
	for(i=0;i<i2cdelay;i++);
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    WCOL = 0;   //Write collision detect bit.  Must be cleared in SW.
    SSPOV = 0;  //Receiver overflow indicator bit. Must be cleared in SW.
	/* SSPBUF HOUSE CLEANING */
	
    tempaddr = (uint8_t)((baseaddress << 1) | I2CWRITE);		//LSB is Read/n_Write bit
    
    /*For transmit sequence, see page 317 of datasheet*/

    while((SSPCON2 & 0x1F) >= 1);       //Checking if ACKEN, RCEN, PEN, RSEN, or SEN is 1
    
    I2CGenStart = 1; 			        //Generate the start condition
    while(I2CGenStart == 1);        //Bit will automatically get cleared in HW
    
    if (WCOL == 1){                     //Bus collision detected (p.320)
        REDLED = ledon;                 //TODO: This is in for debugging only
        WCOL=0;
        //return;
    }
    

    SSPBUF = tempaddr;			//First send SAD + W
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    
    if (WCOL == 1){             //Bus collision detected (p.320)
        REDLED = ledon;             //TODO: This is in for debugging only
        WCOL=0;
        //return;
    }

        
    if(I2CACKStat == NACK){     //Slave did not acknowledge transmission of base address 
        YELLED = ledon;                 //TODO: This is in for debugging only
        //return;
    }

	/* SSPBUF HOUSE CLEANING */
	for(i=0;i<i2cdelay;i++);
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    WCOL = 0;   //Write collision detect bit.  Must be cleared in SW.
    SSPOV = 0;  //Receiver overflow indicator bit. Must be cleared in SW.
	/* SSPBUF HOUSE CLEANING */
    
	SSPBUF = subaddress;		//Send the sub address to the slave

    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    
    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        GRNLED = ledon;                 //TODO: This is in for debugging only
        //return;
    }
    
    tempaddr = (uint8_t)((baseaddress << 1) | I2CREAD);		//LSB is Read/n_Write bit


    I2CRepStart = 1;            //Generate repeated start condition 
    while(I2CRepStart == 1);            //Bit automatically cleared in HW
    
    if (WCOL == 1){             //Bus collision detected (p.320)
        REDLED = ledon;         //TODO: This is in for debugging only
        WCOL=0;
        //return;
    }
	
	/* SSPBUF HOUSE CLEANING */
	for(i=0;i<i2cdelay;i++);
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
	for(i=0;i<i2cdelay;i++);
    WCOL = 0;   //Write collision detect bit.  Must be cleared in SW.
    SSPOV = 0;  //Receiver overflow indicator bit. Must be cleared in SW.
	/* SSPBUF HOUSE CLEANING */
    
    
    SSPBUF = tempaddr;
    while(I2CTXBusy == 1);		//Wait for the 8 clock cycles to transmit the data
    
    if(I2CACKStat == NACK){        //Slave did not acknowledge transmission of base address 
        REDLED = ledon;                 //TODO: This is in for debugging only
        //return;
    }
	
	/* SSPBUF HOUSE CLEANING */
	for(i=0;i<i2cdelay;i++);
    rtndata = SSPBUF;                   //Read from SSPBUF to "clean"
    for(i=0;i<i2cdelay;i++);
    WCOL = 0;   //Write collision detect bit.  Must be cleared in SW.
    SSPOV = 0;  //Receiver overflow indicator bit. Must be cleared in SW.
	/* SSPBUF HOUSE CLEANING */

    i = 0;
    while(MSSP_Active){
        i++;
        if(i >= 50000)
            break;
    }
                				//Receive enable bit ignored if the module is active!!
	
    I2CRecEnable = 1;               //Enable I2C Receiver
    while(I2CRecEnable == 1);
    //while(I2CBF != 1);       		//Wait for the buffer to be full
    //for(i=0;i<50000;i++);
    //rtndata = SSPBUF;               //Should automatically clear BF flag (defined by I2CBF)
    
    I2CACKBit = NACK;               //Set the ACK/NACK bit to ACK
    SSPIF = 0;
    I2CGenACK = 1;                  //Assert acknowledge on I2C bus for slave to see
    //while(SSPIF == 0){}
    //while(I2CGenACK == 1);        //Wait for ACK to complete
    //for(i=0;i<i2cdelay;i++);
    
    rtndata = SSPBUF;               //Should automatically clear BF flag (defined by I2CBF)
    
    I2CGenStop = 1;					//Create the STOP condition on the bus
    while(I2CGenStop == 1);         //Bit automatically cleared in HW
    SSPEN = 0;	                //This and the two lines below forces the PIC to release the bus.  TODO.  Don't feel like this should be required.  
    TRISC3 = input;                     //I2C SCK. Must be defined as input
    TRISC4 = input;                     //I2C SDA. Must be defined as input
     
    return(rtndata);               //Return data in form xxxxxx(b9)(b8)(b7)(b6)(b5)(b4)(b3)(b2)(b1)(b0)
}