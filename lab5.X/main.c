/*
 * File:   main.c
 * Author: cal
 *
 * Created on November 21, 2014, 3:40 AM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/


#include <p32xxxx.h>
#include <plib.h>

//	Function Prototypes
int main(void);


char SPI_READ_STATUS_REGISTER = 0b00000101;
char SPI_WRITE_STATUS_REGISTER = 0b00000001;

char SPI_READ_DATA_MEMORY = 0b00000011;
char SPI_WRITE_DATA_MEMORY = 0b00000010;

char SPI_RESET_WRITE_ENABLE_LATCH = 0b00000100;
char SPI_SET_WRITE_ENABLE_LATCH = 0b00000110;

char SPI_WRITE_ENABLE_MASK = 0b00000010;

int WRITE_LENGTH = 63; //0-63 = 64 divisions
int READ_LENGTH = 63;
char DUMMY_DATA = 0xE5;
char TEST_DATA = 0x55;
char ADDR_MSA = 0x10;//testing things out with a read to the very first page (or last page)
char ADDR_LSA = 0x00;//lsb doesn't matter since we are doing reads on a page

#pragma config POSCMOD=XT, FNOSC=PRIPLL, FPLLIDIV=DIV_2, FPLLMUL=MUL_20, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_1, FWDTEN=OFF, CP=OFF, BWP=OFF
#pragma config FSOSCEN=OFF, IESO=OFF

int main(void) {
    SYSTEMConfig(80000000L, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //variable initialization

    int i = 0;
    int status = 0x55;
    int status2 = 0x55;
    char iBuff[64];
    char oBuff[64];

    for(i = 0; i < 64; i++)
    {
        oBuff[i] = i;
    }

    oBuff[0] = 0xFF;
//---Begin SPI configuration---
    //RB10 = CS (Chip Select), output, active low, initialize to 1 before setting to output
    
    LATBbits.LATB10 = 1; // initalize to 1
    TRISBbits.TRISB10 = 0; // set to output
    AD1PCFGbits.PCFG10 = 1; //turn off analog input control

    
    //using SPI 1, with CKE = 0, CKP = 1, SMP = 0,
    SPI1CONbits.CKE = 0;
    SPI1CONbits.CKP = 1;
    SPI1CONbits.SMP = 0;
    SPI1CONbits.MODE16 = 0;
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MSTEN = 1; // is this necessary?
    //and BRG for 220ns at 80MHZ system operation
    //FSCK = FPB / ( 2 x (BRG + 1))
    //SPI1BRG = 8; // 225ns at 80MHz operation
    SPI1BRG = 39; // 1MHz operation 

    SPI1CONbits.ON = 1;


    /*
    IEC0CLR = 0x03800000;
    SPI1CON = 0;
    SPI1BUF;
    SPI1BRG = 39;
    SPI1STATCLR = 0x40;
    SPI1CON = 0x8220;
    */




//---End SPI Configuration---

//---Begin SPI Read Status command--- //2 byte shifted out
    //communication pattern for read status
    //1. Assert CS.
    //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
    //3. Wait for TBE (transmitter buffer empty)
    //4. Write a dummy data byte to SPI1BUF (we need to write a byte to get the SPI to clock the returned status byte in)
    //5. Wait for RBF (receive buffer full) which will be set after the read status command is fully shifted out.
    //6. Read SPI1BUF and discard the dummy data that was clocked in while the read status command was sent out.
    //7. Wait for RBF which will be set after the dummy data byte (sent at step 3) is clocked out
    //8. Read the status byte which was clocked in from the 25LC256 while the dummy data byte (sent at step 3) was clocked out.
    //9. Negate CS.

    //1. Assert CS
    LATBbits.LATB10 = 0;
     //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
    SPI1BUF = SPI_READ_STATUS_REGISTER;
    //SPI_READ_STATUS_REGISTER = SPI1BUF;
    //3. Wait for TBE (transmitter buffer empty)
    while( SPI1STATbits.SPITBE == 0 );
    //4. Write a dummy data byte to SPI1BUF (we need to write a byte to get the SPI to clock the returned status byte in)
    SPI1BUF = DUMMY_DATA;
    //5. Wait for RBF (receive buffer full) which will be set after the read status command is fully shifted out.
    while(SPI1STATbits.SPIRBF == 0);
    //6. Read SPI1BUF and discard the dummy data that was clocked in while the read status command was sent out.
    status2 = SPI1BUF;
    //7. Wait for RBF which will be set after the dummy data byte (sent at step 3) is clocked out
    while(SPI1STATbits.SPIRBF == 0);
    //8. Read the status byte which was clocked in from the 25LC256 while the dummy data byte (sent at step 3) was clocked out.
    status = SPI1BUF;
    
    //9. Negate CS.
    
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    
    LATBbits.LATB10 = 1;

//---End Read Status Command---
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
//---Begin Set Write Enable Latch Command--- // 1 byte shifted out
    //note: this may not need the second read/write cycle
    //1. Assert CS
    LATBbits.LATB10 = 0;
    //2. Write a write status command to the 25LC256
    SPI1BUF = SPI_SET_WRITE_ENABLE_LATCH;
    //3. Wait for RBF (receive buffer full) which will be set after the write status command is fully shifted out.
    while(SPI1STATbits.SPIRBF == 0);
    //4. Read
    status = SPI1BUF;
    //5. Negate CS.
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    LATBbits.LATB10 = 1;

//---End Set Write Enable Latch Command---
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
//---Begin SECOND Read Status Command
    //1. Assert CS
    LATBbits.LATB10 = 0;
     //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
    SPI1BUF = SPI_WRITE_DATA_MEMORY;
    //SPI_READ_STATUS_REGISTER = SPI1BUF;
    //3. Wait for TBE (transmitter buffer empty)
    while( SPI1STATbits.SPITBE == 0 );
    //4. Write a dummy data byte to SPI1BUF (we need to write a byte to get the SPI to clock the returned status byte in)
    SPI1BUF = DUMMY_DATA;
    //5. Wait for RBF (receive buffer full) which will be set after the read status command is fully shifted out.
    while(SPI1STATbits.SPIRBF == 0);
    //6. Read SPI1BUF and discard the dummy data that was clocked in while the read status command was sent out.
    status2 = SPI1BUF;
    //7. Wait for RBF which will be set after the dummy data byte (sent at step 3) is clocked out
    while(SPI1STATbits.SPIRBF == 0);
    //8. Read the status byte which was clocked in from the 25LC256 while the dummy data byte (sent at step 3) was clocked out.
    status = SPI1BUF;

    //9. Negate CS.
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    LATBbits.LATB10 = 1;

//---End SECOND Read Status Command---
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
//---Begin Page Write Command---
//all writes wrap within their 64 byte page. This means the max write length is 64 bytes.
    //1. Assert CS
    LATBbits.LATB10 = 0;
     //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
    SPI1BUF = SPI_WRITE_DATA_MEMORY;
    //SPI_READ_STATUS_REGISTER = SPI1BUF;
    //3. Wait for TBE (transmitter buffer empty)
    while( SPI1STATbits.SPITBE == 0 );
    //4. Write the address MSB
    SPI1BUF = ADDR_MSA;
    //5. Wait for RBF (receive buffer full)
    while(SPI1STATbits.SPIRBF == 0);
    //6. Read SPI1BUF and discard the dummy data
    SPI1BUF;
    //7. Write LSA
    SPI1BUF = ADDR_LSA;
 
    //loop for data write
    for(i = 0; i < WRITE_LENGTH; i++) //loop two less than the actual number of
    {
        //11. Wait for RBF
        while(SPI1STATbits.SPIRBF == 0);
        //12. discard dummy data
        SPI1BUF;
        //13.
        SPI1BUF = oBuff[i]; // REAL DATA not actual dummy data
    }

    //14. Wait for RBF
    while(SPI1STATbits.SPIRBF == 0);
    //15. discard dummy data
    SPI1BUF;
    //16. Wait for RBF
    while(SPI1STATbits.SPIRBF == 0);
    //17. discard dummy data
    SPI1BUF;

    //18. Negate CS.
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    LATBbits.LATB10 = 1;
//---End Page Write Command---
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
 //---Begin Page Read Command---
//all writes wrap within their 64 byte page. This means the read length is 64 bytes.
    //1. Assert CS
    LATBbits.LATB10 = 0;
     //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
    SPI1BUF = SPI_READ_DATA_MEMORY;
    //SPI_READ_STATUS_REGISTER = SPI1BUF;
    //3. Wait for TBE (transmitter buffer empty)
    while( SPI1STATbits.SPITBE == 0 );
    //4. Write the address MSB
    SPI1BUF = ADDR_MSA;
    //5. Wait for RBF (receive buffer full)
    while(SPI1STATbits.SPIRBF == 0);
    //6. Read SPI1BUF and discard the dummy data
    SPI1BUF;
    //7. Write LSA
    SPI1BUF = ADDR_LSA;

    
    //8. Wait for RBF
    while(SPI1STATbits.SPIRBF == 0);
    //9. Read SPI1BUF and discard the dummy data
    SPI1BUF;
    //10. Write dummy data
    SPI1BUF = DUMMY_DATA;
    
    
    //8. Wait for RBF
    while(SPI1STATbits.SPIRBF == 0);
    //9. Read SPI1BUF and discard the dummy data
    SPI1BUF;
    //10. Write dummy data
    SPI1BUF = DUMMY_DATA;
    
    
    //loop for data write
    for(i = 0; i < READ_LENGTH - 2; i++) //loop two less than the actual number of
    {
        //11. Wait for RBF
        while(SPI1STATbits.SPIRBF == 0);
        //12. save recieved byte
        iBuff[i] = SPI1BUF;
        //13. write dummy data
        SPI1BUF = DUMMY_DATA; // 
    }

    //14. Wait for RBF
    while(SPI1STATbits.SPIRBF == 0);
    //15. Read second to last byte of data
    iBuff[62] = SPI1BUF;
    //16. Wait for RBF
    while(SPI1STATbits.SPIRBF == 0);
    //17. Read last byte of data
    iBuff[63] = SPI1BUF;

    //18. Negate CS.
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    LATBbits.LATB10 = 1;
//---End Page Write Command---


    while (1);
}
