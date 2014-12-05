/*
 * File:   main.c
 * Author: cal
 *
 * Created on December 2, 2014, 11:37 PM
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

//set the priority of the timer2 service routine
#pragma interrupt SPI1ISR IPL4 vector 23

//	Function Prototypes
int main(void);

//void ReadEEProm(int nbytes, unsigned int address, unsigned char[] readbuffer);
//void WriteEEProm(int nbytes, unsigned int address, unsigned char[] writebuffer);
void ReadEEProm(int nbytes, unsigned int address, unsigned char* readbuffer);
void WriteEEProm(int nbytes, unsigned int address, unsigned char* writebufer);

//actual SPI ISR
void SPI1ISR();

char SPI_READ_STATUS_REGISTER = 0b00000101;
char SPI_WRITE_STATUS_REGISTER = 0b00000001;

char SPI_READ_DATA_MEMORY = 0b00000011;
char SPI_WRITE_DATA_MEMORY = 0b00000010;

char SPI_RESET_WRITE_ENABLE_LATCH = 0b00000100;
char SPI_SET_WRITE_ENABLE_LATCH = 0b00000110;

char SPI_WRITE_ENABLE_MASK = 0b00000010;

int WRITE_LENGTH = 63; //0-63 = 64 divisions
int READ_LENGTH = 63;
char DUMMY_DATA = 0x45;
char TEST_DATA = 0x55;
char ADDR_MSA = 0x20;//testing things out with a read to the very first page (or last page)
char ADDR_LSA = 0x00;//lsb doesn't matter since we are doing reads on a page

int entrycount = 0;

//ISR variables
int EEPromSysBusy = 0; // initalize to 0, will tell the user if the system is busy using the SPI device or not
unsigned int state = 0; // controls the state machine
int Rnbytes = 0; // use for global access of how many bytes to read
char RADDR_MSA = 0x00;
char RADDR_LSA = 0x00;
char * RBUFF;
unsigned int READNUM = 0; // internal ISR counter for number of bytes to read still
char rstatus = 0;

int Wnbytes = 0;
char WADDR_MSA = 0x00;
char WADDR_LSA = 0x00;
char * WBUFF;
unsigned int WRITENUM = 0;

#define CHECKSTATUS 0
#define CHECKSTATUSREAD1 200
#define CHECKSTATUSREAD2 201
#define CHECKSTATUSREAD3 202
#define READ1 1
#define READ2 2
#define READ3 3
#define READ4 4
#define READ5 5
#define READ6 6
#define READ7 7
#define READ8 8
#define READ9 9
#define READ10 10
#define READ11 11
#define N1 100
#define CHECKWELSTATUS1 300
#define CHECKWELSTATUS2 301
#define CHECKWELSTATUS3 302
#define CHECKWELSTATUS4 303
#define SETWEL1 500
#define SETWEL2 501
#define CHECKSTATUSWRITE1 400
#define CHECKSTATUSWRITE2 401
#define CHECKSTATUSWRITE3 402
#define WRITE1 1000
#define WRITE2 1001
#define WRITE3 1002
#define WRITE4 1003
#define WRITE5 1004
#define WRITE6 1005

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
    char iBuff2[64];
    char oBuff[64];

    for(i = 0; i < 64; i++)
    {
        iBuff[i] = 0;
        iBuff2[i] = 0;
        oBuff[i] = i;
    }

    oBuff[0] = 0x55;
    oBuff[63] = 0x55;
    
//---Begin Timer config for timing how long TBE takes to set--
    
    //disable everything about t2 just in case
    T2CONbits.ON = 0; // make sure the timer is off just in case
    mT2ClearIntFlag(); // clear the interupt flag just in case

    //configure timer 2
    PR2 = -1; //set the period register to interrupt every 2^16 counts
    T2CONbits.TCKPS = 0; //set the prescaler to 1:1
    TMR2 = 0x0; // zero out the timer value just in case

    T2CONbits.ON = 1;

//---End Timer Config---

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

    //turn on multi vectored mode
    INTCONbits.MVEC = 1;

    //using interupts
    // bits 23, 24, and 25
    // 0011 1000 0000 0000 0000 0000 0000 = 0x3800000
    IFS0CLR = 0x3800000; // clear the interupt flags just in case
    //IEC0SET = 0x3800000; // set the interrupt enables
    IEC0bits.SPI1RXIE = 1;

    //bits 28:26 = priority
    // 0001 1100 0000 0000 0000 0000 0000 0000 = 0x1C000000
    IPC5SET = 0x10000000; //set the priority to 4
    //bits 25:24 = subpriority
    // 0011 0000 0000 0000 0000 0000 0000 = 0x3000000
    IPC5CLR = 0x3000000; //clear the subpriority to 0

    //mSPI1SetIntPriority(4);
    //mSPI1SetIntSubPriority(0);

    //mSPI1SetIntEnable(1);



//---End SPI Configuration---

    //enable interrupts
    asm("ei");

    //ReadEEProm(1, 0x1234, iBuff);









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
    for(i = 0; i < WRITE_LENGTH + 1; i++) //loop two less than the actual number of
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
//--- Begin Check status register until write is done---
do
{
//---Begin SECOND Read Status Command
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


}
while(status & 0b00000001);// check if work in progress bit is set

//---End SECOND Read Status Command---
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");

    /*
//---Begin Page Read Command---
//all writes wrap within their 64 byte page. This means the read length is 64 bytes.
    //1. Assert CS
    LATBbits.LATB10 = 0;
     //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
    SPI1BUF = SPI_READ_DATA_MEMORY;

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
    for(i = 0; i < READ_LENGTH - 1; i++) //loop two less than the actual number of
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
//---End Page Read Command---

    for(i = 0; i < 64; i++)
    {
        iBuff[i] = 0;
    }
*/

    ReadEEProm(64, 0x2000, iBuff);

    WriteEEProm(64, 0x4000, oBuff);

    //ReadEEProm(64, 0x4000, iBuff2);
    
    while (1);
}



//manually set the SPI1 interrupt flag
void ReadEEProm(int nbytes, unsigned int address, unsigned char* readbuffer)
{
    if( nbytes != 0)
    {
        //check EEPromSysBusy until it is 0
        while(EEPromSysBusy);

        asm("di"); //disable interrupts while global variables are being accessed
        //prepare the system to read nbytes bytes.

        state = CHECKSTATUSREAD1;
        EEPromSysBusy = 1;
        Rnbytes = nbytes;
        RADDR_MSA = (address >> 8);
        RADDR_LSA = address & 0x00FF;
        RBUFF = readbuffer; // this may need to copy address of instead
        READNUM = 0; // initalize to 0

        asm("ei"); //enable interrupts after global variables are being accessed

        //manually set the SPI1 TX interrupt flag
        //IFS0SET = 0x01000000;
        //manually set the SPI1 RX interrupt flag
        IFS0SET = 0x02000000;
        //manually set the SPI1 error interrupt flag
        //IFS0SET = 0x04000000;
        //IFS0bits.SPI1TXIF = 1;

    }
    else
    {
        //error? cant read 0 bytes
    }
    
    return;
}

void WriteEEProm(int nbytes, unsigned int address, unsigned char* writebuffer)
{
    if( nbytes != 0)
    {
        //check EEPromSysBusy until it is 0
        while(EEPromSysBusy);

        asm("di"); //disable interrupts while global variables are being accessed
        //prepare the system to read nbytes bytes.

        state = CHECKWELSTATUS1;
        EEPromSysBusy = 1;
        Wnbytes = nbytes;
        WADDR_MSA = (address >> 8);
        WADDR_LSA = address & 0x00FF;
        WBUFF = writebuffer;
        WRITENUM = 0; // re-initalize to 0

        asm("ei"); //enable interrupts after global variables are being accessed

        //manually set the SPI1 RX interrupt flag
        IFS0SET = 0x02000000;

    }
    else
    {
        //error?
    }
    return;
}

void SPI1ISR()
{
    //note: will probably need to check which interrupt flag has been set
    //manually clear the SPI1 TX interrupt flag
    IFS0CLR = 0x07000000;
    
    switch(state)
    {
        case CHECKSTATUS:
            entrycount++;
            state = CHECKSTATUS;
            break;

        case CHECKSTATUSREAD1: //--- Begin Check status register until write is done---

            //1. Assert CS
            LATBbits.LATB10 = 0;
             //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
            SPI1BUF = SPI_READ_STATUS_REGISTER;
            //SPI_READ_STATUS_REGISTER = SPI1BUF;
            //3. Wait for TBE (transmitter buffer empty)
            while( SPI1STATbits.SPITBE == 0 );
            //4. Write a dummy data byte to SPI1BUF (we need to write a byte to get the SPI to clock the returned status byte in)
            SPI1BUF = DUMMY_DATA;
            state = CHECKSTATUSREAD2;
            break;

        case CHECKSTATUSREAD2:
            //5. Wait for RBF (receive buffer full) which will be set after the read status command is fully shifted out.
            //while(SPI1STATbits.SPIRBF == 0);
            //6. Read SPI1BUF and discard the dummy data that was clocked in while the read status command was sent out.
            SPI1BUF;

            state = CHECKSTATUSREAD3;
            break;

        case CHECKSTATUSREAD3:
            //7. Wait for RBF which will be set after the dummy data byte (sent at step 3) is clocked out
            //while(SPI1STATbits.SPIRBF == 0);
            //8. Read the status byte which was clocked in from the 25LC256 while the dummy data byte (sent at step 3) was clocked out.
            rstatus = SPI1BUF;

            /*
            //9. Negate CS.
            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
            */
            LATBbits.LATB10 = 1;

            // check if work in progress bit is set
            if(rstatus & 0b00000001)
            {
                state = CHECKSTATUSREAD1; //if it is set, keep checking until it isn't
            }
            else
            {
                state = READ1;
            }
            //manually set the SPI1 RX interrupt flag so the ISR will re-enter
            IFS0SET = 0x02000000;
            break;

        case READ1:
            //---Begin Page Read Command---
            //all writes wrap within their 64 byte page. This means the read length is 64 bytes.
            //1. Assert CS
            LATBbits.LATB10 = 0;
            //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
            SPI1BUF = SPI_READ_DATA_MEMORY;

            //3. Wait for TBE (transmitter buffer empty)
            while( SPI1STATbits.SPITBE == 0 );
            //4. Write the address MSB
            SPI1BUF = RADDR_MSA;
            state = READ2;
            break;

        case READ2:
            //5. Wait for RBF (receive buffer full)
            //while(SPI1STATbits.SPIRBF == 0);
            //6. Read SPI1BUF and discard the dummy data
            SPI1BUF;
            //7. Write LSA
            SPI1BUF = RADDR_LSA;
            state = READ3;
            break;

        case READ3:
            //8. Wait for RBF
            //while(SPI1STATbits.SPIRBF == 0);
            //9. Read SPI1BUF and discard the dummy data
            SPI1BUF;
            //10. Write dummy data
            SPI1BUF = DUMMY_DATA;
            if(Rnbytes == 1)
            {
                state = READ7;
            }else if(Rnbytes == 2)
            {
                state = READ5;
            }else
            {
                state = N1;
            }
            break;
        case N1:
            SPI1BUF;
            SPI1BUF = DUMMY_DATA;
            state = READ4;
            break;


        case READ4: //11. Wait for RBF

            if( READNUM < Rnbytes - 3 )
            {}
            else
            {
                state = READ10;
            }
            //12. save recieved byte
            RBUFF[READNUM] = SPI1BUF;
            //13. write dummy data
            SPI1BUF = DUMMY_DATA; //
            //}
            READNUM++;
            break;
        case READ5:
            SPI1BUF;
            SPI1BUF = DUMMY_DATA; 
            state = READ6;
            break;
        case READ6://14. Wait for RBF
            
            //15. Read second to last byte of data
            RBUFF[READNUM] = SPI1BUF;
            //?? clock out last dummy data
            SPI1BUF = DUMMY_DATA; 
            READNUM++;
            state = READ8;
            break;
        case READ7:
            SPI1BUF;
            state = READ8;
            break;
        case READ8: //16. Wait for RBF
            //17. Read last byte of data
            RBUFF[READNUM] = SPI1BUF;
            state = READ9;
            break;
        case READ9:
            //18. Negate CS.
            LATBbits.LATB10 = 1; // note: CS MAY be reasserted to early

            //---End Page Write Command---
            state = CHECKSTATUS;
            break;
        case READ10:
            //14. Read second to last byte of data
            RBUFF[READNUM] = SPI1BUF;
            READNUM++;
            state = READ11;
            break;
        case READ11:
            //15. Read last byte of data
            RBUFF[READNUM] = SPI1BUF;
            //18. Negate CS.
            LATBbits.LATB10 = 1; // note: CS MAY be reasserted to early

            //---End Page Write Command---
            state = CHECKSTATUS;
            break;

        case CHECKWELSTATUS1:
            //1. Assert CS
            LATBbits.LATB10 = 0;
             //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
            SPI1BUF = SPI_READ_STATUS_REGISTER;
            //SPI_READ_STATUS_REGISTER = SPI1BUF;
            //3. Wait for TBE (transmitter buffer empty)
            while( SPI1STATbits.SPITBE == 0 );
            //4. Write a dummy data byte to SPI1BUF (we need to write a byte to get the SPI to clock the returned status byte in)
            SPI1BUF = DUMMY_DATA;
            state = CHECKWELSTATUS2;
            break;

        case CHECKWELSTATUS2:
            //5. Wait for RBF (receive buffer full) which will be set after the read status command is fully shifted out.
            //while(SPI1STATbits.SPIRBF == 0);
            //6. Read SPI1BUF and discard the dummy data that was clocked in while the read status command was sent out.
            SPI1BUF;
            state = CHECKWELSTATUS3;
            break;

        case CHECKWELSTATUS3:
            //7. Wait for RBF which will be set after the dummy data byte (sent at step 3) is clocked out
            // while(SPI1STATbits.SPIRBF == 0);
            //8. Read the status byte which was clocked in from the 25LC256 while the dummy data byte (sent at step 3) was clocked out.
            rstatus = SPI1BUF;

            state = CHECKWELSTATUS4;
            break;
        case CHECKWELSTATUS4:

            
            //9. Negate CS.
            LATBbits.LATB10 = 1;

            SPI1BUF;
            //check if write enable latch is not set
            if((rstatus & 0b00000010) == 0)
            {
                
                state = SETWEL1;
            }
            // check if write in progress bit is set
            else if(rstatus & 0b00000001)
            {
                state = CHECKSTATUSWRITE1; //if it is set, keep checking until it isn't
            }
            else
            {
                state = WRITE1;
            }
            //manually set the SPI1 RX interrupt flag so the ISR will re-enter
            IFS0SET = 0x02000000;

            break;

        case SETWEL1:
            //---Begin Set Write Enable Latch Command--- // 1 byte shifted out
           //note: this may not need the second read/write cycle
             //1. Assert CS
            LATBbits.LATB10 = 0;
            //2. Write a write status command to the 25LC256
            SPI1BUF = SPI_SET_WRITE_ENABLE_LATCH;
            state = SETWEL2;
            break;

        case SETWEL2:
            //3. Wait for RBF (receive buffer full) which will be set after the write status command is fully shifted out.
            //4. Read dummy data
            //SPI1BUF;
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

            state = CHECKWELSTATUS1;
            //manually set the SPI1 RX interrupt flag so the ISR will re-enter
            IFS0SET = 0x02000000;
            break;

        case WRITE1:
            //1. Assert CS
            LATBbits.LATB10 = 0;
             //2. Write a read status command to the 25LC256 (write a read status command to SPI1BUF).
            SPI1BUF = SPI_WRITE_DATA_MEMORY;
            //SPI_READ_STATUS_REGISTER = SPI1BUF;
            //3. Wait for TBE (transmitter buffer empty)
            while( SPI1STATbits.SPITBE == 0 );
            //4. Write the address MSB
            SPI1BUF = WADDR_MSA;
            state = WRITE2;
            break;

        case WRITE2:
            //5. Wait for RBF (receive buffer full)
            //while(SPI1STATbits.SPIRBF == 0);
            //6. Read SPI1BUF and discard the dummy data
            SPI1BUF;
            //7. Write LSA
            SPI1BUF = WADDR_LSA;
            state = WRITE3;
            break;
          
        case WRITE3: //11. Wait for RBF
            if( WRITENUM + 2 > Wnbytes  )
            {
                state = WRITE4;
            }
            //12. discard dummy data
            SPI1BUF;
            //13.
            SPI1BUF = WBUFF[WRITENUM]; // REAL DATA not actual dummy data
            WRITENUM++;
            break;
            
        case WRITE4:
            //14. Wait for RBF
            //while(SPI1STATbits.SPIRBF == 0);
            //15. discard dummy data
            SPI1BUF;
            state = WRITE5;
            break;

        case WRITE5:
            //16. Wait for RBF
            //while(SPI1STATbits.SPIRBF == 0);
            //17. discard dummy data
            SPI1BUF;
            state = WRITE6;
            break;
        case WRITE6:
            //18. Negate CS.
            LATBbits.LATB10 = 1;
            state = CHECKSTATUS; // what should actually go here?
            break;
            
        default:
            //error?
            break;
    }
    EEPromSysBusy = 0;
}