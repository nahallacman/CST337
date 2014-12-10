/*
 * File:   main.c
 * Author: cal
 *
 * Created on December 5, 2014, 11:10 PM
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

//set the priority of the UART1 service routine
#pragma interrupt UART1ISR IPL7 vector 24
//set the priortiy of the Timer 2&3 service routine
#pragma interrupt T23ISR IPL2 vector 12
//set the priortiy of the Timer 4&5 service routine
#pragma interrupt T45ISR IPL1 vector 20

//UART1 ISR declaration
void UART1ISR();
//Timer2and3 ISR declaration
void T23ISR();
//timer 4and5 ISR declaration
void T45ISR();

#include <p32xxxx.h>
#include <plib.h>

//	Function Prototypes
int main(void);

#pragma config POSCMOD=XT, FNOSC=PRIPLL, FPLLIDIV=DIV_2, FPLLMUL=MUL_20, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_1, FWDTEN=OFF, CP=OFF, BWP=OFF
#pragma config FSOSCEN=OFF, IESO=OFF

char const TEST_DATA = 0x55;
char recieved = 0x55; // for testing

int main(void) {
    //set the system to 80MHz operation, auto(2) wait states, use prefetch cache, use instruction cache, disable bmx breakpoints
    SYSTEMConfig(80000000L, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //turn on multi vectored mode
    INTCONbits.MVEC = 1;

    //---begin LED config---
    //using LED0 on RD0 (overflow) and LED1 on RD1 (rx) and LED2 on RD2(tx)
    TRISDbits.TRISD0 = 0; //overflow ( RED )
    TRISDbits.TRISD1 = 0; //RX ( YELLOW )
    TRISDbits.TRISD2 = 0; //TX ( GREEN )

    PORTDbits.RD0 = 0; //initalize to off
    PORTDbits.RD1 = 0;
    PORTDbits.RD2 = 0;

    //using timer 2 and 3 for 1/2 a second delay of RX LED
    //PBclk = 80mhz, need 40,000,000 clocks for 1/2 second delay
    //40,000,000 / 256 = 156250
    //2^16 = 65536
    //so we need to use a 32 bit timer as we don't want to change PBclk due to UART1
    TMR2 = 0;//0 out timer just in case

    T2CONbits.TCKPS = 0b000; //1:1 prescaler
    T2CONbits.T32 = 1; // 32 bit mode

    PR2 = 40000000; // 1/2 second period

    IFS0bits.T3IF = 0; // clear flag
    IEC0bits.T3IE = 1; // enable

    IPC3bits.T3IP = 0b010; // priority 2
    IPC3bits.T3IS = 0; // subpriority 0

    //using timers 4 and 5 for 1/2 a second delay of TX LED
    TMR4 = 0;//0 out timer just in case

    T4CONbits.TCKPS = 0b000; //1:1 prescaler
    T4CONbits.T32 = 1; // 32 bit mode

    PR4 = 40000000; // 1/2 second period

    IFS0bits.T5IF = 0; // clear flag
    IEC0bits.T5IE = 1; // enable

    IPC5bits.T5IP = 0b001; // priority 1
    IPC5bits.T5IS = 0; // subpriority 0

    //---end LED config---

    //---begin button config---
    //using button RC1

    //using timer 1 for 15ms debounce checks

    //---end button config---

    //---begin UART1 config---



    U1MODEbits.BRGH = 0;// standard 16x mode
    U1MODEbits.UEN = 0b10; //CTS and RTS are under hardware control
    U1MODEbits.PDSEL = 0b00; //8 bits no parity
    U1MODEbits.RTSMD = 0; //UxRTS pin is in Flow Control mode

    //U1MODEbits.UEN = 0b11; // use this mode to test the baud clock on pin RD16
    //U1MODEbits.UEN = 0b00; // guess testing

    U1STA = 0; //clear status register just in case

    U1STAbits.URXEN = 1; //enable hardware control of RX pin
    U1STAbits.UTXEN = 1; //enable hardware control of TX pin
    U1STAbits.UTXISEL = 0b10; // interrupt when the transmit buffer becomes empty

    //intended baud rate = 115.2k baud
    //Baud Rate = Fpb / (16 * (U1BRG + 1))
    //or U1BRG = Fpb / ( 16 * Baud Rate ) - 1
    // ( 80000000 / ( 16 * 115200 ) )  - 1 = 42.402778 or about 42
    U1BRG = 42;

    //intended baud rate = 9600 baud
    // ( 80000000 / ( 16 * 9600 ) )  - 1 = 519.83333333333333333333333333333 or about 520
    //U1BRG = 520; // THIS VALUE BREAKS THINGS -> possiblites? 512 max value?

    //U1MODEbits.LPBACK = 1; // turn on loopback for testing purposes

    U1MODEbits.ON = 1; // turn UART1 on at the end of config
    //---end UART1 config---

    //---begin Interrupt config---
    IFS0bits.U1RXIF = 0; //clear interrupt flags just in case
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1EIF = 0;

    IPC6bits.U1IP = 0b111;
    IPC6bits.U1IS = 0b00;

    IEC0bits.U1RXIE = 1; //enable RX and TX interrupts
    IEC0bits.U1TXIE = 1;
    IEC0bits.U1EIE = 1;
    //---end Interrupt config---

    asm("ei");
    T2CONbits.ON = 1; //start timer 2 for testing
    //turn on RX LED for testing
    PORTDbits.RD1 = 1;

    T4CONbits.ON = 1; //start timer 4 for testing
    //turn on TX LED for testing
    PORTDbits.RD2 = 1;

    //IFS0bits.U1RXIF = 1; //manually set flag to test interrupts
    //IFS0bits.U1TXIF = 1;
    //IFS0bits.U1EIF = 1;

    //IFS0bits.T3IF = 1;

    while (1);
}

void UART1ISR()
{
    //char recieved = 0; // for testing
    
    //needs to check if the transmit or recieve interrupt flag is set?
    //IFS0CLR = 0x1C000000;  //bits 26(error), 27(RX), and 28(TX)
    if(IFS0bits.U1EIF == 1) //overflow error
    {
        //turn on Overflow LED
        PORTDbits.RD0 = 1;
        IFS0CLR = 0x4000000; // clear overflow flag
    }
    else if(IFS0bits.U1RXIF == 1)
    {
        int temp = 0;
        //start RX LED timer
        //turn on RX LED
        PORTDbits.RD1 = 1;
        IFS0CLR = 0x8000000; // clear recieve flag
        //read from U1 RX buffer
        temp = U1RXREG;
        if(temp)
        {
            recieved = temp;
        }
        U1TXREG = recieved;
        TMR2 = 0; //clear the timer
        T2CONbits.ON = 1; // start the timer
    }
    else if(IFS0bits.U1TXIF == 1)
    {
        //start TX LED timer
        //turn on TX LED
        PORTDbits.RD2 = 1;
        IFS0CLR = 0x10000000; // clear transmit flag
        //write more to the U1 TX buffer
        U1TXREG = recieved;
        TMR4 = 0; //clear the timer
        T4CONbits.ON = 1; // start the timer
    }
}

//Timer2and3 ISR declaration
void T23ISR()
{
    //clear T3IF flag atomically
    IFS0CLR = 0x1000;
    //turn off RX LED
    PORTDbits.RD1 = 0;
    T2CONbits.ON = 0; // stop the timer
    TMR2 = 0; //clear the timer
}
//timer 4and5 ISR declaration
void T45ISR()
{
    //clear T5IF flag atomically
    IFS0CLR = 0x100000;

    //turn off TX LED
    PORTDbits.RD2 = 0;
    T4CONbits.ON = 0; // stop the timer
    TMR4 = 0; //clear the timer
}