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
#pragma interrupt UART1ISR IPL4 vector 24

//UART1 ISR declaration
void UART1ISR();

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

    //using timer 2 for 1/2 a second delay of RX LED
    TMR2 = 0;//0 out timer just in case
    PR2 = 1;

    //using timer 3 for 1/2 a second delay of TX LED

    //---end LED config---

    //---begin button config---
    //using button RC1

    //using timer 4 for 15ms debounce checks

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

    IPC6bits.U1IP = 0b011;
    IPC6bits.U1IS = 0b00;

    IEC0bits.U1RXIE = 1; //enable RX and TX interrupts
    IEC0bits.U1TXIE = 1;
    IEC0bits.U1EIE = 1;
    //---end Interrupt config---

    asm("ei");

    //IFS0bits.U1RXIF = 1; //manually set flag to test interrupts
    //IFS0bits.U1TXIF = 1;
    //IFS0bits.U1EIF = 1;

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
    }
    else if(IFS0bits.U1TXIF == 1)
    {
        //start TX LED timer
        //turn on TX LED
        PORTDbits.RD2 = 1;
        IFS0CLR = 0x10000000; // clear transmit flag
        //write more to the U1 TX buffer
        U1TXREG = recieved;
    }
}

