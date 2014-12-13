/*
 * File:   main.c
 * Author: Cal Barkman
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
//set the priority of the Timer 1 service routine
#pragma interrupt T1ISR IPL1 vector 4
//set the priortiy of the Timer 2&3 service routine
#pragma interrupt T23ISR IPL3 vector 12
//set the priortiy of the Timer 4&5 service routine
#pragma interrupt T45ISR IPL2 vector 20


//UART1 ISR definition
void UART1ISR();
//timer 1 ISR declaration
void T1ISR();
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

//variables for circular buffer
char buffer[1000]; //8KB buffer, needs to go into RAM
char * h;
char * t;
char * begin;
char * end;
int num_items;
int max = 1000;



int T1STATE = 0;
int debouncestate = 0;
int buttonpressed = 0;


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
    IEC0bits.T3IE = 1; // enable interrupt

    IPC3bits.T3IP = 0b011; // priority 3
    IPC3bits.T3IS = 0; // subpriority 0

    //using timers 4 and 5 for 1/2 a second delay of TX LED
    TMR4 = 0;//0 out timer just in case

    T4CONbits.TCKPS = 0b000; //1:1 prescaler
    T4CONbits.T32 = 1; // 32 bit mode

    PR4 = 40000000; // 1/2 second period

    IFS0bits.T5IF = 0; // clear flag
    IEC0bits.T5IE = 1; // enable interrupt

    IPC5bits.T5IP = 0b010; // priority 2
    IPC5bits.T5IS = 0; // subpriority 0

    //---end LED config---

    //---begin button config---
    //using button RC1
    TRISCbits.TRISC1 = 1; // set to input
    

    //using timer 1 for 15ms debounce checks
    TMR1 = 0; //clear the timer just in case
    T1CONbits.TCKPS = 0b11; //1:256 prescaler
    PR1 = 46875; //15ms delay, (80,000,000 / 256) * .15 = 46875, max PR1 val is 65535

    IFS0bits.T1IF = 0; // clear flag
    IEC0bits.T1IE = 1; // enable interrupt

    IPC1bits.T1IP = 0b001; // priority 1
    IPC1bits.T1IS = 0; // subpriority 0
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

    //---begin circular buffer init
    num_items = 0;
    begin = h = t = buffer;
    //buffer;
    end = begin + max;
    // I think I may be off by one
    //end = begin + max - 1;
    //circular buffer LED config
    TRISBbits.TRISB4 = 0;//using LED RB4 (output)
    PORTBbits.RB4 = 1;//init to on
    //---end circular buffer init

    //---end UART1 config---



     asm("ei");

    int defaultstate = 0;

    while (1)
    {
        if(buttonpressed == 1)
        {
            if(PORTCbits.RC1 == 1)
            {
                T1CONbits.ON = 1;
                //IFS0bits.T1IF = 1; // set timer flag to start button press state machine
            }
        }
        else
        {
            if(PORTCbits.RC1 == 0)
            {
                T1CONbits.ON = 1;
                //IFS0bits.T1IF = 1; // set timer flag to start button press state machine
            }
        }
    }
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
        /*
        temp = U1RXREG;
        if(temp)
        {
            recieved = temp;
        }
        U1TXREG = recieved;
         */

        // do we have to check if the value is a break first?

        //begin circular buffer insert
        if(num_items < max) // then the buffer isn't full
        {
            *h = U1RXREG;
            num_items++;
            h++;
            if( h > end )
            {
                h = begin;
            }
        }
        else
        {
            //turn on buffer error LED
            PORTBbits.RB4 = 0;
            //do I try to keep reading here?
            U1RXREG;
        }

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
        //U1TXREG = recieved;
        
        //begin circular buffer removal
        if(num_items > 0)
        {
            U1TXREG = *t;
            num_items--;
            t++;
            if(t > end) // make buffer circular
            {
                t = begin;
            }
        }

        TMR4 = 0; //clear the timer
        T4CONbits.ON = 1; // start the timer
    }
}

//Timer2and3 ISR definition
void T23ISR()
{
    //clear T3IF flag atomically
    IFS0CLR = 0x1000;
    //turn off RX LED
    PORTDbits.RD1 = 0;
    T2CONbits.ON = 0; // stop the timer
    TMR2 = 0; //clear the timer
}
//timer 4and5 ISR definition
void T45ISR()
{
    //clear T5IF flag atomically
    IFS0CLR = 0x100000;

    //turn off TX LED
    PORTDbits.RD2 = 0;
    T4CONbits.ON = 0; // stop the timer
    TMR4 = 0; //clear the timer
}

//timer 1 ISR definition
void T1ISR()
{
    //clear T1IF flag atomically
    IFS0CLR = 0x10;
    if(buttonpressed == 1) //button is going from active to inactive (0 to 1)
    {
        switch(T1STATE)
        {
            case 0:
                if(PORTCbits.RC1 == 1) // button still not pressed
                {
                    T1STATE = 1; // continue
                }
                else    //start the sequence over
                {
                    T1STATE = 0;
                    T1CONbits.ON = 0;
                    TMR1 = 0;
                }
                break;
            case 1:
                if(PORTCbits.RC1 == 1) // button still pressed
                {
                    buttonpressed = 0;
                }
                T1STATE = 0; // reset
                T1CONbits.ON = 0;
                TMR1 = 0;
                break;
            default:
                break;
        }
    }
    else // button is going from inactive to active ( 1 to 0 )
    {
        switch(T1STATE)
        {
            case 0:
                if(PORTCbits.RC1 == 0) // button still pressed
                {
                    T1STATE = 1; // continue
                }
                else    //start the sequence over
                {
                    T1STATE = 0;
                    T1CONbits.ON = 0;
                    TMR1 = 0;
                }
                break;
            case 1:
                if(PORTCbits.RC1 == 0) // button still pressed
                {
                    buttonpressed = 1; // button press confirmed
                    PORTBbits.RB4 = 1; // clear the overrun LED
                    IFS0SET = 0x10000000; // set transmit flag to begin chain of transmits
                }
                T1STATE = 0; // reset
                T1CONbits.ON = 0;
                TMR1 = 0;
                break;
            default:
                break;
        }
    }
}