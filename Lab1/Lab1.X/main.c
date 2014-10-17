/*
 * File:   main.c
 * Author: cal
 *
 * Created on October 9, 2014, 12:59 AM
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


//#include <p32xxxx.h>
#include <xc.h>


//using pragmas to configure a project since the settings usually change from project to project anyways
#pragma config POSCMOD=XT, FNOSC=PRIPLL, FPLLIDIV=DIV_2, FPLLMUL=MUL_18, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_2, FWDTEN=OFF, CP=OFF, BWP=OFF
#pragma config FSOSCEN=OFF, IESO=OFF


//	Function Prototypes
int main(void);

int main(void) {




    //timer config
    //we are trying to delay for 1/4th of a second
    //system clock = 72MHz
    //periperial bus clock = 36MHz
    //at this periperial bus speed, it would take 9,000,000 cycles
    // = .25 s / (1/(36 * 10 ^ 6)) = 9,000,000 cycles
    //needs to be configured for 16 bit ungated operation
    //16 bits means 2^16 - 1 = 65535 max count so we need a prescaler
    //9,000,000 / 256 = 35156
    //therefore we use a prescalere of 256
    //and a period register of 35156


    T2CON = 0x0; // Stop Timer and clear control register, set prescaler at 1:1, internal clock source
    
    T2CONbits.TCKPS = 0b111; // set prescaler to a value of 1:256
    TMR2 = 0x0; // Clear timer register
  
    //PR2 = 35156; // Load period register // 35156 = 0x8954
    PR2 = 4;
    //T2CONSET = 0x8000; // Start Timer
    T2CONbits.ON = 1; // Start Timer

    //before port D can be used for an output it needs to be configured properly
    //not sure on this:
    //to do this, we simply ensure the TRISD bit 0 is set to 0
    TRISDCLR = 1;

    while (1)
    {
        //if(TMR2 == 35156)
        if(TMR2 == 3)
        {
        //toggle LED RD0
        
        //PORTD |= 1;
        //PORTD &= ~1;

        //PORTDbits.RD0 = 0;
        //PORTDbits.RD0 = 1;

        //PORTDSET = 1;
        //PORTDCLR = 1;
        TMR2 = 0x0; // reset the timer so it doesn't enter this loop multiple times
        PORTDINV = 1;
        
        }
    }
}



