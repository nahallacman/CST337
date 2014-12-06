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

//	Function Prototypes
int main(void);

int main(void) {

    //---begin LED config---


    //---end LED config---

    //---begin UART1 config---
    U1MODEbits.BRGH = 0;// standard 16x mode
    U1MODEbits.UEN = 0b10; //CTS and RTS are under software control
    U1MODEbits.PDSEL = 0b00; //8 bits no parity

    U1STA = 0; //clear status register just in case

    //intended baud rate = 115.2k baud
    //Baud Rate = Fpb / (16 * (U1BRG + 1))
    //or U1BRG = Fpb / ( 16 * Baud Rate ) - 1
    // ( 80000000 / ( 16 * 115200 ) )  - 1 = 1843199
    U1BRG = 1843199;

    U1MODEbits.LPBACK = 1; // turn on loopback for testing purposes

    U1MODEbits.ON = 1; // turn things on at the end
    //---end UART1 config---

    while (1);
}

void UART1ISR()
{
    //needs to check if the transmit or recieve interrupt flag is set?


}

