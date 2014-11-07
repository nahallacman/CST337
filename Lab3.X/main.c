/*
 * File:   main.c
 * Author: cal
 *
 * Created on October 23, 2014, 8:32 PM
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


//setting the system to 72 MHz operation, with a PBclk of 72 MHz
#pragma config POSCMOD=XT, FNOSC=PRIPLL, FPLLIDIV=DIV_2, FPLLMUL=MUL_18, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_1, FWDTEN=OFF, CP=OFF, BWP=OFF
#pragma config FSOSCEN=OFF, IESO=OFF


//set the priority of the timer2 service routine
#pragma interrupt T2ISR IPL4 vector 8

void T2ISR(void);

int global_count;

//	Function Prototypes
int main(void);

int main(void) {
    //set the system to 72MHz operation, 2 wait states, use prefetch cache, use instruction cache, disable bmx breakpoints
    SYSTEMConfig(72000000L, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //disable everything about t2 just in case
    T2CONbits.ON = 0; // make sure the timer is off just in case
    mT2ClearIntFlag(); // clear the interupt flag just in case

    //configure timer 2
    PR2 = 511; //set the period register to interrupt every 512 counts
    T2CONbits.TCKPS = 0; //set the prescaler to 1:1
    TMR2 = 0x0; // zero out the timer value just in case

    //turn on multi vectored mode
    INTCONbits.MVEC = 1;

    //configure the timer 2 priority
    mT2SetIntPriority(4);
    // and sub priority
    mT2SetIntSubPriority(0);
    //Then enable the interrupt (TODO: double check if this is an atomic operation)
    mT2IntEnable(1);

    //enable interrupts
    asm("ei");
    
    //configure the LED for output
    TRISDCLR = 1;

    //initalize global counter
    global_count = 0;

    T2CONbits.ON = 1; // start the timer


    int a = 0;
    int MAX_NUM = 7200000;
    while (1)
    {
        if(a > MAX_NUM)
        {
            a = 0;
            //TMR2 = 0x0; // reset the timer
            PORTDINV = 1;
        }
        else
        {
            a++;
        }
    }
}

void T2ISR(void)
{
    //increment global counter
    global_count++;
    //atomically clear the interrupt flag
    mT2ClearIntFlag();
}

