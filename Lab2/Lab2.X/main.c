/*
 * File:   main.c
 * Author: cal
 *
 * Created on October 16, 2014, 7:30 PM
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
//#include <fourier.h>
#include <C:\Users\cal\Desktop\School\CST337\CST337\Lab2\Lab2.X\FOURIER.h>
//#include <FOURIER.h>
//#include <FFTMISC.c>
#include <math.h>

//included to use macros
#include <plib.h>

#define M_PI 3.14159265358979323846 // approximation of PI
#define NSAMP 256 //used for Number of SAMPles
#define TPIN (2 * M_PI/NSAMP) 

extern float in[];
extern float outR[];
extern float outI[];
extern float mag[];

//	Function Prototypes
int main(void);

//using pragmas to configure a project since the settings usually change from project to project anyways
//setting the system to 72 MHz operation, with a PBclk of 72 MHz
#pragma config POSCMOD=XT, FNOSC=PRIPLL, FPLLIDIV=DIV_2, FPLLMUL=MUL_18, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_1, FWDTEN=OFF, CP=OFF, BWP=OFF
#pragma config FSOSCEN=OFF, IESO=OFF


int main(void) {

    //configuring the prefetch cache
    //mCheConfigure(CHECON | 0x30);
    //CHECONbits.PREFEN = 0x3;


    //configuring the instruction cache
    //CheKseg0CacheOn();

    //configuring wait states
    //default reset value is 7
    // processor = 72 MHz, Flash speed = 30MHz
    //this means 2.4 clock cycles per flash cycle
    //by default the processor always uses one instruction for the load instruction
    //this means we need 3 cycles total, meaning we need to set the wait state cycles to 2
    //so we configure the CHECON (Cache Control) register's PFMWS (prefetch module wait states)
    //CHECONbits.PFMWS = 2;

    //NOTE: the 3 above steps are done with this single macro:
    SYSTEMConfig(72000000L, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //configuring timer 2 and 3 to be a 32 bit timer
    //OpenTimer23(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 0); // set the timer
    OpenTimer23(T2_OFF | T2_SOURCE_INT | T2_PS_1_1 | T2_32BIT_MODE_ON, 0); // set timer 2 (and 3) with settings.  internal source, 1:1 prescaler, 32 bit mode
    int time = 0; // variable to count time with

    //configuring timer 2 and 3 to be a 32 bit timer
    /*
    T2CONbits.T32 = 1; //This sets operation to 32 bit mode using timers 2 and 3 together
    T2CONbits.TCKPS = 0; //set the prescaler to 1:1
    T2CONbits.ON = 1; // start the timer
    */

float in[NSAMP];  //these need to be global for the DMCI tool we will use later
float outR[NSAMP];
float outI[NSAMP];
float mag[NSAMP];

    //int in,out1,out2 [1000];
    int f = 2;// f can have values from 2 to 8
    int n = 0;
    for(n = 0; n < NSAMP + 1 ; n++)
    {
        //in[n] = 200 * sin(f * 2 * pi /NSAMP * n);
        in[n] = 200 * sin(f * 2 * M_PI /NSAMP * n);
    }

    WriteTimer23(0); //make sure the timer value is 0 before we start
    T2CONbits.ON = 1; // start the timer
    //the equation we want to find the length of is fft_float
    fft_float(
            NSAMP, // number of samples
            0, // forward FFT
            in, //real in
            0,  //imaginary in
            outR,   //real out
            outI    //imaginary out
        );
    time = ReadTimer23();

    for(n = 0; n < NSAMP +1 ; n++)
    {
        //mag[n] = sqrt(outR[n]^2 + outI[n]^2);
        mag[n] = sqrt ( ( outR[n] * outR[n] ) + ( outI[n] * outI[n] ) );
    }
    

    while (1);
}



