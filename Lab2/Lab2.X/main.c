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

#define M_PI 3.14159265358979323846 // approximation of PI
#define NSAMP 256 //used for Number of SAMPles
#define TPIN (2 * M_PI/NSAMP) 

extern float in[];
extern float outR[];
extern float outI[];
extern float mag[];

//	Function Prototypes
int main(void);

int main(void) {

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

    fft_float(
            NSAMP, // number of samples
            0, // forward FFT
            in, //real in
            0,  //imaginary in
            outR,   //real out
            outI    //imaginary out
        );

    for(n = 0; n < NSAMP +1 ; n++)
    {
        //mag[n] = sqrt(outR[n]^2 + outI[n]^2);
        mag[n] = sqrt ( ( outR[n] * outR[n] ) + ( outI[n] * outI[n] ) );
    }
    

    while (1);
}



