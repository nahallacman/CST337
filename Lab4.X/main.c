/*
 * File:   main.c
 * Author: cal
 *
 * Created on November 8, 2014, 3:19 AM
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

// Memory Allocation and Initialization Test

#include <xc.h>
#include <string.h>

const char a[] = "CST 321 Lab 4";
char b[] =       "Initialized Global Var";
const char c[] = "Initialized Constant String";
char d[100] =    "Initialized String Array";
unsigned int gc = 0; //global initialized to 0
unsigned int gd; //global uninitialized
unsigned int ge = 0x45; //global initialized to 0x45

int main(void)
    {
    unsigned int ic = 0x0F1E2D3C;
    unsigned int id = 1;
    unsigned int ie;
    const unsigned int icf = 0x98765432;
    static unsigned int isf = 0x67452301;
    static const unsigned int iscf = 0xABCDEFFE;

    unsigned long long lc = 0x4B5AA9788796A5B4LL;
    static const unsigned long long lcc = 0xFEDCBA9876543210LL;
    unsigned long long ld = 1;
    unsigned long long le;

    char la[] = "Local String Test";

    ie = 0x19283746;
    ic = icf;
    ic = isf;
    ic = iscf;
    id = gc;

    le = 0x0123456789ABCDEFLL;

    strcpy(d,b);
    strcpy(d,a);
    strcpy(d,c);
    strcpy(b,c);

    char * e;
    //allocate 14 bytes to copy from a with
    e = (char*)malloc( 13 * sizeof(char) );
    strcpy(e , a);


    while(1);
}

