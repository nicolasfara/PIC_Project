/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1826
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set Rand aliases
#define Rand_TRIS               TRISA2
#define Rand_LAT                LATA2
#define Rand_PORT               RA2
#define Rand_ANS                ANSA2
#define Rand_SetHigh()    do { LATA2 = 1; } while(0)
#define Rand_SetLow()   do { LATA2 = 0; } while(0)
#define Rand_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define Rand_GetValue()         RA2
#define Rand_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define Rand_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define Rand_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define Rand_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set K1 aliases
#define K1_TRIS               TRISA3
#define K1_LAT                LATA3
#define K1_PORT               RA3
#define K1_ANS                ANSA3
#define K1_SetHigh()    do { LATA3 = 1; } while(0)
#define K1_SetLow()   do { LATA3 = 0; } while(0)
#define K1_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define K1_GetValue()         RA3
#define K1_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define K1_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define K1_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define K1_SetDigitalMode()   do { ANSA3 = 0; } while(0)
// get/set K2 aliases
#define K2_TRIS               TRISA4
#define K2_LAT                LATA4
#define K2_PORT               RA4
#define K2_ANS                ANSA4
#define K2_SetHigh()    do { LATA4 = 1; } while(0)
#define K2_SetLow()   do { LATA4 = 0; } while(0)
#define K2_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define K2_GetValue()         RA4
#define K2_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define K2_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define K2_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define K2_SetDigitalMode()   do { ANSA4 = 0; } while(0)
// get/set D0 aliases
#define D0_TRIS               TRISB0
#define D0_LAT                LATB0
#define D0_PORT               RB0
#define D0_WPU                WPUB0
#define D0_SetHigh()    do { LATB0 = 1; } while(0)
#define D0_SetLow()   do { LATB0 = 0; } while(0)
#define D0_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define D0_GetValue()         RB0
#define D0_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define D0_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define D0_SetPullup()    do { WPUB0 = 1; } while(0)
#define D0_ResetPullup()   do { WPUB0 = 0; } while(0)
// get/set D1 aliases
#define D1_TRIS               TRISB1
#define D1_LAT                LATB1
#define D1_PORT               RB1
#define D1_WPU                WPUB1
#define D1_ANS                ANSB1
#define D1_SetHigh()    do { LATB1 = 1; } while(0)
#define D1_SetLow()   do { LATB1 = 0; } while(0)
#define D1_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define D1_GetValue()         RB1
#define D1_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define D1_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define D1_SetPullup()    do { WPUB1 = 1; } while(0)
#define D1_ResetPullup()   do { WPUB1 = 0; } while(0)
#define D1_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define D1_SetDigitalMode()   do { ANSB1 = 0; } while(0)
// get/set D2 aliases
#define D2_TRIS               TRISB2
#define D2_LAT                LATB2
#define D2_PORT               RB2
#define D2_WPU                WPUB2
#define D2_ANS                ANSB2
#define D2_SetHigh()    do { LATB2 = 1; } while(0)
#define D2_SetLow()   do { LATB2 = 0; } while(0)
#define D2_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define D2_GetValue()         RB2
#define D2_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define D2_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define D2_SetPullup()    do { WPUB2 = 1; } while(0)
#define D2_ResetPullup()   do { WPUB2 = 0; } while(0)
#define D2_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define D2_SetDigitalMode()   do { ANSB2 = 0; } while(0)
// get/set D3 aliases
#define D3_TRIS               TRISB3
#define D3_LAT                LATB3
#define D3_PORT               RB3
#define D3_WPU                WPUB3
#define D3_ANS                ANSB3
#define D3_SetHigh()    do { LATB3 = 1; } while(0)
#define D3_SetLow()   do { LATB3 = 0; } while(0)
#define D3_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define D3_GetValue()         RB3
#define D3_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define D3_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define D3_SetPullup()    do { WPUB3 = 1; } while(0)
#define D3_ResetPullup()   do { WPUB3 = 0; } while(0)
#define D3_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define D3_SetDigitalMode()   do { ANSB3 = 0; } while(0)
// get/set D4 aliases
#define D4_TRIS               TRISB4
#define D4_LAT                LATB4
#define D4_PORT               RB4
#define D4_WPU                WPUB4
#define D4_ANS                ANSB4
#define D4_SetHigh()    do { LATB4 = 1; } while(0)
#define D4_SetLow()   do { LATB4 = 0; } while(0)
#define D4_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define D4_GetValue()         RB4
#define D4_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define D4_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define D4_SetPullup()    do { WPUB4 = 1; } while(0)
#define D4_ResetPullup()   do { WPUB4 = 0; } while(0)
#define D4_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define D4_SetDigitalMode()   do { ANSB4 = 0; } while(0)
// get/set D5 aliases
#define D5_TRIS               TRISB5
#define D5_LAT                LATB5
#define D5_PORT               RB5
#define D5_WPU                WPUB5
#define D5_ANS                ANSB5
#define D5_SetHigh()    do { LATB5 = 1; } while(0)
#define D5_SetLow()   do { LATB5 = 0; } while(0)
#define D5_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define D5_GetValue()         RB5
#define D5_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define D5_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define D5_SetPullup()    do { WPUB5 = 1; } while(0)
#define D5_ResetPullup()   do { WPUB5 = 0; } while(0)
#define D5_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define D5_SetDigitalMode()   do { ANSB5 = 0; } while(0)
// get/set D6 aliases
#define D6_TRIS               TRISB6
#define D6_LAT                LATB6
#define D6_PORT               RB6
#define D6_WPU                WPUB6
#define D6_ANS                ANSB6
#define D6_SetHigh()    do { LATB6 = 1; } while(0)
#define D6_SetLow()   do { LATB6 = 0; } while(0)
#define D6_Toggle()   do { LATB6 = ~LATB6; } while(0)
#define D6_GetValue()         RB6
#define D6_SetDigitalInput()    do { TRISB6 = 1; } while(0)
#define D6_SetDigitalOutput()   do { TRISB6 = 0; } while(0)

#define D6_SetPullup()    do { WPUB6 = 1; } while(0)
#define D6_ResetPullup()   do { WPUB6 = 0; } while(0)
#define D6_SetAnalogMode()   do { ANSB6 = 1; } while(0)
#define D6_SetDigitalMode()   do { ANSB6 = 0; } while(0)
// get/set D7 aliases
#define D7_TRIS               TRISB7
#define D7_LAT                LATB7
#define D7_PORT               RB7
#define D7_WPU                WPUB7
#define D7_ANS                ANSB7
#define D7_SetHigh()    do { LATB7 = 1; } while(0)
#define D7_SetLow()   do { LATB7 = 0; } while(0)
#define D7_Toggle()   do { LATB7 = ~LATB7; } while(0)
#define D7_GetValue()         RB7
#define D7_SetDigitalInput()    do { TRISB7 = 1; } while(0)
#define D7_SetDigitalOutput()   do { TRISB7 = 0; } while(0)

#define D7_SetPullup()    do { WPUB7 = 1; } while(0)
#define D7_ResetPullup()   do { WPUB7 = 0; } while(0)
#define D7_SetAnalogMode()   do { ANSB7 = 1; } while(0)
#define D7_SetDigitalMode()   do { ANSB7 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */