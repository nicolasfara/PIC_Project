/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1826
        Driver Version    :  2.00
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

#include "mcc_generated_files/mcc.h"

#define Display PORTB

/*
                         Main application
 */

const unsigned char LedTable[] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111, 119, 124, 57, 94, 121, 113};

uint8_t Sec = 0, MSec = 0, change = 0, DSec = 0, USec = 0;

void main(void) {
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    TMR0_Initialize();//inizzializzo Timer0
    TMR1_Initialize();//inizzializzo Timer1

    while (1) {
       
        //Controllo se il Timer1 ha generato un overflow (overflow ogni 100ms)
        if(TMR1_HasOverflowOccured()){
            
            TMR1IF = 0; //azzero il bit di flag del Timer1
            
            MSec++; //incremento la variabile dei milli secondi
            //ogni 10x100ms = 1s
            if(MSec == 10){
                MSec = 0; //azzero i milli secondi per rincominciare a contare
                Sec++; // incremento la variabile dei secondi
                //se secondi = 60 azzero variabile (trascorso 1 minuto)
                if(Sec == 60)
                    Sec = 0;
            }
            
            TMR1_Reload(); //ricarico il Timer 1
        }
        
        DSec = Sec / 10; //Variabile per visualizzare le decine dei secondi
        USec = Sec % 10; //Variabile per visualizzare le unita dei secondi
        
        //-----------------GESTIONE MULTIPLEXING DISPLAY-----------------------//
        //Controllo se il Timer0 ha generato l'overflow        
        if(TMR0_HasOverflowOccured()){
            
            TMR0IF = 0; //Azzero il bit flag del Timer0
            //Switch case per il multiplexaggio dei display
            switch(change){
                //Abilitazione display delle decine secondi
                case 0:
                    Display = 0; //prevengo effetto "ghost" sui display
                    K1_SetHigh(); //abilito il catodo del display
                    K2_SetLow(); //disabilito quello delle unità
                    Display = LedTable[DSec]; //visualizzo il dato sul display
                    change = 1; //abilito la visualizzazione del display successivo
                    break; //esco dal ciclo
                    
                case 1:
                    Display = 0;
                    K1_SetLow();
                    K2_SetHigh();
                    Display = LedTable[USec] | 0x80; //oltre a visualizzare il dato accendo il punto decimale
                    change = 2;
                    break;
                    
                case 2:
                    Display = 0;
                    K1_SetLow(); //essendoci il diodo sul catoto di questo display disabilito gli altri due e basta
                    K2_SetLow();
                    Display = LedTable[MSec];
                    change = 0; //rincomincio ciclo di visualizzazione
                    break;
                
            }
            
            TMR0_Reload(); //ricarico Timer0
        }                       
    }
}
/**
 End of File
 */