#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <builtins.h>

#define _XTAL_FREQ 4000000  // Internal Clock speed
#include "lib.h"

#pragma config OSC = INTIO67  // Oscillator Selection bits
#pragma config WDT = OFF      // Watchdog Timer Enable bit
#pragma config PWRT = OFF     // Power-up Enable bit
#pragma config BOREN = ON     // Brown-out Reset Enable bit
#pragma config PBADEN = OFF   // Watchdog Timer Enable bit
#pragma config LVP = OFF      // Low Voltage (single -supply) In-Circute Serial Pragramming Enable bit
#pragma config CPD = OFF      // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)

int flash_state = 0;
// void onReadLine(char *line, byte len) {
// }

// void onReadChar(char c) {
// }

void __interrupt(high_priority) H_ISR() {
    if (interruptByRB0External()) {
        clearInterrupt_RB0External();
    }
    // if (interruptByTimer1Overflow()) {
    //     clearInterrupt_Timer1Overflow();
    // }
    // if (interruptByTimer3Overflow()) {
    //     clearInterrupt_Timer3Overflow();
    // }
    if (interruptByADConverter()) {
        flash_state = getADConverter() / 341;
        clearInterrupt_ADConverter();
    }
}

void __interrupt(low_priority) Lo_ISR(void) {
    if (processSerialReceive())
        return;
}

void main(void) {
    setIntrnalClock();
    setPortBPullup(PORTB_PULLUP_ENABLE);

    enableInterruptPriorityMode(1);  // enable the priority interrupt
    enableGlobalInterrupt(1);        // enable the global interrupt
    enablePeripheralInterrupt(1);    // enable peripheral interrupt

    // Button
    pinMode(PIN_RB0, PIN_INPUT);
    enableInterrupt_RB0External();  // enable RB0 interrupt

    // UART
    serialBegin(9600, 0);
    // serialOnReadLine = onReadLine;
    // serialOnReadChar = onReadChar;

    // LED
    pinMode(PIN_RA1, PIN_OUTPUT);  // set the RA1 as LED output
    pinMode(PIN_RA2, PIN_OUTPUT);  // set the RA2 as LED output
    pinMode(PIN_RA3, PIN_OUTPUT);  // set the RA3 as LED output

    digitalWrite(PIN_RA1, 0);
    digitalWrite(PIN_RA2, 0);
    digitalWrite(PIN_RA3, 0);

    // Potentiometer
    pinMode(PIN_RA0, PIN_INPUT);
    setANPinVoltageReferenceConfig(0, 0);
    setANPinADConfig(0b1110);        // AN0 analog
    setANPinAnalogChannelSelect(0);  // AN0 channel select
    enableADConverter();             // enable A/D Converter module
    enableInterrupt_ADConverter(1);  // enable ADC interrupt

    // // Servo
    // pinMode(PIN_RC2, PIN_OUTPUT);             // Set servo pin CCP1 output
    // digitalWrite(PIN_RC2, 0);                 // Set servo pin CCP1 to low
    // enableTimer2(TIMER2_PRESCALE_16, 0b0000); // Enable Timer2 for PWM control
    // setTimer2InterruptPeriod(4100, 16, 1);    // Setup servo report rate, 4.1ms
    // setCCP1Mode(ECCP_MODE_PWM_HH);            // PWM mode, P1A, P1C active-high; P1B, P1D active-high
    // setCCP1ServoAngle(0, 16);                 // Reset angle to 0

    // // Timer1
    // enableTimer1(TIMER1_PRESCALE_4);
    // setTimer1InterruptPeriod(100, 4);   // 100 µs
    // enableInterrupt_Timer1Overflow(1);  // high priority

    // // Timer3
    // enableTimer3(TIMER3_PRESCALE_4);
    // setTimer3InterruptPeriod(100, 4);   // 100 µs
    // enableInterrupt_Timer3Overflow(1);  // high priority

    int count = 0;
    serialPrint("Ready\n");
    char cache[20];
    while (1) {
        startADConverter();
        __delay_us(10);

        switch (flash_state) {
        case 0:
            if (pinState(PIN_RA1)) {
                sprintf(cache, "State1 count=%d\n", count);
                serialPrint(cache);
                count++;
                memset(cache, 0, 20);
            }
            digitalWrite(PIN_RA1, !pinState(PIN_RA1));
            __delay_ms(1000);
            break;
        case 1:
            if (pinState(PIN_RA1)) {
                sprintf(cache, "State2 count=%d\n", count);
                serialPrint(cache);
                count++;
                memset(cache, 0, 20);
            }
            digitalWrite(PIN_RA1, !pinState(PIN_RA1));
            __delay_ms(500);
            break;
        case 2:
            if (pinState(PIN_RA1)) {
                sprintf(cache, "State3 count=%d\n", count);
                serialPrint(cache);
                count++;
                memset(cache, 0, 20);
            }
            digitalWrite(PIN_RA1, !pinState(PIN_RA1));
            __delay_ms(250);
            break;
        }
    }

    return;
}