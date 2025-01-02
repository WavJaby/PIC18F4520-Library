### Timer

#### Functions

```c
// Timer0
enableTimer0(prescale, prescaleEnable, clockSource, mode);
disableTimer0();
clearInterrupt_Timer0Overflow();
enableInterrupt_Timer0Overflow(priority);
interruptByTimer0Overflow();
setTimer0InterruptPeriod8(period, prescale);   // For 8bit mode
setTimer0InterruptPeriod16(period, prescale);  // For 16bit mode

// Timer1
enableTimer1(prescale);
disableTimer1();
clearInterrupt_Timer1Overflow();
enableInterrupt_Timer1Overflow(priority);
interruptByTimer1Overflow();
setTimer1InterruptPeriod(period, prescale);

// Timer2
enableTimer2(prescale, poscaleBits);
disableTimer2();
clearInterrupt_Timer2PR2();
enableInterrupt_Timer2PR2(priority);
interruptByTimer2PR2();
disableInterrupt_Timer2PR2();
setTimer2InterruptPeriod(period, prescale, postscale);

// Timer3
enableTimer3(prescale);
disableTimer3();
clearInterrupt_Timer3Overflow();
enableInterrupt_Timer3Overflow(priority);
interruptByTimer3Overflow();
setTimer3InterruptPeriod(period, prescale);
```

#### Example

```c
#define _XTAL_FREQ 4000000  // Internal Clock speed
#include "lib.h"
/* === Omit configuration settings === */

void __interrupt(high_priority) Lo_ISR(void) {
    if (interruptByTimer1Overflow()) {
        // do stuff...
        clearInterrupt_Timer1Overflow();
    }
    if (interruptByTimer3Overflow()) {
        // do stuff...
        clearInterrupt_Timer3Overflow();
    }
}
void main(void) {
    // System init
    setIntrnalClock();
    setPortBPullup(PORTB_PULLUP_ENABLE);
    enableInterruptPriorityMode(1);  // enable the priority interrupt
    enableGlobalInterrupt(1);        // enable the global interrupt
    enablePeripheralInterrupt(1);    // enable peripheral interrupt

    // Timer1
    enableTimer1(TIMER1_PRESCALE_4);
    setTimer1InterruptPeriod(100, 4);   // 100 µs
    enableInterrupt_Timer1Overflow(1);  // high priority

    // Timer3
    enableTimer3(TIMER3_PRESCALE_4);
    setTimer3InterruptPeriod(200, 4);   // 200 µs
    enableInterrupt_Timer3Overflow(1);  // high priority
}
```

### UART

#### Functions
```c
void serialBegin(long baudRate, byte receiveInterruptPriority);
void serialAvailableForWrite();
void serialWrite(char c);
void serialPrint(char *text);
bool processSerialReceive();
// Listener
void (*serialOnReadLine)(char *line, byte len);
void (*serialOnReadChar)(char c);
```

#### Example

```c
#define _XTAL_FREQ 4000000  // Internal Clock speed
#include "lib.h"
/* === Omit configuration settings === */

void onReadLine(char *line, byte len) {
    if (strncmp("mode0", line, len)) {
        // do stuff...
    }
}
void onReadChar(char c) {
    byte num = c - '0';
}
void __interrupt(low_priority) Lo_ISR(void) {
    if (processSerialReceive()) // Essential for serial read listener
        return;
}
void main(void) {
    // System init
    setIntrnalClock();
    setPortBPullup(PORTB_PULLUP_ENABLE);
    enableInterruptPriorityMode(1);  // enable the priority interrupt
    enableGlobalInterrupt(1);        // enable the global interrupt
    enablePeripheralInterrupt(1);    // enable peripheral interrupt

    // UART Init
    serialBegin(9600, 0b0); // Low priority
    serialOnReadLine = onReadLine;
    serialOnReadChar = onReadChar;
}
```
