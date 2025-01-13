#ifdef __XC8
#include <xc.h>
#include <pic18f4520.h>
#define bit __bit
#define bool _Bool
#else
#define __interrupt(priority)
#include "C:/Program Files/Microchip/xc8/v2.50/pic/include/proc/pic18f4520.h"
#define bit unsigned char
#define bool unsigned char
#endif

#include <stdio.h>

#define false 0b0
#define true 0b1

#define uint16_t unsigned short
#define byte unsigned char

#define STR(x) #x
#define XSTR(s) STR(s)
#define MACRO_CODE_CONCAT(A, B) A##B
#define MACRO_CODE_CONCAT3(A, B, C) A##B##C
#define _pinGetPortBits(reg, port, pin) MACRO_CODE_CONCAT3(reg, port, bits)
#define _pinGetPinBit(reg, port, pin) MACRO_CODE_CONCAT(reg, pin)

#define AD_CLOCK_SOURCE_2TOSC 0b000   // Max device frequance: 2.86 MHz
#define AD_CLOCK_SOURCE_4TOSC 0b100   // Max device freqOscillator Switchover mode disableduance: 5.71 MHz
#define AD_CLOCK_SOURCE_8TOSC 0b001   // Max device frequance: 11.43 MHz
#define AD_CLOCK_SOURCE_16TOSC 0b101  // Max device frequance: 22.86 MHz
#define AD_CLOCK_SOURCE_32TOSC 0b010  // Max device frequance: 40.0 MHz
#define AD_CLOCK_SOURCE_64TOSC 0b110  // Max device frequance: 40.0 MHz
#define AD_CLOCK_SOURCE_RC 0b011      // Max device frequance: 1.00 MHz

// Internal Oscillator Frequency Select bits
#define INTERNAL_CLOCK_8MHz 0b111    // 8 MHz (INTOSC drives clock directly)
#define INTERNAL_CLOCK_4MHz 0b110    // 4 MHz
#define INTERNAL_CLOCK_2MHz 0b101    // 2 MHz
#define INTERNAL_CLOCK_1MHz 0b100    // 1 MHz(3)
#define INTERNAL_CLOCK_500kHz 0b011  // 500 kHz
#define INTERNAL_CLOCK_250kHz 0b010  // 250 kHz
#define INTERNAL_CLOCK_125kHz 0b001  // 125 kHz
#define INTERNAL_CLOCK_31kHz 0b000   // 31 kHz (from either INTOSC/256 or INTRC directly)(2)
#if (_XTAL_FREQ == 32000000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_8MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_32TOSC
#define PLL_ENABLE
#elif (_XTAL_FREQ == 16000000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_4MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_16TOSC
#define PLL_ENABLE
#elif (_XTAL_FREQ == 8000000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_8MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_8TOSC
#elif (_XTAL_FREQ == 4000000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_4MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_4TOSC
#elif (_XTAL_FREQ == 2000000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_2MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 1000000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_1MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 500000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_500kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 250000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_250kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 125000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_125kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 31000)
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_31kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#else
#error Illegal internal clock speed is set in "_XTAL_FREQ", please change it.
#endif

#ifdef PLL_ENABLE
/**
 * OSCCON: OSCILLATOR CONTROL REGISTER
 * https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=30
 * */
#define setIntrnalClock()                  \
    OSCCONbits.IRCF = INTERNAL_CLOCK_IRCF; \
    Nop();                                 \
    OSCTUNEbits.PLLEN = 0b1
#else
/**
 * OSCCON: OSCILLATOR CONTROL REGISTER
 * https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=30
 * */
#define setIntrnalClock() OSCCONbits.IRCF = INTERNAL_CLOCK_IRCF
#endif

#pragma region AD_AcquisitionTime
/**
 * A/D Acquisition Time https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=230
 * Nano second
 */
#define MINIMUM_ACQUISITION_TIME 2400

#if (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_2TOSC)
#define _AD_CONVETER_TAD 2 * 1000000000 / _XTAL_FREQ
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_4TOSC)
#define _AD_CONVETER_TAD 4 * 1000000000 / _XTAL_FREQ
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_8TOSC)
#define _AD_CONVETER_TAD 8 * 1000000000 / _XTAL_FREQ
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_16TOSC)
#define _AD_CONVETER_TAD 12 * 1000000000 / _XTAL_FREQ
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_32TOSC)
#define _AD_CONVETER_TAD 32 * 1000000000 / _XTAL_FREQ
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_64TOSC)
#define _AD_CONVETER_TAD 64 * 1000000000 / _XTAL_FREQ
#else
#error IDK what RC is
#endif

#define AD_ACQUISITION_TIME_0TAD 0b000
#define AD_ACQUISITION_TIME_2TAD 0b001
#define AD_ACQUISITION_TIME_4TAD 0b010
#define AD_ACQUISITION_TIME_6TAD 0b011
#define AD_ACQUISITION_TIME_8TAD 0b100
#define AD_ACQUISITION_TIME_12TAD 0b101
#define AD_ACQUISITION_TIME_16TAD 0b110
#define AD_ACQUISITION_TIME_20TAD 0b111
#if (2 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_2TAD
#elif (4 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_4TAD
#elif (6 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_6TAD
#elif (8 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_8TAD
#elif (12 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_12TAD
#elif (16 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_16TAD
#elif (20 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_20TAD
#endif
#pragma endregion AD_AcquisitionTime

#pragma region Timer0
#define TIMER0_PRESCALE_2 0b000
#define TIMER0_PRESCALE_4 0b001
#define TIMER0_PRESCALE_8 0b010
#define TIMER0_PRESCALE_16 0b011
#define TIMER0_PRESCALE_32 0b100
#define TIMER0_PRESCALE_64 0b101
#define TIMER0_PRESCALE_128 0b110
#define TIMER0_PRESCALE_256 0b111

#define TIMER0_PRESCALE_ENABLE 0b0
#define TIMER0_PRESCALE_DISABLE 0b1

#define TIMER0_CLOCK_SOURCE_T0CKI_PIN 0b1
#define TIMER0_CLOCK_SOURCE_INTERNAL 0b0

#define TIMER0_MODE_8BIT 0b1
#define TIMER0_MODE_16BIT 0b0
/**
 * T0CON: TIMER0 CONTROL REGISTER
 * https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=125
 */
#define enableTimer0(prescale, prescaleEnable, clockSource, mode) \
    T0CONbits.TMR0ON = 0b1;                                       \
    T0CONbits.T08BIT = mode;                                      \
    T0CONbits.T0CS = clockSource;                                 \
    T0CONbits.PSA = prescaleEnable;                               \
    T0CONbits.T0PS = prescale;
#define disableTimer0() T0CONbits.TMR0ON = 0b0
#define clearInterrupt_Timer0Overflow() INTCONbits.TMR0IF = 0b0
#define enableInterrupt_Timer0Overflow(priority) \
    /*TMR0 Overflow Interrupt Enable bit*/       \
    INTCONbits.TMR0IE = 0b1;                     \
    /*TMR0 Priority */                           \
    INTCON2bits.TMR0IP = priority;               \
    clearInterrupt_Timer0Overflow()
#define interruptByTimer0Overflow() INTCONbits.TMR0IF

/**
 * @param period µs
 * @param prescale 2, 4, 6, 8, 16, 32, 64, 128, 256
 */
#define setTimer0InterruptPeriod8(period, prescale) TMR0 = (byte)(255 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

/**
 * @param period µs
 * @param prescale 2, 4, 6, 8, 16, 32, 64, 128, 256
 */
#define setTimer0InterruptPeriod16(period, prescale) TMR0 = (unsigned short)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

#pragma endregion Timer0

#pragma region Timer1
#define TIMER1_PRESCALE_1 0b00
#define TIMER1_PRESCALE_2 0b01
#define TIMER1_PRESCALE_4 0b10
#define TIMER1_PRESCALE_8 0b11
/**
 * T1CON: TIMER1 CONTROL REGISTER
 * https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=129
 */
#define enableTimer1(prescale)   \
    T1CONbits.RD16 = 1;          \
    T1CONbits.T1CKPS = prescale; \
    T1CONbits.TMR1ON = 0b1
#define configTimer1(prescale) \
    T1CONbits.RD16 = 1;        \
    T1CONbits.T1CKPS = prescale
#define enableTimer1bit() T1CONbits.TMR1ON = 0b1
#define disableTimer1() T1CONbits.TMR1ON = 0b0
#define clearInterrupt_Timer1Overflow() PIR1bits.TMR1IF = 0b0
#define enableInterrupt_Timer1Overflow(priority) \
    /*TMR1 Overflow Interrupt Enable */          \
    PIE1bits.TMR1IE = 0b1;                       \
    /*TMR1 Priority */                           \
    IPR1bits.TMR1IP = priority;                  \
    clearInterrupt_Timer1Overflow()
#define interruptByTimer1Overflow() PIR1bits.TMR1IF
/**
 * @param period µs
 * @param prescale 1, 2, 4, 8
 */
#define setTimer1InterruptPeriod(period, prescale) TMR1 = (unsigned short)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)
#pragma endregion Timer1

#pragma region Timer2
#define TIMER2_PRESCALE_1 0b00
#define TIMER2_PRESCALE_4 0b01
#define TIMER2_PRESCALE_16 0b10
/**
 * T2CON: TIMER2 CONTROL REGISTER
 * https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=135
 * @param poscaleBits 4bit
 */
#define enableTimer2(prescale, poscaleBits) \
    T2CONbits.T2CKPS = prescale;            \
    T2CONbits.T2OUTPS = poscaleBits;        \
    T2CONbits.TMR2ON = 0b1
#define configTimer2(prescale, poscaleBits) \
    T2CONbits.T2CKPS = prescale;            \
    T2CONbits.T2OUTPS = poscaleBits
#define enableTimer2bit() T2CONbits.TMR2ON = 0b1
#define disableTimer2() T2CONbits.TMR2ON = 0b0
#define clearInterrupt_Timer2PR2() PIR1bits.TMR2IF = 0b0
#define enableInterrupt_Timer2PR2(priority) \
    /*TMR2 to PR2 Match Interrupt Enable*/  \
    PIE1bits.TMR2IE = 0b1;                  \
    /*TMR2 Priority */                      \
    IPR1bits.TMR2IP = priority;             \
    clearInterrupt_Timer2PR2()
#define interruptByTimer2PR2() PIR1bits.TMR2IF

#define disableInterrupt_Timer2PR2() \
    PIE1bits.TMR2IE = 0b0;           \
    clearInterrupt_Timer2PR2()

/**
 * @param period µs
 * @param prescale 1, 4, 16
 */
#define setTimer2InterruptPeriod(period, prescale, postscale) PR2 = (byte)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale / postscale - 1)

#pragma endregion Timer2

#pragma region Timer3
#define TIMER3_PRESCALE_1 0b00
#define TIMER3_PRESCALE_2 0b01
#define TIMER3_PRESCALE_4 0b10
#define TIMER3_PRESCALE_8 0b11
/**
 * T3CON: TIMER3 CONTROL REGISTER
 * https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=137
 */
#define enableTimer3(prescale) \
    T3CONbits.TMR3ON = 0b1;    \
    T3CONbits.T3CKPS = prescale;
#define disableTimer3() T3CONbits.TMR3ON = 0b0
#define clearInterrupt_Timer3Overflow() PIR2bits.TMR3IF = 0b0
#define enableInterrupt_Timer3Overflow(priority) \
    /*TMR1 Overflow Interrupt Enable */          \
    PIE2bits.TMR3IE = 0b1;                       \
    /*TMR1 Priority */                           \
    IPR2bits.TMR3IP = priority;                  \
    clearInterrupt_Timer3Overflow()
#define interruptByTimer3Overflow() PIR2bits.TMR3IF

/**
 * @param period µs
 * @param prescale 1, 2, 4, 8
 */
#define setTimer3InterruptPeriod(period, prescale) TMR3 = (unsigned short)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

#pragma endregion Timer3

#pragma region PWM_Control

#define ECCP_MODE_OFF 0b0000          // Capture/Compare/PWM off (resets ECCP module)
#define ECCP_MODE_RESERVED 0b0001     // Reserved
#define ECCP_MODE_COMPARE_TOM 0b0010  // Compare mode, toggle output on match
#define ECCP_MODE_CAPTURE 0b0011      // Capture mode
#define ECCP_MODE_CAPTURE_EFE 0b0100  // Capture mode, every falling edge
#define ECCP_MODE_CAPTURE_ERE 0b0101  // Capture mode, every rising edge
#define ECCP_MODE_CAPTURE_R04 0b0110  // Capture mode, every 4th rising edge
#define ECCP_MODE_CAPTURE_R16 0b0111  // Capture mode, every 16th rising edge
#define ECCP_MODE_COMPARE_SOM 0b1000  // Compare mode, initialize CCP1 pin low; set output on compare match (set CCP1IF)
#define ECCP_MODE_COMPARE_COM 0b1001  // Compare mode, initialize CCP1 pin high; clear output on compare match (set CCP1IF)
#define ECCP_MODE_COMPARE_RIO 0b1010  // Compare mode, generate software interrupt only; CCP1 pin reverts to I/O state
#define ECCP_MODE_COMPARE_TSE 0b1011  // Compare mode, trigger special event (ECCP resets TMR1 or TMR3, sets CCP1IF bit)
#define ECCP_MODE_PWM_HH 0b1100       // PWM mode, P1A, P1C active-high; P1B, P1D active-high
#define ECCP_MODE_PWM_HL 0b1101       // PWM mode, P1A, P1C active-high; P1B, P1D active-low
#define ECCP_MODE_PWM_LH 0b1110       // PWM mode, P1A, P1C active-low; P1B, P1D active-high
#define ECCP_MODE_PWM_LL 0b1111       // PWM mode, P1A, P1C active-low; P1B, P1D active-low
/**
 * ENHANCED CAPTURE/COMPARE/PWM (ECCP) MODULE
 * CCP1CON: ECCP CONTROL REGISTER (40/44-PIN DEVICES)
 */
#define setCCP1Mode(eccpMode) CCP1CONbits.CCP1M = eccpMode
#define setCCP2Mode(eccpMode) CCP2CONbits.CCP2M = eccpMode

/**
 * @param length µs
 * @param prescale 1, 4, 16
 */
#define setCCP1PwmDutyCycle(length, prescale)                                                \
    do {                                                                                     \
        unsigned int value = (unsigned int)((length) / (1000000.0 / _XTAL_FREQ) / prescale); \
        CCP1CONbits.DC1B = (byte)value & 0b11;                                               \
        CCPR1L = (byte)(value >> 2);                                                         \
    } while (0)
/**
 * @param length µs
 * @param prescale 1, 4, 16
 */
#define setCCP2PwmDutyCycle(length, prescale)                                                \
    do {                                                                                     \
        unsigned int value = (unsigned int)((length) / (1000000.0 / _XTAL_FREQ) / prescale); \
        CCP2CONbits.DC2B = (byte)value & 0b11;                                               \
        CCPR2L = (byte)(value >> 2);                                                         \
    } while (0)

#pragma endregion PWM_Control

#pragma region PIC18F4520_Pins
#define PIN_MCLR       // 1    MCLR / VPP / RE3
#define PIN_RA0 A, A0  // 2    RA0 / AN0
#define PIN_RA1 A, A1  // 3    RA1 / AN1
#define PIN_RA2 A, A2  // 4    RA2 / AN2 / VREF- / CVREF
#define PIN_RA3 A, A3  // 5    RA3 / AN3 / VREF+
#define PIN_RA4 A, A4  // 6    RA4 / T0CKI / C1OUT
#define PIN_RA5 A, A5  // 7    RA5 / AN4 / SS / HLVDIN / C2OUT
#define PIN_RE0 E, E0  // 8    RE0 / RD / AN5
#define PIN_RE1 E, E1  // 9    RE1 / WR / AN6
#define PIN_RE2 E, E2  // 10   RE2 / CS / AN7
#define PIN_OSC1       // 13   OSC1 / CLKI / RA7
#define PIN_OSC2       // 14   OSC2 / CLKO / RA6
#define PIN_RC0 C, C0  // 15   RC0 / T1OSO / T13CKI
#define PIN_RC1 C, C1  // 16   RC1 / T1OSI / CCP2
#define PIN_RC2 C, C2  // 17   RC2 / CCP1 / P1A
#define PIN_RC3 C, C3  // 18   RC3 / SCK / SCL
#define PIN_RD0 D, D0  // 19   RD0 / PSP0
#define PIN_RD1 D, D1  // 20   RD1 / PSP1
#define PIN_RD2 D, D2  // 21   RD2 / PSP2
#define PIN_RD3 D, D3  // 22   RD3 / PSP3
#define PIN_RC4 C, C4  // 23   RC4 / SDI / SDA
#define PIN_RC5 C, C5  // 24   RC5 / SDO
#define PIN_RC6 C, C6  // 25   RC6 / TX / CK
#define PIN_RC7 C, C7  // 26   RC7 / RX / DT
#define PIN_RD4 D, D4  // 27   RD4 / PSP4
#define PIN_RD5 D, D5  // 28   RD5 / PSP5 / P1B
#define PIN_RD6 D, D6  // 29   RD6 / PSP6 / P1C
#define PIN_RD7 D, D7  // 30   RD7 / PSP7 / P1D
#define PIN_RB0 B, B0  // 33   RB0 / INT0 / FLT0 / AN12
#define PIN_RB1 B, B1  // 34   RB1 / INT1 / AN10
#define PIN_RB2 B, B2  // 35   RB2 / INT2 / AN8
#define PIN_RB3 B, B3  // 36   RB3 / AN9 / CCP2
#define PIN_RB4 B, B4  // 37   RB4 / KBI0 / AN11
#define PIN_RB5 B, B5  // 38   RB5 / KBI1 / PGM
#define PIN_RB6 B, B6  // 39   RB6 / KBI2 / PGC
#define PIN_RB7 B, B7  // 40   RB7 / KBI3 / PGD
#pragma endregion PIC18F4520_Pins

#pragma region PinControl
#define PIN_INPUT 0b1
#define PIN_OUTPUT 0b0
#define pinMode(pin, mode) _pinGetPortBits(TRIS, pin)._pinGetPinBit(R, pin) = mode
#define digitalWrite(pin, value) _pinGetPortBits(LAT, pin)._pinGetPinBit(L, pin) = value
#define pinState(pin) _pinGetPortBits(LAT, pin)._pinGetPinBit(L, pin)

#define PORTB_PULLUP_ENABLE 0b0
#define PORTB_PULLUP_DISABLE 0b1
// Set PORTB internal pullup
#define setPortBPullup(state) INTCON2bits.RBPU = state
#pragma endregion PinControl

#pragma region AD_Control
// https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=228
/**
 * ADCON1: A/D CONTROL REGISTER 1, PCFG<3:0>: A/D Port Configuration Control bits
 * https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=226
 * @param value 4bit, 0 = analog, 1 = digital
 */
#define setANPinADConfig(value) ADCON1bits.PCFG = value
/**
 * Set voltage reference to AN2,AN3 pin or use chip input voltage
 * VCFG1: Voltage Reference Configuration bit (VREF- source)
 * - 1 = VREF- (AN2)
 * - 0 = VSS
 *
 * VCFG0: Voltage Reference Configuration bit (VREF+ source)
 * - 1 = VREF+ (AN3)
 * - 0 = VDD
 */
#define setANPinVoltageReferenceConfig(conf0, conf1) \
    ADCON1bits.VCFG1 = conf0;                        \
    ADCON1bits.VCFG0 = conf1
/**
 * Select AN-pin to convert signal from analog to digital
 * @param value 4bit
 */
#define setANPinAnalogChannelSelect(value) ADCON0bits.CHS = value
/**
 * GO: A/D Conversion Status bit
 */
#define startADConverter() ADCON0bits.GO = 1;
#define getADConverter() ADRES

/**
 * ADON: A/D On bit
 * format 1 = Right justified, 0 = Left justified
 */
#define enableADConverter()                                                   \
    ADCON0bits.ADON = 0b1;                /* A/D On */                        \
    ADCON2bits.ADFM = 0b1;                /* A/D Result Format Select */      \
    ADCON2bits.ADCS = AD_CLOCK_SOURCE;    /* Set A/D Converter Clock Source*/ \
    ADCON2bits.ACQT = AD_ACQUISITION_TIME /* Set A/D Converter acquisition time*/

#define clearInterrupt_ADConverter() PIR1bits.ADIF = 0
#define enableInterrupt_ADConverter(priority) \
    clearInterrupt_ADConverter();             \
    /*A/D Converter Interrupt Enable*/        \
    PIE1bits.ADIE = 1;                        \
    /*A/D Converter Interrupt Priority */     \
    IPR1bits.ADIP = priority
#define interruptByADConverter() PIR1bits.ADIF

#pragma endregion AD_Control

#pragma region InterruptControl
/**
 * Enable priority levels on interrupts
 * IPEN: Interrupt Priority Enable bit
 * @param state 1 = Enable, 0 = Disable
 */
#define enableInterruptPriorityMode(state) RCONbits.IPEN = state

/**
 * Global Interrupt Enable bit
 * When IPEN = 0:
 * - 1 = Enables all unmasked interrupts
 * - 0 = Disables all interrupts
 *
 * When IPEN = 1:
 * - 1 = Enables all high-priority interrupts
 * - 0 = Disables all interrupts
 */
#define enableGlobalInterrupt(state) INTCONbits.GIE = state
/**
 * Peripheral Interrupt Enable bit
 * When IPEN = 0:
 * - 1 = Enables all unmasked peripheral interrupts
 * - 0 = Disables all peripheral interrupts
 *
 * When IPEN = 1:
 * - 1 = Enables all low-priority peripheral interrupts
 * - 0 = Disables all low-priority peripheral interrupts
 */
#define enablePeripheralInterrupt(state) INTCONbits.PEIE = state

#define clearInterrupt_RB0External() INTCONbits.INT0IF = 0b0
// The RB0/INT0 external interrupt occurred (must be cleared in software)
#define enableInterrupt_RB0External() \
    clearInterrupt_RB0External();     \
    INTCONbits.INT0IE = 0b1;
#define interruptByRB0External() INTCONbits.INT0IF

#define clearInterrupt_RB1External() INTCON3bits.INT1IF = 0b0
// The RB1/INT1 external interrupt occurred (must be cleared in software)
#define enableInterrupt_RB1External(priority)    \
    clearInterrupt_RB1External();                \
    INTCON3bits.INT1IE = 0b1;                    \
    /*RB1/INT1 External Interrupt Priority bit*/ \
    INTCON3bits.INT1IP = priority;

#define interruptByRB1External() INTCON3bits.INT1IF

#define clearInterrupt_RB2External() INTCON3bits.INT2IF = 0b0
// The RB1/INT1 external interrupt occurred (must be cleared in software)
#define enableInterrupt_RB2External(priority)    \
    clearInterrupt_RB2External();                \
    INTCON3bits.INT2IE = 0b1;                    \
    /*RB1/INT1 External Interrupt Priority bit*/ \
    INTCON3bits.INT2IP = priority

#define interruptByRB2External() INTCON3bits.INT2IF

#define clearInterrupt_RBPortChange() INTCONbits.RBIF = 0b0
// At least one of the RB<7:4> pins changed state (must be cleared in software)
#define enableInterrupt_RBPortChange(priority) \
    clearInterrupt_RBPortChange();             \
    /*RB Port Change Interrupt Enable bit*/    \
    INTCONbits.RBIE = 0b1;                     \
    /*RB Port Change Interrupt Priority bit*/  \
    INTCON2bits.RBIP = priority

#define interruptByRBPortChange() INTCONbits.RBIF

#define clearInterrupt_TransmitUART() PIR1bits.TXIF = 0b0
#define enableInterrupt_TransmitUART(priority) \
    clearInterrupt_TransmitUART();             \
    /*EUSART Transmit Interrupt Enable bit*/   \
    PIE1bits.TXIE = 0b1;                       \
    /*EUSART Transmit Interrupt Priority bit*/ \
    IPR1bits.TXIP = priority

#define interruptByTransmitUART() PIR1bits.TXIF

#define clearInterrupt_ReceiveUART() PIR1bits.RCIF = 0b0
#define enableInterrupt_ReceiveUART(priority) \
    clearInterrupt_ReceiveUART();             \
    /*EUSART Receive Interrupt Enable bit*/   \
    PIE1bits.RCIE = 0b1;                      \
    /*EUSART Receive Interrupt Priority bit*/ \
    IPR1bits.RCIP = priority
#define interruptByReceiveUART() PIR1bits.RCIF

#pragma endregion InterruptControl

#pragma region UART
#ifndef SEIAL_PRINTF_STATIC_SIZE
#define SEIAL_PRINTF_STATIC_SIZE 16
#endif
char serialPrintfCache[SEIAL_PRINTF_STATIC_SIZE];

#define serialReceiveEnable(state) RCSTAbits.CREN = state
/**
 * Overrun error (can be cleared by clearing bit, CREN)
 */
#define serialReceiveOverrunError() RCSTAbits.OERR
/**
 *  Framing error (can be cleared by reading RCREG register and receiving next valid byte)
 */
#define serialReceiveFramingError() RCSTAbits.FERR

inline void serialBegin(long baudRate, byte receiveInterruptPriority) {
    pinMode(PIN_RC6, PIN_OUTPUT);  // RC6(TX)
    pinMode(PIN_RC7, PIN_INPUT);   // RC7(RX)

    // Setting Baud rate
    TXSTAbits.SYNC = 0;     // Asynchronus
    BAUDCONbits.BRG16 = 1;  // 16 bits or 8 bits

    long baudRateGenerator = _XTAL_FREQ / baudRate;
    TXSTAbits.BRGH = baudRate > 2400;  // High Baud Rate Select bit

    // https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=207
    if (!TXSTAbits.SYNC && !BAUDCONbits.BRG16 && !TXSTAbits.BRGH)
        baudRateGenerator /= 64;
    else if (!TXSTAbits.SYNC && BAUDCONbits.BRG16 != TXSTAbits.BRGH)
        baudRateGenerator /= 16;
    else
        baudRateGenerator /= 4;

    // Baud rate = 1200 (Look up table)
    SPBRGH = (byte)(baudRateGenerator >> 8);
    SPBRG = (byte)baudRateGenerator;

    // Serial enable
    RCSTAbits.SPEN = 1;  // Enable serial port (configures RX/DT and TX/CK pins as serial port pins)
    TXSTAbits.TXEN = 1;  // Enable transmission
    RCSTAbits.CREN = 1;  // Continuous receive enable bit, will be cleared when error occured
    enableInterrupt_ReceiveUART(receiveInterruptPriority);

    /* Transmitter (output)
     TSR   : Current Data
     TXREG : Next Data
     TXSTAbits.TRMT will set when TSR is empty
    */
    /* Reiceiver (input)
     RSR   : Current Data
     RCREG : Correct Data (have been processed) : read data by reading the RCREG Register
    */
}

#define serialAvailableForWrite() TXSTAbits.TRMT

void serialWrite(char c) {
    while (!serialAvailableForWrite());  // Busy Waiting
    TXREG = c;                           // write to TXREG will send data
}

void serialPrint(char *text) {
    for (int i = 0; text[i] != '\0'; i++) {
        while (!serialAvailableForWrite());  // Busy Waiting
        TXREG = text[i];                     // write to TXREG will send data
    }
}

#define serialPrintf(format, ...)                    \
    sprintf(serialPrintfCache, format, __VA_ARGS__); \
    serialPrint(serialPrintfCache)

char serialRead() {
    while (!interruptByReceiveUART());
    return RCREG;
}

void (*serialOnReadLine)(char *line, byte len);
void (*serialOnReadChar)(char c);
char serialBuffer[64];
byte serialBufferLen = 0;
char serialLastChar = '\0';

bool processSerialReceive() {
    if (interruptByReceiveUART()) {
        // Clear overrun error
        if (serialReceiveOverrunError()) {
            serialReceiveEnable(0);
            Nop();
            serialReceiveEnable(1);
        }
        char c = serialRead();
        // Skip if framing error
        if (!serialReceiveFramingError()) {
            switch (c) {
            case '\x7f':
                if (!serialBufferLen)
                    break;
                // Backspace
                serialWrite('\b');
                serialWrite(' ');
                serialWrite('\b');
                serialBuffer[--serialBufferLen] = '\0';
                break;
            case '\r':
            case '\n':
                // Use CRLF, ignore CR
                if (c == '\n' && !serialBufferLen && serialLastChar == '\r')
                    break;
                serialWrite('\n');
                serialBuffer[serialBufferLen] = '\0';
                if (serialOnReadLine)
                    serialOnReadLine(serialBuffer, serialBufferLen);
                serialBuffer[serialBufferLen = 0] = '\0';
                break;
            case 0xff:
                break;
            default:
                if (serialOnReadChar)
                    serialOnReadChar(c);
                // Check serial buffer size
                if (serialBufferLen < (byte)sizeof(serialBuffer) - 1) {
                    serialWrite(c);
                    serialBuffer[serialBufferLen++] = c;
                }
            }
            serialLastChar = c;
        }
        return true;
    }
    return false;
}

#pragma endregion UART

/**
 * @param angle 0~180 deg
 */
#define setCCP1ServoAngle(angle, prescale) setCCP1PwmDutyCycle(450 + ((2450 - 450) / 180.0) * (angle), prescale)
