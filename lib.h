/**
 * PIC18F4520 微控制器函式庫
 * PIC18F4520 Microcontroller Library
 *
 * 此標頭檔提供 PIC18F4520 微控制器的完整硬體抽象層
 * 包含：時脈設定、計時器、PWM、ADC、UART、中斷控制等功能
 */

/* ========== 編譯器相容性設定 / Compiler Compatibility ========== */
#ifdef __XC8
// XC8 編譯器環境 - 用於實際燒錄到 PIC 微控制器
#include <xc.h>
#include <pic18f4520.h>
#define bit __bit      // XC8 的位元型別
#ifndef bool
#define bool _Bool     // XC8 的布林型別
#endif
#else
// 非 XC8 環境 (用於 IDE 語法檢查/模擬)
#define __interrupt(priority)  // 空的中斷巨集定義
#include "C:/Program Files/Microchip/xc8/v2.50/pic/include/proc/pic18f4520.h"
#define bit unsigned char   // 模擬位元型別
#define bool unsigned char  // 模擬布林型別
#endif

#include <stdio.h>

/* ========== 基本型別定義 / Basic Type Definitions ========== */
#ifndef false
#define false 0b0              // 布林假值
#endif

#ifndef true
#define true 0b1               // 布林真值
#endif

#ifndef uint16_t
#define uint16_t unsigned short  // 16位元無號整數
#endif

#ifndef byte
#define byte unsigned char       // 8位元無號整數 (位元組)
#endif

/* ========== 巨集輔助工具 / Macro Helpers ========== */
#define STR(x) #x                                                            // 字串化巨集
#define XSTR(s) STR(s)                                                       // 展開後字串化
#define MACRO_CODE_CONCAT(A, B) A##B                                         // 連接兩個符號
#define MACRO_CODE_CONCAT3(A, B, C) A##B##C                                  // 連接三個符號
#define _pinGetPortBits(reg, port, pin) MACRO_CODE_CONCAT3(reg, port, bits)  // 取得埠位元結構
#define _pinGetPinBit(reg, port, pin) MACRO_CODE_CONCAT(reg, pin)            // 取得腳位位元

/* ========== ADC 時脈來源設定 / ADC Clock Source Settings ========== */
/**
 * ADC 轉換時脈來源選擇
 * TAD = ADC 轉換一個位元所需的時間
 * 選擇時需考慮裝置主頻率，確保 TAD >= 0.7µs
 */
#define AD_CLOCK_SOURCE_2TOSC 0b000   // FOSC/2  - 最大裝置頻率: 2.86 MHz
#define AD_CLOCK_SOURCE_4TOSC 0b100   // FOSC/4  - 最大裝置頻率: 5.71 MHz
#define AD_CLOCK_SOURCE_8TOSC 0b001   // FOSC/8  - 最大裝置頻率: 11.43 MHz
#define AD_CLOCK_SOURCE_16TOSC 0b101  // FOSC/16 - 最大裝置頻率: 22.86 MHz
#define AD_CLOCK_SOURCE_32TOSC 0b010  // FOSC/32 - 最大裝置頻率: 40.0 MHz
#define AD_CLOCK_SOURCE_64TOSC 0b110  // FOSC/64 - 最大裝置頻率: 40.0 MHz
#define AD_CLOCK_SOURCE_RC 0b011      // 內部 RC 振盪器 - 最大裝置頻率: 1.00 MHz

/* ========== 內部振盪器頻率選擇 / Internal Oscillator Frequency ========== */
/**
 * IRCF<2:0>: 內部振盪器頻率選擇位元
 * 用於設定 OSCCON 暫存器的 IRCF 位元
 */
#define INTERNAL_CLOCK_8MHz 0b111    // 8 MHz (INTOSC 直接驅動時脈)
#define INTERNAL_CLOCK_4MHz 0b110    // 4 MHz
#define INTERNAL_CLOCK_2MHz 0b101    // 2 MHz
#define INTERNAL_CLOCK_1MHz 0b100    // 1 MHz
#define INTERNAL_CLOCK_500kHz 0b011  // 500 kHz
#define INTERNAL_CLOCK_250kHz 0b010  // 250 kHz
#define INTERNAL_CLOCK_125kHz 0b001  // 125 kHz
#define INTERNAL_CLOCK_31kHz 0b000   // 31 kHz (來自 INTOSC/256 或直接 INTRC)
/**
 * 根據 _XTAL_FREQ 自動選擇對應的時脈設定
 * _XTAL_FREQ: 系統主頻率 (Hz)，需在專案設定中定義
 *
 * 32MHz 和 16MHz 需要啟用 PLL (Phase-Locked Loop，鎖相迴路)
 * PLL 會將 8MHz/4MHz 內部振盪器倍頻至目標頻率
 */
#if (_XTAL_FREQ == 32000000)
// 32 MHz: 使用 8MHz 內部振盪器 + 4x PLL
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_8MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_32TOSC
#define PLL_ENABLE
#elif (_XTAL_FREQ == 16000000)
// 16 MHz: 使用 4MHz 內部振盪器 + 4x PLL
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_4MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_16TOSC
#define PLL_ENABLE
#elif (_XTAL_FREQ == 8000000)
// 8 MHz: 直接使用 8MHz 內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_8MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_8TOSC
#elif (_XTAL_FREQ == 4000000)
// 4 MHz: 直接使用 4MHz 內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_4MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_4TOSC
#elif (_XTAL_FREQ == 2000000)
// 2 MHz: 直接使用 2MHz 內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_2MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 1000000)
// 1 MHz: 直接使用 1MHz 內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_1MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 500000)
// 500 kHz: 直接使用 500kHz 內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_500kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 250000)
// 250 kHz: 直接使用 250kHz 內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_250kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 125000)
// 125 kHz: 直接使用 125kHz 內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_125kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 31000)
// 31 kHz: 使用最低頻率內部振盪器
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_31kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#else
#error 不合法的內部時脈速度設定於 "_XTAL_FREQ"，請修改。
#endif

/* ========== 內部時脈設定巨集 / Internal Clock Setup Macro ========== */
#ifdef PLL_ENABLE
/**
 * setIntrnalClock() - 設定內部振盪器時脈 (含 PLL)
 *
 * OSCCON: 振盪器控制暫存器
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=30
 *
 * 步驟:
 * 1. 設定 IRCF 位元選擇內部振盪器頻率
 * 2. 執行 Nop() 等待穩定
 * 3. 啟用 PLL 將頻率倍頻 4 倍
 */
#define setIntrnalClock()                  \
    OSCCONbits.IRCF = INTERNAL_CLOCK_IRCF; \
    Nop();                                 \
    OSCTUNEbits.PLLEN = 0b1
#else
/**
 * setIntrnalClock() - 設定內部振盪器時脈 (無 PLL)
 *
 * OSCCON: 振盪器控制暫存器
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=30
 *
 * 直接設定 IRCF 位元選擇內部振盪器頻率
 */
#define setIntrnalClock() OSCCONbits.IRCF = INTERNAL_CLOCK_IRCF
#endif

/* ========== ADC 取樣時間設定 / ADC Acquisition Time ========== */
#pragma region AD_AcquisitionTime
/**
 * ADC 取樣時間計算
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=230
 *
 * ADC 在開始轉換前需要一段取樣時間讓取樣電容充電
 * 最小取樣時間 = 2.4µs (2400 奈秒)
 */
#define MINIMUM_ACQUISITION_TIME 2400  // 最小取樣時間 (奈秒)

/**
 * 計算 TAD (ADC 時脈週期)
 * TAD = ADC 轉換一個位元所需的時間
 * 公式: TAD = (分頻比) / FOSC * 10^9 (轉換為奈秒)
 */
#if (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_2TOSC)
#define _AD_CONVETER_TAD 2 * 1000000000 / _XTAL_FREQ  // TAD = 2/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_4TOSC)
#define _AD_CONVETER_TAD 4 * 1000000000 / _XTAL_FREQ  // TAD = 4/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_8TOSC)
#define _AD_CONVETER_TAD 8 * 1000000000 / _XTAL_FREQ  // TAD = 8/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_16TOSC)
#define _AD_CONVETER_TAD 12 * 1000000000 / _XTAL_FREQ  // TAD = 12/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_32TOSC)
#define _AD_CONVETER_TAD 32 * 1000000000 / _XTAL_FREQ  // TAD = 32/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_64TOSC)
#define _AD_CONVETER_TAD 64 * 1000000000 / _XTAL_FREQ  // TAD = 64/FOSC
#else
#error 不支援 RC 時脈來源的 TAD 計算
#endif

/**
 * ACQT<2:0>: A/D 取樣時間選擇位元
 * 用於 ADCON2 暫存器，設定轉換前的取樣週期數
 */
#define AD_ACQUISITION_TIME_0TAD 0b000   // 0 個 TAD (手動控制)
#define AD_ACQUISITION_TIME_2TAD 0b001   // 2 個 TAD
#define AD_ACQUISITION_TIME_4TAD 0b010   // 4 個 TAD
#define AD_ACQUISITION_TIME_6TAD 0b011   // 6 個 TAD
#define AD_ACQUISITION_TIME_8TAD 0b100   // 8 個 TAD
#define AD_ACQUISITION_TIME_12TAD 0b101  // 12 個 TAD
#define AD_ACQUISITION_TIME_16TAD 0b110  // 16 個 TAD
#define AD_ACQUISITION_TIME_20TAD 0b111  // 20 個 TAD

/**
 * 自動選擇最小符合需求的取樣時間
 * 確保取樣時間 >= MINIMUM_ACQUISITION_TIME (2.4µs)
 */
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

/* ========== 計時器0 / Timer0 ========== */
#pragma region Timer0
/**
 * Timer0 是 8/16 位元可選的計時器/計數器
 * 特點:
 * - 可選擇 8 位元或 16 位元模式
 * - 可程式化預分頻器 (1:2 到 1:256)
 * - 可選擇內部或外部時脈來源
 * - 溢位時可產生中斷
 *
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=125
 */

/* Timer0 預分頻器選擇 (T0PS<2:0>) */
#define TIMER0_PRESCALE_2 0b000    // 1:2 分頻
#define TIMER0_PRESCALE_4 0b001    // 1:4 分頻
#define TIMER0_PRESCALE_8 0b010    // 1:8 分頻
#define TIMER0_PRESCALE_16 0b011   // 1:16 分頻
#define TIMER0_PRESCALE_32 0b100   // 1:32 分頻
#define TIMER0_PRESCALE_64 0b101   // 1:64 分頻
#define TIMER0_PRESCALE_128 0b110  // 1:128 分頻
#define TIMER0_PRESCALE_256 0b111  // 1:256 分頻

/* Timer0 預分頻器啟用/停用 (PSA 位元) */
#define TIMER0_PRESCALE_ENABLE 0b0   // 啟用預分頻器
#define TIMER0_PRESCALE_DISABLE 0b1  // 停用預分頻器 (1:1)

/* Timer0 時脈來源選擇 (T0CS 位元) */
#define TIMER0_CLOCK_SOURCE_T0CKI_PIN 0b1  // 使用 T0CKI 腳位外部時脈
#define TIMER0_CLOCK_SOURCE_INTERNAL 0b0   // 使用內部指令週期 (FOSC/4)

/* Timer0 模式選擇 (T08BIT 位元) */
#define TIMER0_MODE_8BIT 0b1   // 8 位元模式 (TMR0L)
#define TIMER0_MODE_16BIT 0b0  // 16 位元模式 (TMR0H:TMR0L)

/**
 * enableTimer0() - 啟用並設定 Timer0
 * @param prescale     預分頻比 (TIMER0_PRESCALE_x)
 * @param prescaleEnable 預分頻器啟用 (TIMER0_PRESCALE_ENABLE/DISABLE)
 * @param clockSource  時脈來源 (TIMER0_CLOCK_SOURCE_x)
 * @param mode         位元模式 (TIMER0_MODE_8BIT/16BIT)
 *
 * T0CON 暫存器設定
 */
#define enableTimer0(prescale, prescaleEnable, clockSource, mode) \
    T0CONbits.TMR0ON = 0b1;                                       \
    T0CONbits.T08BIT = mode;                                      \
    T0CONbits.T0CS = clockSource;                                 \
    T0CONbits.PSA = prescaleEnable;                               \
    T0CONbits.T0PS = prescale;

#define disableTimer0() T0CONbits.TMR0ON = 0b0                   // 停用 Timer0
#define clearInterrupt_Timer0Overflow() INTCONbits.TMR0IF = 0b0  // 清除溢位中斷旗標

/**
 * enableInterrupt_Timer0Overflow() - 啟用 Timer0 溢位中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 */
#define enableInterrupt_Timer0Overflow(priority)            \
    INTCONbits.TMR0IE = 0b1;       /* 啟用 TMR0 溢位中斷 */ \
    INTCON2bits.TMR0IP = priority; /* 設定中斷優先權 */     \
    clearInterrupt_Timer0Overflow()

#define interruptByTimer0Overflow() INTCONbits.TMR0IF  // 檢查是否為 Timer0 溢位中斷

/**
 * setTimer0InterruptPeriod8() - 設定 8 位元模式的中斷週期
 * @param period    週期時間 (微秒 µs)
 * @param prescale  預分頻比數值 (2, 4, 8, 16, 32, 64, 128, 256)
 *
 * 計算公式: TMR0 = 255 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer0InterruptPeriod8(period, prescale)                                                                    \
    _Static_assert((uint32_t)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1) <= 255, "Period time too long"); \
    TMR0 = (byte)(255 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

/**
 * setTimer0InterruptPeriod16() - 設定 16 位元模式的中斷週期
 * @param period    週期時間 (微秒 µs)
 * @param prescale  預分頻比數值 (2, 4, 8, 16, 32, 64, 128, 256)
 *
 * 計算公式: TMR0 = 65535 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer0InterruptPeriod16(period, prescale)                                                                     \
    _Static_assert((uint32_t)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1) <= 65535, "Period time too long"); \
    TMR0 = (uint16_t)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

#pragma endregion Timer0

/* ========== 計時器1 / Timer1 ========== */
#pragma region Timer1
/**
 * Timer1 是 16 位元計時器/計數器
 * 特點:
 * - 16 位元計時器/計數器 (TMR1H:TMR1L)
 * - 可程式化預分頻器 (1:1, 1:2, 1:4, 1:8)
 * - 可選擇內部或外部時脈來源
 * - 支援 CCP 模組的捕獲/比較功能
 * - 溢位時可產生中斷
 *
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=129
 */

/* Timer1 預分頻器選擇 (T1CKPS<1:0>) */
#define TIMER1_PRESCALE_1 0b00  // 1:1 分頻 (無分頻)
#define TIMER1_PRESCALE_2 0b01  // 1:2 分頻
#define TIMER1_PRESCALE_4 0b10  // 1:4 分頻
#define TIMER1_PRESCALE_8 0b11  // 1:8 分頻

/**
 * enableTimer1() - 啟用 Timer1
 * @param prescale  預分頻比 (TIMER1_PRESCALE_x)
 *
 * 設定為 16 位元讀寫模式並啟用計時器
 */
#define enableTimer1(prescale)                         \
    T1CONbits.RD16 = 1;          /* 16 位元讀寫模式 */ \
    T1CONbits.T1CKPS = prescale; /* 設定預分頻比 */    \
    T1CONbits.TMR1ON = 0b1       /* 啟用 Timer1 */

/**
 * configTimer1() - 設定 Timer1 但不啟用
 * @param prescale  預分頻比 (TIMER1_PRESCALE_x)
 */
#define configTimer1(prescale) \
    T1CONbits.RD16 = 1;        \
    T1CONbits.T1CKPS = prescale

#define enableTimer1bit() T1CONbits.TMR1ON = 0b1               // 啟用 Timer1
#define disableTimer1() T1CONbits.TMR1ON = 0b0                 // 停用 Timer1
#define clearInterrupt_Timer1Overflow() PIR1bits.TMR1IF = 0b0  // 清除溢位中斷旗標

/**
 * enableInterrupt_Timer1Overflow() - 啟用 Timer1 溢位中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 */
#define enableInterrupt_Timer1Overflow(priority)         \
    PIE1bits.TMR1IE = 0b1;      /* 啟用 TMR1 溢位中斷 */ \
    IPR1bits.TMR1IP = priority; /* 設定中斷優先權 */     \
    clearInterrupt_Timer1Overflow()

#define interruptByTimer1Overflow() PIR1bits.TMR1IF  // 檢查是否為 Timer1 溢位中斷

/**
 * setTimer1InterruptPeriod() - 設定 Timer1 中斷週期
 * @param period    週期時間 (微秒 µs)
 * @param prescale  預分頻比數值 (1, 2, 4, 8)
 *
 * 計算公式: TMR1 = 65535 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer1InterruptPeriod(period, prescale)                                                                       \
    _Static_assert((uint32_t)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1) <= 65535, "Period time too long"); \
    TMR1 = (uint16_t)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)
#pragma endregion Timer1

/* ========== 計時器2 / Timer2 ========== */
#pragma region Timer2
/**
 * Timer2 是 8 位元計時器，具有預分頻器和後分頻器
 * 特點:
 * - 8 位元計時器 (TMR2)
 * - 8 位元週期暫存器 (PR2)
 * - 可程式化預分頻器 (1:1, 1:4, 1:16)
 * - 可程式化後分頻器 (1:1 到 1:16)
 * - TMR2 與 PR2 匹配時產生中斷
 * - 用於 PWM 模式的時基
 *
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=135
 */

/* Timer2 預分頻器選擇 (T2CKPS<1:0>) */
#define TIMER2_PRESCALE_1 0b00   // 1:1 分頻 (無分頻)
#define TIMER2_PRESCALE_4 0b01   // 1:4 分頻
#define TIMER2_PRESCALE_16 0b10  // 1:16 分頻

/**
 * enableTimer2() - 啟用 Timer2
 * @param prescale      預分頻比 (TIMER2_PRESCALE_x)
 * @param poscaleBits   後分頻比 (0-15，對應 1:1 到 1:16)
 *
 * 後分頻器用於減少中斷頻率: 實際中斷頻率 = 匹配頻率 / (postscale + 1)
 */
#define enableTimer2(prescale, poscaleBits)             \
    T2CONbits.T2CKPS = prescale;     /* 設定預分頻比 */ \
    T2CONbits.T2OUTPS = poscaleBits; /* 設定後分頻比 */ \
    T2CONbits.TMR2ON = 0b1           /* 啟用 Timer2 */

/**
 * configTimer2() - 設定 Timer2 但不啟用
 * @param prescale      預分頻比 (TIMER2_PRESCALE_x)
 * @param poscaleBits   後分頻比 (0-15)
 */
#define configTimer2(prescale, poscaleBits) \
    T2CONbits.T2CKPS = prescale;            \
    T2CONbits.T2OUTPS = poscaleBits

#define enableTimer2bit() T2CONbits.TMR2ON = 0b1          // 啟用 Timer2
#define disableTimer2() T2CONbits.TMR2ON = 0b0            // 停用 Timer2
#define clearInterrupt_Timer2PR2() PIR1bits.TMR2IF = 0b0  // 清除 TMR2/PR2 匹配中斷旗標

/**
 * enableInterrupt_Timer2PR2() - 啟用 Timer2 與 PR2 匹配中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 *
 * 當 TMR2 值等於 PR2 時觸發中斷
 */
#define enableInterrupt_Timer2PR2(priority)                  \
    PIE1bits.TMR2IE = 0b1;      /* 啟用 TMR2/PR2 匹配中斷 */ \
    IPR1bits.TMR2IP = priority; /* 設定中斷優先權 */         \
    clearInterrupt_Timer2PR2()

#define interruptByTimer2PR2() PIR1bits.TMR2IF  // 檢查是否為 Timer2/PR2 匹配中斷

/**
 * disableInterrupt_Timer2PR2() - 停用 Timer2 與 PR2 匹配中斷
 */
#define disableInterrupt_Timer2PR2() \
    PIE1bits.TMR2IE = 0b0;           \
    clearInterrupt_Timer2PR2()

/**
 * setTimer2InterruptPeriod() - 設定 Timer2 中斷週期
 * @param period    週期時間 (微秒 µs)
 * @param prescale  預分頻比數值 (1, 4, 16)
 * @param postscale 後分頻比數值 (1-16)
 *
 * 計算公式: PR2 = (period × FOSC) / (4 × prescale × postscale × 10^6) - 1
 * 此值設定 TMR2 從 0 計數到 PR2 的週期
 */
#define setTimer2InterruptPeriod(period, prescale, postscale)                                                                      \
    _Static_assert((uint32_t)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale / postscale - 1) <= 255, "Period time too long"); \
    PR2 = (byte)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale / postscale - 1)

#pragma endregion Timer2

/* ========== 計時器3 / Timer3 ========== */
#pragma region Timer3
/**
 * Timer3 是 16 位元計時器/計數器
 * 特點:
 * - 16 位元計時器/計數器 (TMR3H:TMR3L)
 * - 可程式化預分頻器 (1:1, 1:2, 1:4, 1:8)
 * - 可選擇內部或外部時脈來源
 * - 可作為 CCP 模組的捕獲/比較時基
 * - 溢位時可產生中斷
 *
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=137
 */

/* Timer3 預分頻器選擇 (T3CKPS<1:0>) */
#define TIMER3_PRESCALE_1 0b00  // 1:1 分頻 (無分頻)
#define TIMER3_PRESCALE_2 0b01  // 1:2 分頻
#define TIMER3_PRESCALE_4 0b10  // 1:4 分頻
#define TIMER3_PRESCALE_8 0b11  // 1:8 分頻

/**
 * enableTimer3() - 啟用 Timer3
 * @param prescale  預分頻比 (TIMER3_PRESCALE_x)
 */
#define enableTimer3(prescale)                    \
    T3CONbits.TMR3ON = 0b1;     /* 啟用 Timer3 */ \
    T3CONbits.T3CKPS = prescale /* 設定預分頻比 */

#define disableTimer3() T3CONbits.TMR3ON = 0b0                 // 停用 Timer3
#define clearInterrupt_Timer3Overflow() PIR2bits.TMR3IF = 0b0  // 清除溢位中斷旗標

/**
 * enableInterrupt_Timer3Overflow() - 啟用 Timer3 溢位中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 */
#define enableInterrupt_Timer3Overflow(priority)         \
    PIE2bits.TMR3IE = 0b1;      /* 啟用 TMR3 溢位中斷 */ \
    IPR2bits.TMR3IP = priority; /* 設定中斷優先權 */     \
    clearInterrupt_Timer3Overflow()

#define interruptByTimer3Overflow() PIR2bits.TMR3IF  // 檢查是否為 Timer3 溢位中斷

/**
 * setTimer3InterruptPeriod() - 設定 Timer3 中斷週期
 * @param period    週期時間 (微秒 µs)
 * @param prescale  預分頻比數值 (1, 2, 4, 8)
 *
 * 計算公式: TMR3 = 65535 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer3InterruptPeriod(period, prescale)                                                                       \
    _Static_assert((uint32_t)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1) <= 65535, "Period time too long"); \
    TMR3 = (uint16_t)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

#pragma endregion Timer3

/* ========== PWM / 捕獲 / 比較模組控制 / CCP Module Control ========== */
#pragma region PWM_Control
/**
 * 增強型捕獲/比較/PWM (ECCP) 模組
 *
 * CCP 模組提供三種主要功能:
 * 1. 捕獲模式 (Capture): 捕獲外部事件發生時的計時器值
 * 2. 比較模式 (Compare): 當計時器值與設定值匹配時觸發事件
 * 3. PWM 模式: 產生脈寬調變輸出信號
 *
 * PIC18F4520 有兩個 CCP 模組: CCP1 (增強型) 和 CCP2
 */

/* CCP 模組模式選擇 (CCP1M<3:0> / CCP2M<3:0>) */
#define ECCP_MODE_OFF 0b0000          // 關閉 - 重設 CCP 模組
#define ECCP_MODE_RESERVED 0b0001     // 保留
#define ECCP_MODE_COMPARE_TOM 0b0010  // 比較模式 - 匹配時切換輸出
#define ECCP_MODE_CAPTURE 0b0011      // 捕獲模式 - 每個邊緣
#define ECCP_MODE_CAPTURE_EFE 0b0100  // 捕獲模式 - 每個下降緣
#define ECCP_MODE_CAPTURE_ERE 0b0101  // 捕獲模式 - 每個上升緣
#define ECCP_MODE_CAPTURE_R04 0b0110  // 捕獲模式 - 每第 4 個上升緣
#define ECCP_MODE_CAPTURE_R16 0b0111  // 捕獲模式 - 每第 16 個上升緣
#define ECCP_MODE_COMPARE_SOM 0b1000  // 比較模式 - 初始低電位，匹配時設為高電位
#define ECCP_MODE_COMPARE_COM 0b1001  // 比較模式 - 初始高電位，匹配時設為低電位
#define ECCP_MODE_COMPARE_RIO 0b1010  // 比較模式 - 僅產生軟體中斷
#define ECCP_MODE_COMPARE_TSE 0b1011  // 比較模式 - 觸發特殊事件 (重設 TMR1/TMR3)
#define ECCP_MODE_PWM_HH 0b1100       // PWM 模式 - P1A,P1C 高電位有效; P1B,P1D 高電位有效
#define ECCP_MODE_PWM_HL 0b1101       // PWM 模式 - P1A,P1C 高電位有效; P1B,P1D 低電位有效
#define ECCP_MODE_PWM_LH 0b1110       // PWM 模式 - P1A,P1C 低電位有效; P1B,P1D 高電位有效
#define ECCP_MODE_PWM_LL 0b1111       // PWM 模式 - P1A,P1C 低電位有效; P1B,P1D 低電位有效

/**
 * setCCP1Mode() / setCCP2Mode() - 設定 CCP 模組模式
 * @param eccpMode  模式選擇 (ECCP_MODE_x)
 *
 * CCP1CON / CCP2CON 暫存器設定
 */
#define setCCP1Mode(eccpMode) CCP1CONbits.CCP1M = eccpMode  // 設定 CCP1 模式
#define setCCP2Mode(eccpMode) CCP2CONbits.CCP2M = eccpMode  // 設定 CCP2 模式

/**
 * setCCP1PwmDutyCycle() - 設定 CCP1 PWM 佔空比
 * @param length    高電位時間長度 (微秒 µs)
 * @param prescale  Timer2 預分頻比數值 (1, 4, 16)
 *
 * PWM 佔空比 = (CCPR1L:DC1B) × TOSC × Timer2 預分頻比
 * 10 位元解析度: CCPR1L (高 8 位) + DC1B (低 2 位)
 */
#define setCCP1PwmDutyCycle(length, prescale)                                                \
    do {                                                                                     \
        unsigned int value = (unsigned int)((length) / (1000000.0 / _XTAL_FREQ) / prescale); \
        CCP1CONbits.DC1B = (byte)value & 0b11; /* 低 2 位 */                                 \
        CCPR1L = (byte)(value >> 2);           /* 高 8 位 */                                 \
    } while (0)

/**
 * setCCP2PwmDutyCycle() - 設定 CCP2 PWM 佔空比
 * @param length    高電位時間長度 (微秒 µs)
 * @param prescale  Timer2 預分頻比數值 (1, 4, 16)
 *
 * PWM 佔空比 = (CCPR2L:DC2B) × TOSC × Timer2 預分頻比
 * 10 位元解析度: CCPR2L (高 8 位) + DC2B (低 2 位)
 */
#define setCCP2PwmDutyCycle(length, prescale)                                                \
    do {                                                                                     \
        unsigned int value = (unsigned int)((length) / (1000000.0 / _XTAL_FREQ) / prescale); \
        CCP2CONbits.DC2B = (byte)value & 0b11; /* 低 2 位 */                                 \
        CCPR2L = (byte)(value >> 2);           /* 高 8 位 */                                 \
    } while (0)

#pragma endregion PWM_Control

/* ========== PIC18F4520 腳位定義 / Pin Definitions ========== */
#pragma region PIC18F4520_Pins
/**
 * PIC18F4520 40-pin DIP 封裝腳位定義
 *
 * 腳位巨集格式: PORT, PINx
 * 用於 pinMode(), digitalWrite() 等巨集
 *
 * 每個腳位可能有多種功能:
 * - 數位 I/O
 * - 類比輸入 (ANx)
 * - 特殊功能 (UART, SPI, I2C, CCP 等)
 */

/* PORTA 腳位 */
#define PIN_MCLR       // 腳位 1:  MCLR / VPP / RE3 (主清除/程式電壓)
#define PIN_RA0 A, A0  // 腳位 2:  RA0 / AN0 (類比通道 0)
#define PIN_RA1 A, A1  // 腳位 3:  RA1 / AN1 (類比通道 1)
#define PIN_RA2 A, A2  // 腳位 4:  RA2 / AN2 / VREF- / CVREF (負參考電壓)
#define PIN_RA3 A, A3  // 腳位 5:  RA3 / AN3 / VREF+ (正參考電壓)
#define PIN_RA4 A, A4  // 腳位 6:  RA4 / T0CKI / C1OUT (Timer0 外部時脈)
#define PIN_RA5 A, A5  // 腳位 7:  RA5 / AN4 / SS / HLVDIN / C2OUT (SPI 從機選擇)

/* PORTE 腳位 */
#define PIN_RE0 E, E0  // 腳位 8:  RE0 / RD / AN5 (平行從機埠讀取)
#define PIN_RE1 E, E1  // 腳位 9:  RE1 / WR / AN6 (平行從機埠寫入)
#define PIN_RE2 E, E2  // 腳位 10: RE2 / CS / AN7 (平行從機埠片選)

/* 振盪器腳位 */
#define PIN_OSC1  // 腳位 13: OSC1 / CLKI / RA7 (振盪器輸入)
#define PIN_OSC2  // 腳位 14: OSC2 / CLKO / RA6 (振盪器輸出)

/* PORTC 腳位 */
#define PIN_RC0 C, C0  // 腳位 15: RC0 / T1OSO / T13CKI (Timer1 振盪器輸出)
#define PIN_RC1 C, C1  // 腳位 16: RC1 / T1OSI / CCP2 (Timer1 振盪器輸入/CCP2)
#define PIN_RC2 C, C2  // 腳位 17: RC2 / CCP1 / P1A (CCP1 PWM 輸出)
#define PIN_RC3 C, C3  // 腳位 18: RC3 / SCK / SCL (SPI 時脈/I2C 時脈)
#define PIN_RC4 C, C4  // 腳位 23: RC4 / SDI / SDA (SPI 資料輸入/I2C 資料)
#define PIN_RC5 C, C5  // 腳位 24: RC5 / SDO (SPI 資料輸出)
#define PIN_RC6 C, C6  // 腳位 25: RC6 / TX / CK (UART 傳送)
#define PIN_RC7 C, C7  // 腳位 26: RC7 / RX / DT (UART 接收)

/* PORTD 腳位 */
#define PIN_RD0 D, D0  // 腳位 19: RD0 / PSP0 (平行從機埠資料 0)
#define PIN_RD1 D, D1  // 腳位 20: RD1 / PSP1 (平行從機埠資料 1)
#define PIN_RD2 D, D2  // 腳位 21: RD2 / PSP2 (平行從機埠資料 2)
#define PIN_RD3 D, D3  // 腳位 22: RD3 / PSP3 (平行從機埠資料 3)
#define PIN_RD4 D, D4  // 腳位 27: RD4 / PSP4 (平行從機埠資料 4)
#define PIN_RD5 D, D5  // 腳位 28: RD5 / PSP5 / P1B (ECCP P1B 輸出)
#define PIN_RD6 D, D6  // 腳位 29: RD6 / PSP6 / P1C (ECCP P1C 輸出)
#define PIN_RD7 D, D7  // 腳位 30: RD7 / PSP7 / P1D (ECCP P1D 輸出)

/* PORTB 腳位 */
#define PIN_RB0 B, B0  // 腳位 33: RB0 / INT0 / FLT0 / AN12 (外部中斷 0)
#define PIN_RB1 B, B1  // 腳位 34: RB1 / INT1 / AN10 (外部中斷 1)
#define PIN_RB2 B, B2  // 腳位 35: RB2 / INT2 / AN8 (外部中斷 2)
#define PIN_RB3 B, B3  // 腳位 36: RB3 / AN9 / CCP2 (備用 CCP2)
#define PIN_RB4 B, B4  // 腳位 37: RB4 / KBI0 / AN11 (鍵盤中斷 0)
#define PIN_RB5 B, B5  // 腳位 38: RB5 / KBI1 / PGM (低電壓程式設計)
#define PIN_RB6 B, B6  // 腳位 39: RB6 / KBI2 / PGC (程式時脈)
#define PIN_RB7 B, B7  // 腳位 40: RB7 / KBI3 / PGD (程式資料)
#pragma endregion PIC18F4520_Pins

/* ========== 腳位控制 / Pin Control ========== */
#pragma region PinControl
/**
 * 數位 I/O 腳位控制巨集
 *
 * 使用方式:
 * - pinMode(PIN_RA0, PIN_OUTPUT);  // 設定 RA0 為輸出
 * - digitalWrite(PIN_RA0, 1);      // 設定 RA0 輸出高電位
 * - byte state = pinState(PIN_RA0); // 讀取 RA0 輸出狀態
 */

/* 腳位方向設定 (TRISx 暫存器) */
#define PIN_INPUT 0b1   // 設定為輸入 (高阻抗)
#define PIN_OUTPUT 0b0  // 設定為輸出

/**
 * pinMode() - 設定腳位輸入/輸出方向
 * @param pin   腳位 (使用 PIN_Rxx 巨集)
 * @param mode  方向 (PIN_INPUT 或 PIN_OUTPUT)
 *
 * 操作 TRISx 暫存器的對應位元
 */
#define pinMode(pin, mode) _pinGetPortBits(TRIS, pin)._pinGetPinBit(R, pin) = mode

/**
 * digitalWrite() - 設定腳位輸出電位
 * @param pin    腳位 (使用 PIN_Rxx 巨集)
 * @param value  輸出值 (0=低電位, 1=高電位)
 *
 * 操作 LATx 暫存器 (鎖存器) 的對應位元
 * 使用 LAT 而非 PORT 可避免讀-修改-寫問題
 */
#define digitalWrite(pin, value) _pinGetPortBits(LAT, pin)._pinGetPinBit(L, pin) = value

/**
 * pinState() - 讀取腳位輸出鎖存器狀態
 * @param pin  腳位 (使用 PIN_Rxx 巨集)
 * @return     目前輸出鎖存值 (0 或 1)
 */
#define pinState(pin) _pinGetPortBits(LAT, pin)._pinGetPinBit(L, pin)

/* PORTB 內部上拉電阻控制 */
#define PORTB_PULLUP_ENABLE 0b0   // 啟用 PORTB 內部上拉電阻
#define PORTB_PULLUP_DISABLE 0b1  // 停用 PORTB 內部上拉電阻

/**
 * setPortBPullup() - 設定 PORTB 內部上拉電阻
 * @param state  狀態 (PORTB_PULLUP_ENABLE/DISABLE)
 *
 * PORTB 的 RB<7:4> 腳位具有可程式化的內部弱上拉電阻
 * 當設定為輸入時，可用於按鈕等應用，省去外部上拉電阻
 */
#define setPortBPullup(state) INTCON2bits.RBPU = state
#pragma endregion PinControl

/* ========== 類比數位轉換器控制 / ADC Control ========== */
#pragma region AD_Control
/**
 * 10 位元類比數位轉換器 (ADC)
 *
 * PIC18F4520 具有 13 個類比輸入通道 (AN0-AN12)
 * ADC 將類比電壓轉換為 10 位元數位值 (0-1023)
 *
 * 轉換公式: 數位值 = (輸入電壓 / 參考電壓) × 1023
 *
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=228
 */

/**
 * setANPinADConfig() - 設定類比/數位腳位配置
 * @param value  4 位元值，每個位元對應一組腳位
 *               0 = 類比輸入, 1 = 數位 I/O
 *
 * ADCON1 暫存器的 PCFG<3:0> 位元
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=226
 */
#define setANPinADConfig(value) ADCON1bits.PCFG = value

/**
 * setANPinVoltageReferenceConfig() - 設定 ADC 參考電壓來源
 * @param conf0  VREF- 來源: 1 = AN2 腳位, 0 = VSS (接地)
 * @param conf1  VREF+ 來源: 1 = AN3 腳位, 0 = VDD (電源電壓)
 *
 * 使用外部參考電壓可提高轉換精度
 * 預設使用 VDD 和 VSS 作為參考電壓
 */
#define setANPinVoltageReferenceConfig(conf0, conf1) \
    ADCON1bits.VCFG1 = conf0;                        \
    ADCON1bits.VCFG0 = conf1

/**
 * setANPinAnalogChannelSelect() - 選擇要轉換的類比通道
 * @param value  通道編號 (0-12，對應 AN0-AN12)
 *
 * 設定 ADCON0 的 CHS<3:0> 位元
 * 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=225
 */
#define setANPinAnalogChannelSelect(value) ADCON0bits.CHS = value

/**
 * startADConverter() - 開始 ADC 轉換
 *
 * 設定 GO/DONE 位元啟動轉換
 * 轉換完成後此位元會自動清除
 */
#define startADConverter() ADCON0bits.GO = 1

/**
 * getADConverter() - 讀取 ADC 轉換結果
 * @return  10 位元轉換結果 (0-1023)
 *
 * 讀取 ADRESH:ADRESL 暫存器
 */
#define getADConverter() ADRES

/**
 * enableADConverter() - 啟用並初始化 ADC 模組
 *
 * 執行以下設定:
 * - 啟用 ADC 模組 (ADON = 1)
 * - 設定結果右對齊 (ADFM = 1): 高 6 位元在 ADRESH，低 8 位元在 ADRESL
 * - 設定 ADC 時脈來源 (根據系統頻率自動選擇)
 * - 設定取樣時間 (根據時脈自動計算)
 */
#define enableADConverter()                                       \
    ADCON0bits.ADON = 0b1;                /* 啟用 ADC */          \
    ADCON2bits.ADFM = 0b1;                /* 結果右對齊 */        \
    ADCON2bits.ADCS = AD_CLOCK_SOURCE;    /* 設定 ADC 時脈來源 */ \
    ADCON2bits.ACQT = AD_ACQUISITION_TIME /* 設定取樣時間 */

#define clearInterrupt_ADConverter() PIR1bits.ADIF = 0  // 清除 ADC 中斷旗標

/**
 * enableInterrupt_ADConverter() - 啟用 ADC 轉換完成中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 *
 * 當 ADC 轉換完成時觸發中斷
 */
#define enableInterrupt_ADConverter(priority)    \
    clearInterrupt_ADConverter();                \
    PIE1bits.ADIE = 1;       /* 啟用 ADC 中斷 */ \
    IPR1bits.ADIP = priority /* 設定中斷優先權 */

#define interruptByADConverter() PIR1bits.ADIF  // 檢查是否為 ADC 轉換完成中斷

#pragma endregion AD_Control

/* ========== 中斷控制 / Interrupt Control ========== */
#pragma region InterruptControl
/**
 * PIC18F4520 中斷系統
 *
 * 支援兩級優先權中斷:
 * - 高優先權: 向量位址 0x0008
 * - 低優先權: 向量位址 0x0018
 *
 * 中斷來源包括:
 * - 外部中斷 (INT0, INT1, INT2)
 * - 計時器溢位
 * - ADC 轉換完成
 * - UART 收發
 * - PORTB 狀態改變
 * - 等等...
 */

/**
 * enableInterruptPriorityMode() - 啟用中斷優先權模式
 * @param state  1 = 啟用兩級優先權, 0 = 停用 (相容模式)
 *
 * RCON 暫存器的 IPEN 位元
 * 啟用後可區分高/低優先權中斷
 */
#define enableInterruptPriorityMode(state) RCONbits.IPEN = state

/**
 * enableGlobalInterrupt() - 全域中斷致能
 * @param state  1 = 啟用, 0 = 停用
 *
 * 當 IPEN = 0 (相容模式):
 * - 1 = 啟用所有未遮罩的中斷
 * - 0 = 停用所有中斷
 *
 * 當 IPEN = 1 (優先權模式):
 * - 1 = 啟用所有高優先權中斷
 * - 0 = 停用所有中斷
 */
#define enableGlobalInterrupt(state) INTCONbits.GIE = state

/**
 * enablePeripheralInterrupt() - 周邊中斷致能
 * @param state  1 = 啟用, 0 = 停用
 *
 * 當 IPEN = 0 (相容模式):
 * - 1 = 啟用所有未遮罩的周邊中斷
 * - 0 = 停用所有周邊中斷
 *
 * 當 IPEN = 1 (優先權模式):
 * - 1 = 啟用所有低優先權周邊中斷
 * - 0 = 停用所有低優先權周邊中斷
 */
#define enablePeripheralInterrupt(state) INTCONbits.PEIE = state

/* ---------- 外部中斷 INT0 (RB0 腳位) ---------- */
#define clearInterrupt_RB0External() INTCONbits.INT0IF = 0b0  // 清除 INT0 中斷旗標

/**
 * enableInterrupt_RB0External() - 啟用 RB0/INT0 外部中斷
 *
 * INT0 固定為高優先權，無法設定優先權
 * 中斷發生時必須在軟體中清除 INT0IF 旗標
 */
#define enableInterrupt_RB0External() \
    clearInterrupt_RB0External();     \
    INTCONbits.INT0IE = 0b1

#define interruptByRB0External() INTCONbits.INT0IF  // 檢查是否為 INT0 中斷

/* ---------- 外部中斷 INT1 (RB1 腳位) ---------- */
#define clearInterrupt_RB1External() INTCON3bits.INT1IF = 0b0  // 清除 INT1 中斷旗標

/**
 * enableInterrupt_RB1External() - 啟用 RB1/INT1 外部中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 *
 * 中斷發生時必須在軟體中清除 INT1IF 旗標
 */
#define enableInterrupt_RB1External(priority)          \
    clearInterrupt_RB1External();                      \
    INTCON3bits.INT1IE = 0b1;     /* 啟用 INT1 中斷 */ \
    INTCON3bits.INT1IP = priority /* 設定優先權 */

#define interruptByRB1External() INTCON3bits.INT1IF  // 檢查是否為 INT1 中斷

/* ---------- 外部中斷 INT2 (RB2 腳位) ---------- */
#define clearInterrupt_RB2External() INTCON3bits.INT2IF = 0b0  // 清除 INT2 中斷旗標

/**
 * enableInterrupt_RB2External() - 啟用 RB2/INT2 外部中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 *
 * 中斷發生時必須在軟體中清除 INT2IF 旗標
 */
#define enableInterrupt_RB2External(priority)          \
    clearInterrupt_RB2External();                      \
    INTCON3bits.INT2IE = 0b1;     /* 啟用 INT2 中斷 */ \
    INTCON3bits.INT2IP = priority /* 設定優先權 */

#define interruptByRB2External() INTCON3bits.INT2IF  // 檢查是否為 INT2 中斷

/* ---------- PORTB 狀態改變中斷 (RB7:RB4) ---------- */
#define clearInterrupt_RBPortChange() INTCONbits.RBIF = 0b0  // 清除 PORTB 改變中斷旗標

/**
 * enableInterrupt_RBPortChange() - 啟用 PORTB<7:4> 狀態改變中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 *
 * 當 RB<7:4> 任一腳位狀態改變時觸發
 * 常用於鍵盤掃描、按鈕偵測等應用
 * 必須在軟體中清除 RBIF 旗標
 */
#define enableInterrupt_RBPortChange(priority)            \
    clearInterrupt_RBPortChange();                        \
    INTCONbits.RBIE = 0b1;      /* 啟用 PORTB 改變中斷 */ \
    INTCON2bits.RBIP = priority /* 設定優先權 */

#define interruptByRBPortChange() INTCONbits.RBIF  // 檢查是否為 PORTB 改變中斷

/* ---------- UART 傳送中斷 ---------- */
#define clearInterrupt_TransmitUART() PIR1bits.TXIF = 0b0  // 清除 UART 傳送中斷旗標

/**
 * enableInterrupt_TransmitUART() - 啟用 UART 傳送完成中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 *
 * 當 TXREG 為空（可寫入下一個位元組）時觸發
 */
#define enableInterrupt_TransmitUART(priority)        \
    clearInterrupt_TransmitUART();                    \
    PIE1bits.TXIE = 0b1;     /* 啟用 UART 傳送中斷 */ \
    IPR1bits.TXIP = priority /* 設定優先權 */

#define interruptByTransmitUART() PIR1bits.TXIF  // 檢查是否為 UART 傳送中斷

/* ---------- UART 接收中斷 ---------- */
#define clearInterrupt_ReceiveUART() PIR1bits.RCIF = 0b0  // 清除 UART 接收中斷旗標

/**
 * enableInterrupt_ReceiveUART() - 啟用 UART 接收完成中斷
 * @param priority  中斷優先權 (1=高, 0=低)
 *
 * 當 RCREG 有資料可讀取時觸發
 */
#define enableInterrupt_ReceiveUART(priority)         \
    clearInterrupt_ReceiveUART();                     \
    PIE1bits.RCIE = 0b1;     /* 啟用 UART 接收中斷 */ \
    IPR1bits.RCIP = priority /* 設定優先權 */

#define interruptByReceiveUART() PIR1bits.RCIF  // 檢查是否為 UART 接收中斷

#pragma endregion InterruptControl

/* ========== UART 串列通訊 / Serial Communication ========== */
#pragma region UART
/**
 * EUSART (Enhanced Universal Synchronous Asynchronous Receiver Transmitter)
 * 增強型通用同步/非同步收發器
 *
 * 支援功能:
 * - 全雙工非同步通訊
 * - 可程式化鮑率 (Baud Rate)
 * - 9 位元資料傳輸
 * - 硬體流量控制
 * - 自動鮑率偵測
 *
 * 腳位:
 * - RC6/TX: 傳送
 * - RC7/RX: 接收
 */

/* serialPrintf 緩衝區大小，可在 include 前定義覆蓋 */
#ifndef SEIAL_PRINTF_STATIC_SIZE
#define SEIAL_PRINTF_STATIC_SIZE 16
#endif
char serialPrintfCache[SEIAL_PRINTF_STATIC_SIZE];  // printf 格式化字串緩衝區

/**
 * serialReceiveEnable() - 啟用/停用連續接收模式
 * @param state  1 = 啟用, 0 = 停用
 *
 * CREN 位元控制連續接收
 * 停用再啟用可清除溢位錯誤 (OERR)
 */
#define serialReceiveEnable(state) RCSTAbits.CREN = state

/**
 * serialReceiveOverrunError() - 檢查接收溢位錯誤
 * @return  1 = 發生溢位錯誤, 0 = 無錯誤
 *
 * 當接收緩衝區已滿但又收到新資料時發生
 * 清除方式: 將 CREN 設為 0 再設回 1
 */
#define serialReceiveOverrunError() RCSTAbits.OERR

/**
 * serialReceiveFramingError() - 檢查訊框錯誤
 * @return  1 = 發生訊框錯誤, 0 = 無錯誤
 *
 * 當接收到的停止位元不正確時發生
 * 清除方式: 讀取 RCREG 並接收下一個有效位元組
 */
#define serialReceiveFramingError() RCSTAbits.FERR

/**
 * serialBegin() - 初始化 UART 串列埠
 * @param baudRate                 鮑率 (如 9600, 115200)
 * @param receiveInterruptPriority 接收中斷優先權 (1=高, 0=低)
 *
 * 自動計算鮑率產生器數值
 * 設定為非同步模式、啟用傳送與接收
 */
inline void serialBegin(long baudRate, byte receiveInterruptPriority) {
    pinMode(PIN_RC6, PIN_OUTPUT);  // RC6(TX) 設為輸出
    pinMode(PIN_RC7, PIN_INPUT);   // RC7(RX) 設為輸入

    // 設定鮑率
    TXSTAbits.SYNC = 0;     // 非同步模式
    BAUDCONbits.BRG16 = 1;  // 使用 16 位元鮑率產生器

    long baudRateGenerator = _XTAL_FREQ / baudRate;
    TXSTAbits.BRGH = baudRate > 2400;  // 高鮑率選擇位元

    // 根據設定計算分頻比
    // 參考: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=207
    if (!TXSTAbits.SYNC && !BAUDCONbits.BRG16 && !TXSTAbits.BRGH)
        baudRateGenerator /= 64;
    else if (!TXSTAbits.SYNC && BAUDCONbits.BRG16 != TXSTAbits.BRGH)
        baudRateGenerator /= 16;
    else
        baudRateGenerator /= 4;

    // 設定鮑率產生器暫存器
    SPBRGH = (byte)(baudRateGenerator >> 8);  // 高位元組
    SPBRG = (byte)baudRateGenerator;          // 低位元組

    // 啟用串列埠
    RCSTAbits.SPEN = 1;  // 啟用串列埠 (設定 RX/TX 腳位為串列埠功能)
    TXSTAbits.TXEN = 1;  // 啟用傳送
    RCSTAbits.CREN = 1;  // 啟用連續接收
    enableInterrupt_ReceiveUART(receiveInterruptPriority);

    /* 傳送器架構說明:
     * TSR   : 移位暫存器，正在傳送的資料
     * TXREG : 傳送緩衝器，下一筆要傳送的資料
     * TXSTAbits.TRMT: 當 TSR 為空時設為 1
     */
    /* 接收器架構說明:
     * RSR   : 移位暫存器，正在接收的資料
     * RCREG : 接收緩衝器，已接收完成的資料 (讀取此暫存器取得資料)
     */
}

/**
 * serialAvailableForWrite() - 檢查是否可傳送
 * @return  1 = 可寫入, 0 = 忙碌中
 *
 * 檢查傳送移位暫存器 (TSR) 是否為空
 */
#define serialAvailableForWrite() TXSTAbits.TRMT

/**
 * serialWrite() - 傳送單一字元
 * @param c  要傳送的字元
 *
 * 阻塞式傳送，等待直到可寫入
 */
void serialWrite(char c) {
    while (!serialAvailableForWrite());  // 忙碌等待
    TXREG = c;                           // 寫入 TXREG 開始傳送
}

void putch(char c) {
    while (!serialAvailableForWrite());  // 忙碌等待
    TXREG = c;                           // 寫入 TXREG 開始傳送
}

/**
 * serialPrint() - 傳送字串
 * @param text  以 null 結尾的字串
 *
 * 逐字元傳送直到遇到 '\0'
 */
void serialPrint(char* text) {
    for (int i = 0; text[i] != '\0'; i++) {
        while (!serialAvailableForWrite());  // 忙碌等待
        TXREG = text[i];                     // 寫入 TXREG 開始傳送
    }
}

/**
 * serialPrintf() - 格式化輸出
 * @param format  格式字串 (同 printf)
 * @param ...     可變參數
 *
 * 使用 sprintf 格式化後傳送
 * 注意: 緩衝區大小由 SEIAL_PRINTF_STATIC_SIZE 定義
 */
#define serialPrintf(format, ...)                    \
    sprintf(serialPrintfCache, format, __VA_ARGS__); \
    serialPrint(serialPrintfCache)

/**
 * serialRead() - 阻塞式讀取單一字元
 * @return  接收到的字元
 *
 * 等待直到有資料可讀取
 */
char serialRead() {
    while (!interruptByReceiveUART());  // 等待接收中斷旗標
    return RCREG;                       // 讀取並回傳資料
}

/* 串列埠回呼函式指標 */
void (*serialOnReadLine)(char* line, byte len);  // 收到完整一行時的回呼
void (*serialOnReadChar)(char c);                // 收到單一字元時的回呼

/* 串列埠接收緩衝區 */
char serialBuffer[64];       // 接收緩衝區 (64 位元組)
byte serialBufferLen = 0;    // 目前緩衝區長度
char serialLastChar = '\0';  // 上一個接收的字元 (用於 CRLF 處理)

/**
 * processSerialReceive() - 處理串列埠接收
 * @return  true = 有處理資料, false = 無資料
 *
 * 此函式應在主迴圈或中斷服務常式中呼叫
 * 功能:
 * - 處理溢位錯誤
 * - 處理退格鍵 (0x7F)
 * - 處理換行 (CR/LF/CRLF)
 * - 回顯輸入字元
 * - 呼叫回呼函式
 */
bool processSerialReceive() {
    if (interruptByReceiveUART()) {
        // 清除溢位錯誤
        if (serialReceiveOverrunError()) {
            serialReceiveEnable(0);  // 停用接收
            Nop();                   // 短暫延遲
            serialReceiveEnable(1);  // 重新啟用接收
        }
        char c = serialRead();
        // 若有訊框錯誤則跳過此字元
        if (!serialReceiveFramingError()) {
            switch (c) {
            case '\x7f':  // DEL 鍵 (退格)
                if (!serialBufferLen)
                    break;
                // 回顯退格效果: 退格 + 空格 + 退格
                serialWrite('\b');
                serialWrite(' ');
                serialWrite('\b');
                serialBuffer[--serialBufferLen] = '\0';
                break;
            case '\r':  // CR (Carriage Return)
            case '\n':  // LF (Line Feed)
                // 處理 CRLF: 若為 LF 且上一個是 CR 則跳過
                if (c == '\n' && !serialBufferLen && serialLastChar == '\r')
                    break;
                serialWrite('\n');  // 回顯換行
                serialBuffer[serialBufferLen] = '\0';
                // 呼叫行讀取完成回呼
                if (serialOnReadLine)
                    serialOnReadLine(serialBuffer, serialBufferLen);
                serialBuffer[serialBufferLen = 0] = '\0';  // 清空緩衝區
                break;
            case 0xff:  // 無效字元，跳過
                break;
            default:
                // 呼叫字元接收回呼
                if (serialOnReadChar)
                    serialOnReadChar(c);
                // 檢查緩衝區是否有空間
                if (serialBufferLen < (byte)sizeof(serialBuffer) - 1) {
                    serialWrite(c);                       // 回顯字元
                    serialBuffer[serialBufferLen++] = c;  // 加入緩衝區
                }
            }
            serialLastChar = c;  // 記錄最後字元
        }
        return true;
    }
    return false;
}

#pragma endregion UART

/* ========== 伺服馬達控制 / Servo Motor Control ========== */
/**
 * setCCP1ServoAngle() - 設定伺服馬達角度
 * @param angle     目標角度 (0~180 度)
 * @param prescale  Timer2 預分頻比數值 (1, 4, 16)
 *
 * 伺服馬達 PWM 信號說明:
 * - 週期: 20ms (50Hz)
 * - 脈寬: 0.45ms ~ 2.45ms 對應 0° ~ 180°
 *
 * 脈寬計算公式:
 * pulse_width = 450µs + (angle / 180°) × (2450µs - 450µs)
 *
 * 注意: 使用前需先設定:
 * 1. 設定 Timer2 週期為 20ms
 * 2. 設定 CCP1 為 PWM 模式
 * 3. 設定 RC2 為輸出
 */
#define setCCP1ServoAngle(angle, prescale) setCCP1PwmDutyCycle(450 + ((2450 - 450) / 180.0) * (angle), prescale)

/* ========== LED 二進位顯示巨集 / LED Binary Display Macros ========== */

/**
 * displayBinary3Pins() - 在指定腳位顯示 3 位元二進位值
 *                        Display 3-bit binary value on specified pins
 * @param n     數值 (0-7) / Value (0-7)
 * @param pin0  bit 0 (LSB) 腳位 / bit 0 (LSB) pin
 * @param pin1  bit 1 腳位 / bit 1 pin
 * @param pin2  bit 2 (MSB) 腳位 / bit 2 (MSB) pin
 */
#define displayBinary3Pins(n, pin0, pin1, pin2) \
    do {                                        \
        digitalWrite(pin0, ((n) >> 0) & 1);     \
        digitalWrite(pin1, ((n) >> 1) & 1);     \
        digitalWrite(pin2, ((n) >> 2) & 1);     \
    } while (0)

/**
 * displayBinary3() - 在 RD0-RD2 顯示 3 位元二進位值
 *                    Display 3-bit binary value on RD0-RD2
 * @param n  數值 (0-7) / Value (0-7)
 */
#define displayBinary3(n) displayBinary3Pins(n, PIN_RD0, PIN_RD1, PIN_RD2)

/**
 * displayBinary4Pins() - 在指定腳位顯示 4 位元二進位值
 *                        Display 4-bit binary value on specified pins
 * @param n     數值 (0-15) / Value (0-15)
 * @param pin0  bit 0 (LSB) 腳位 / bit 0 (LSB) pin
 * @param pin1  bit 1 腳位 / bit 1 pin
 * @param pin2  bit 2 腳位 / bit 2 pin
 * @param pin3  bit 3 (MSB) 腳位 / bit 3 (MSB) pin
 */
#define displayBinary4Pins(n, pin0, pin1, pin2, pin3) \
    do {                                              \
        digitalWrite(pin0, ((n) >> 0) & 1);           \
        digitalWrite(pin1, ((n) >> 1) & 1);           \
        digitalWrite(pin2, ((n) >> 2) & 1);           \
        digitalWrite(pin3, ((n) >> 3) & 1);           \
    } while (0)

/**
 * displayBinary4() - 在 RD0-RD3 顯示 4 位元二進位值
 *                    Display 4-bit binary value on RD0-RD3
 * @param n  數值 (0-15) / Value (0-15)
 */
#define displayBinary4(n) displayBinary4Pins(n, PIN_RD0, PIN_RD1, PIN_RD2, PIN_RD3)
