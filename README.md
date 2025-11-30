# PIC18F4520 Library

PIC18F4520 微控制器硬體抽象函式庫 / Hardware Abstraction Library for PIC18F4520

## 快速開始 / Quick Start

```c
#define _XTAL_FREQ 4000000  // 設定系統時脈頻率
#include "lib.h"

void main(void) {
    setIntrnalClock();                    // 設定內部時脈
    setPortBPullup(PORTB_PULLUP_ENABLE);  // 啟用 PORTB 上拉電阻
    enableInterruptPriorityMode(1);       // 啟用中斷優先權模式
    enableGlobalInterrupt(1);             // 啟用全域中斷
    enablePeripheralInterrupt(1);         // 啟用周邊中斷
    // ... 周邊設定
}
```

---

## 範例檔案說明 / Example Files

### `1_b.c` - ADC 控制 LED 閃爍速度
**功能**: 使用可變電阻(電位計)控制 LED 閃爍頻率，透過 UART 輸出狀態

```c
// GPIO 控制
pinMode(PIN_RA1, PIN_OUTPUT);         // 設定腳位為輸出
digitalWrite(PIN_RA1, 0);             // 輸出低電位
pinState(PIN_RA1);                    // 讀取輸出狀態

// ADC 類比轉數位
setANPinVoltageReferenceConfig(0, 0); // 使用 VDD/VSS 作為參考電壓
setANPinADConfig(0b1110);             // AN0 設為類比，其餘數位
setANPinAnalogChannelSelect(0);       // 選擇 AN0 通道
enableADConverter();                  // 啟用 ADC
enableInterrupt_ADConverter(1);       // 啟用 ADC 中斷(高優先權)
startADConverter();                   // 開始轉換
getADConverter();                     // 讀取轉換結果 (0-1023)

// UART 串列通訊
serialBegin(9600, 0b0);               // 初始化 UART，鮑率 9600
serialPrint("Ready\n");               // 傳送字串

// 外部中斷
enableInterrupt_RB0External();        // 啟用 RB0 外部中斷
```

### `2_a.c` - 伺服馬達掃描控制
**功能**: 按鈕切換伺服馬達掃描範圍 (45°-135° 或 0°-180°)

```c
// PWM 伺服馬達控制
pinMode(PIN_RC2, PIN_OUTPUT);              // CCP1 腳位設為輸出
enableTimer2(TIMER2_PRESCALE_16, 0b0000);  // 啟用 Timer2，預分頻 1:16
setTimer2InterruptPeriod(4100, 16, 1);     // 設定 PWM 週期 4.1ms
setCCP1Mode(ECCP_MODE_PWM_HH);             // 設定 PWM 模式
setCCP1ServoAngle(angle, 16);              // 設定伺服馬達角度 (0-180°)
```

### `2_b.c` - UART 控制伺服馬達模式
**功能**: 透過 UART 命令切換伺服馬達掃描速度模式

```c
// UART 回呼函式設定
serialOnReadLine = onReadLine;  // 設定行讀取回呼
serialOnReadChar = onReadChar;  // 設定字元讀取回呼
```

### `3_a.c` - ADC 直接控制伺服馬達
**功能**: 電位計 ADC 值直接映射到伺服馬達角度 (90°-180°)

```c
// 在 ADC 中斷中直接控制伺服馬達
if (interruptByADConverter()) {
    setCCP1ServoAngle((int)(90 + getADConverter() * 90L / 1024), 16);
    clearInterrupt_ADConverter();
}
```

### `3_b.c` - ADC 與 UART 計算應用
**功能**: 從 UART 接收數值，與 ADC 讀數進行計算並輸出

```c
// UART 接收並解析數值
void onReadLine(char *line, byte len) {
    count = atoi(line);  // 將接收的字串轉為數字
}
```

---

## 典型應用模式 / Common Patterns

### 中斷服務常式結構
```c
void __interrupt(high_priority) H_ISR() {
    if (interruptByXXX()) {
        // 處理中斷
        clearInterrupt_XXX();  // 必須清除中斷旗標
    }
}

void __interrupt(low_priority) Lo_ISR(void) {
    if (processSerialReceive())  // UART 接收處理
        return;
}
```

### ADC 連續取樣
```c
while (1) {
    startADConverter();
    __delay_us(10);  // 等待取樣完成
}
```

### 伺服馬達初始化流程
```c
pinMode(PIN_RC2, PIN_OUTPUT);
digitalWrite(PIN_RC2, 0);
enableTimer2(TIMER2_PRESCALE_16, 0b0000);
setTimer2InterruptPeriod(4100, 16, 1);
setCCP1Mode(ECCP_MODE_PWM_HH);
setCCP1ServoAngle(90, 16);  // 初始角度
```

---

## API 參考 / API Reference

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

### GPIO 腳位控制

#### Functions
```c
// 腳位方向設定
pinMode(pin, mode);           // mode: PIN_INPUT / PIN_OUTPUT
digitalWrite(pin, value);     // value: 0 / 1
pinState(pin);                // 讀取輸出鎖存器狀態

// PORTB 上拉電阻
setPortBPullup(state);        // state: PORTB_PULLUP_ENABLE / PORTB_PULLUP_DISABLE
```

#### Example
```c
pinMode(PIN_RA1, PIN_OUTPUT);  // 設定 RA1 為輸出
digitalWrite(PIN_RA1, 1);      // 輸出高電位
digitalWrite(PIN_RA1, !pinState(PIN_RA1));  // 切換輸出狀態
```

### ADC 類比數位轉換

#### Functions
```c
setANPinADConfig(value);                     // 設定類比/數位腳位 (0=類比, 1=數位)
setANPinVoltageReferenceConfig(conf0, conf1); // 設定參考電壓來源
setANPinAnalogChannelSelect(channel);        // 選擇 ADC 通道 (0-12)
enableADConverter();                         // 啟用 ADC 模組
startADConverter();                          // 開始轉換
getADConverter();                            // 讀取結果 (0-1023)
enableInterrupt_ADConverter(priority);       // 啟用 ADC 中斷
clearInterrupt_ADConverter();                // 清除中斷旗標
interruptByADConverter();                    // 檢查是否為 ADC 中斷
```

#### Example
```c
// ADC 初始化
pinMode(PIN_RA0, PIN_INPUT);
setANPinVoltageReferenceConfig(0, 0);  // 使用 VDD/VSS
setANPinADConfig(0b1110);              // AN0 類比，其餘數位
setANPinAnalogChannelSelect(0);        // 選擇 AN0
enableADConverter();
enableInterrupt_ADConverter(1);        // 高優先權

// 主迴圈中連續取樣
while (1) {
    startADConverter();
    __delay_us(10);
}

// 中斷處理
if (interruptByADConverter()) {
    int value = getADConverter();  // 0-1023
    clearInterrupt_ADConverter();
}
```

### PWM 伺服馬達控制

#### Functions
```c
setCCP1Mode(mode);                          // 設定 CCP1 模式
setCCP2Mode(mode);                          // 設定 CCP2 模式
setCCP1PwmDutyCycle(length_us, prescale);   // 設定 PWM 佔空比
setCCP2PwmDutyCycle(length_us, prescale);   // 設定 PWM 佔空比
setCCP1ServoAngle(angle, prescale);         // 設定伺服馬達角度 (0-180°)
```

#### Example
```c
// 伺服馬達初始化
pinMode(PIN_RC2, PIN_OUTPUT);
digitalWrite(PIN_RC2, 0);
enableTimer2(TIMER2_PRESCALE_16, 0b0000);
setTimer2InterruptPeriod(4100, 16, 1);     // PWM 週期
setCCP1Mode(ECCP_MODE_PWM_HH);             // PWM 模式

// 控制角度
setCCP1ServoAngle(0, 16);    // 0°
setCCP1ServoAngle(90, 16);   // 90°
setCCP1ServoAngle(180, 16);  // 180°
```
