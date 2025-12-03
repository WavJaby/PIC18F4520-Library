/*
 * ============================================================================
 * PIC18F4520 微控制器綜合應用程式
 * ============================================================================
 *
 * 【程式概述】
 * 這是一個 PIC18F4520 微控制器的綜合應用程式，整合了多種外設控制功能：
 * - LED 二進位顯示控制 (RD0-RD3)
 * - 伺服馬達角度控制 (RC2/CCP1)
 * - PWM 脈寬調變輸出 (RC1/CCP2)
 * - ADC 類比數位轉換 (RA0/AN0)
 * - UART 串列通訊 (RC6/TX, RC7/RX)
 * - 外部按鈕中斷處理 (RB0)
 * - 計時器中斷處理 (Timer1, Timer2, Timer3)
 *
 * 【硬體接線圖】
 *
 *   PIC18F4520
 *   ┌──────────────┐
 *   │ RA0/AN0  ────┼──── 電位器（可變電阻）中間腳
 *   │ RB0      ────┼──── 按鈕（接地，內部上拉）
 *   │ RC1/CCP2 ────┼──── LED（PWM 亮度控制）
 *   │ RC2/CCP1 ────┼──── 伺服馬達信號線
 *   │ RD0      ────┼──── LED 0（最低位元）
 *   │ RD1      ────┼──── LED 1
 *   │ RD2      ────┼──── LED 2
 *   │ RD3      ────┼──── LED 3（最高位元）
 *   │ RC6/TX   ────┼──── UART 傳送
 *   │ RC7/RX   ────┼──── UART 接收
 *   └──────────────┘
 *
 * ============================================================================
 */

#include <builtins.h>
#include <language_support.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define _XTAL_FREQ 4000000  // 內部時脈頻率 4MHz
#include "lib.h"


/*
 * ============================================================================
 * 晶片配置位元設定
 * ============================================================================
 */
#pragma config OSC = INTIO67  // 振盪器：內部振盪器，RA6/RA7 作為 I/O
#pragma config WDT = OFF      // 看門狗計時器：關閉
#pragma config PWRT = OFF     // 上電延遲計時器：關閉
#pragma config BOREN = ON     // 低電壓重置：啟用
#pragma config PBADEN = OFF   // PORTB 類比輸入：關閉（作為數位 I/O）
#pragma config LVP = OFF      // 低電壓程式燒錄：關閉
#pragma config CPD = OFF      // 資料 EEPROM 保護：關閉


/*
 * ============================================================================
 * 功能啟用旗標（全域變數）
 * ============================================================================
 * 這些布林變數用於控制各項功能的啟用/停用
 *
 * 【注意】功能衝突說明：
 * - RD0-RD3 LED 功能互斥（同時只能啟用一個）
 * - RC1/CCP2 PWM 功能互斥：enableLedPwmADC, enableLedFlashADC
 * - UART 輸入功能互斥：enableSetServoAngleUart, enableUartToBinary
 */
bool enableLedBinaryADC,              // ADC 值以二進位顯示在 LED (RD0-RD3)
     enableServoADC,                  // ADC 值控制伺服馬達角度
     enableLedPwmADC,                 // ADC 值控制 LED PWM 亮度 (RC1)
     enableLedMarqueeADC,             // ADC 值控制 LED 跑馬燈-多顆 (RD0-RD3)
     enableBinaryButtonCount,         // 按鈕計數以二進位顯示在 LED
     enableServoTurnRangeSwitchButton,// 按鈕切換伺服馬達轉動範圍（5 種）
     enableSetServoAngleUart,         // 透過 UART 設定伺服馬達角度
     enableServoTurnAngleButton,      // 按鈕切換伺服馬達固定角度
     enableLedFlashADC,               // ADC 值控制 LED 閃爍頻率（四段式）
     enableLedMarqueeADC_Single,      // ADC 值控制單顆 LED 跑馬燈 (RD0-RD3)
     enable10StateFromADC,            // ADC 映射 10 狀態，二進位顯示 0-9
     enable7TodayStateFromADC,        // ADC 映射 7 狀態，顯示日期
     enableEvenOddADC,                // ADC 遞增顯示奇數，遞減顯示偶數
     enableSequentialLedButton,       // 按鈕切換 LED 跑馬燈速度（3 顆 LED）
     enableSequentialLedButton_4,     // 按鈕切換 LED 跑馬燈速度（4 顆 LED）
     enableUartToBinary;              // UART 輸入數字以二進位顯示在 LED


/*
 * ============================================================================
 * 狀態變數（全域變數）
 * ============================================================================
 */
uint8_t servoTurnRangeState;   // 伺服馬達轉動範圍狀態（1-5）
uint16_t buttonClickCount;     // 按鈕點擊計數器
int currentServoAngle;         // 目前伺服馬達角度（0-180°）
uint16_t servoAngleState;      // 伺服馬達角度狀態機（0, 1, 2）


/*
 * ============================================================================
 * 奇偶數顯示功能狀態變數
 * ============================================================================
 * 用於追蹤 ADC 變化方向，決定顯示奇數或偶數
 */
bool adcIncreasing;            // ADC 變化方向（true=遞增, false=遞減）


/*
 * ============================================================================
 * LED 閃爍功能狀態變數
 * ============================================================================
 * 用於實現四段式 LED 閃爍頻率控制（使用 RC1/CCP2 PWM 輸出）
 * - 可變電阻最左：0.25 秒閃爍（快）
 * - 可變電阻中左：0.5 秒閃爍
 * - 可變電阻中右：0.75 秒閃爍
 * - 可變電阻最右：1.0 秒閃爍（慢）
 */
uint16_t ledFlashCounter;      // LED 閃爍計數器（累計 ADC 中斷次數）
uint16_t ledFlashThreshold;    // LED 閃爍門檻值（達到此值時切換 LED 狀態）
bool ledFlashState;            // LED 目前狀態（true=亮, false=滅）
uint8_t ledFlashZone;          // 目前閃爍區域（1, 2, 3, 4）


/*
 * ============================================================================
 * 按鈕控制 LED 跑馬燈功能狀態變數
 * ============================================================================
 * 3 顆 LED 版本 (RD0-RD2)：0.25s, 0.5s, 1.0s
 * 4 顆 LED 版本 (RD0-RD3)：0.25s, 0.5s, 0.75s, 1.0s
 */
uint8_t seqLedSpeedState;      // 速度狀態（1, 2, 3 或 1, 2, 3, 4）
uint8_t seqLedPosition;        // 目前 LED 位置（0-2 或 0-3）
uint8_t seqLedCounter;         // 計時計數器（Timer1 中斷次數）
uint8_t seqLedThreshold;       // 計時門檻值（達到此值時切換 LED）


/*
 * ============================================================================
 * led4Bit - LED 4 位元顯示函式
 * ============================================================================
 * 【功能】將 4 位元數值（0-15）顯示在 RD0-RD3 四顆 LED 上
 *
 * 【參數】value - 要顯示的數值（取低 4 位元）
 *
 * 【接線】
 *   RD0: 最低位元 (bit 0)
 *   RD1: 位元 1
 *   RD2: 位元 2
 *   RD3: 最高位元 (bit 3)
 *
 * 【範例】
 *   value = 5 (0b0101) → RD0=1, RD1=0, RD2=1, RD3=0
 *   value = 15 (0b1111) → 全亮
 *   value = 0 (0b0000) → 全滅
 */
void led4Bit(byte value) {
  digitalWrite(PIN_RD0, (value >> 0) & 0b1);  // 取出 bit 0 並輸出
  digitalWrite(PIN_RD1, (value >> 1) & 0b1);  // 取出 bit 1 並輸出
  digitalWrite(PIN_RD2, (value >> 2) & 0b1);  // 取出 bit 2 並輸出
  digitalWrite(PIN_RD3, (value >> 3) & 0b1);  // 取出 bit 3 並輸出
}


/*
 * ============================================================================
 * onReadLine - UART 串列通訊接收一行資料回呼函式
 * ============================================================================
 * 【功能】處理從串列埠接收到的完整一行文字
 *
 * 【參數】
 *   line - 接收到的字串指標
 *   len  - 字串長度
 *
 * 【支援的命令】
 *   'r'  - 重置所有狀態
 *   數字 - 根據啟用的功能執行不同動作：
 *          enableSetServoAngleUart: 設定伺服馬達角度（-90 到 +90）
 *          enableUartToBinary: 以二進位顯示在 LED（0-15）
 *
 * 【注意】enableSetServoAngleUart 和 enableUartToBinary 不可同時啟用
 */
void onReadLine(char *line, byte len) {
  /*
   * 【重置命令】收到 'r' 時重置所有狀態
   */
  if (*line == 'r') {
    buttonClickCount = 0;
    servoAngleState = 0;
    servoTurnRangeState = 1;
    seqLedSpeedState = 1;
    seqLedPosition = 0;
    seqLedCounter = 0;
    seqLedThreshold = 1;

    if (enableBinaryButtonCount) {
      led4Bit((byte)buttonClickCount);
    }
    printf("Reset OK\n");
    return;
  }

  /*
   * 【功能】透過 UART 設定伺服馬達角度
   *
   * 輸入範圍轉換：
   *   輸入 -90 → 實際角度 0°
   *   輸入 0   → 實際角度 90°
   *   輸入 90  → 實際角度 180°
   */
  if (enableSetServoAngleUart) {
    int i;
    // 解析輸入的角度值（支援負數）
    if (*line == '-')
      i = -atoi(line + 1);  // 負數：跳過負號後轉換
    else
      i = atoi(line);       // 正數：直接轉換

    // 將輸入角度轉換為 0-180° 範圍
    currentServoAngle = (i + 90) % 181;
    if (currentServoAngle < 0)
      currentServoAngle += 180;

    printf("Servo Angle:%d\n", currentServoAngle);
    setCCP1ServoAngle(currentServoAngle, 16);
  }

  /*
   * 【功能】UART 輸入數字以二進位顯示在 LED
   *
   * 輸入範圍：0-15（超過 15 會被限制為 15）
   *
   * 範例：
   * ┌───────┬─────────┬──────────────────┐
   * │ 輸入  │ 二進位  │ LED (RD3-RD0)    │
   * ├───────┼─────────┼──────────────────┤
   * │   0   │  0000   │ ░░░░             │
   * │   5   │  0101   │ ░█░█             │
   * │  15   │  1111   │ ████             │
   * │  20   │  1111   │ ████ (限制為 15) │
   * └───────┴─────────┴──────────────────┘
   */
  if (enableUartToBinary) {
    int value = atoi(line);

    // 限制數值範圍 0-15
    if (value < 0) value = 0;
    if (value > 15) value = 15;

    led4Bit((byte)value);

    // 輸出確認訊息
    printf("Binary:%d -> %d%d%d%d\n", value,
           (value >> 3) & 1, (value >> 2) & 1,
           (value >> 1) & 1, (value >> 0) & 1);
  }
}


/*
 * ============================================================================
 * onReadChar - UART 串列通訊接收單一字元回呼函式
 * ============================================================================
 * 【功能】處理從串列埠接收到的單一字元（目前未使用）
 *
 * 【參數】c - 接收到的字元
 */
void onReadChar(char c) {}


/*
 * ============================================================================
 * H_ISR - 高優先級中斷服務程式
 * ============================================================================
 * 【功能】處理所有高優先級中斷事件
 *
 * 【處理的中斷源】
 *   1. RB0 外部按鈕中斷 - 處理按鈕點擊事件
 *   2. Timer1 溢位中斷 - LED 跑馬燈計時
 *   3. Timer3 溢位中斷 - （保留）
 *   4. ADC 轉換完成中斷 - 處理類比輸入
 *
 * 【中斷處理流程】
 *   檢查中斷源 → 執行對應處理 → 清除中斷旗標
 */
uint16_t lastADC;  // 上一次 ADC 讀取值（用於平均濾波和方向判斷）

void __interrupt(high_priority) H_ISR() {

  /*
   * ========================================
   * RB0 外部按鈕中斷處理
   * ========================================
   * 觸發條件：RB0 腳位電平變化（按鈕按下）
   */
  if (interruptByRB0External()) {
    __delay_ms(200);  // 消抖延遲，避免按鈕彈跳造成多次觸發
    printf("Button%d\n", buttonClickCount++);

    // 【功能】以二進位方式顯示按鈕計數
    if (enableBinaryButtonCount) {
      led4Bit((byte)buttonClickCount);
    }

    /*
     * 【功能】切換伺服馬達轉動範圍狀態（5 種範圍）
     *
     * State 1: 90° ↔ 180°  (上半範圍)
     * State 2: 0° ↔ 180°   (全範圍)
     * State 3: 0° ↔ 90°    (下半範圍)
     * State 4: 45° ↔ 135°  (中間範圍)
     * State 5: Sleep       (停止擺動)
     */
    if (enableServoTurnRangeSwitchButton) {
      servoTurnRangeState = (servoTurnRangeState % 5) + 1;
      printf("Servo Range State:%d\n", servoTurnRangeState);
    }

    /*
     * 【功能】按鈕控制伺服馬達角度增量
     * 使用狀態機切換不同的角度增量：45° → 90° → 180° → 循環
     */
    if (enableServoTurnAngleButton) {
      switch (servoAngleState) {
      case 0:
        currentServoAngle += 45;
        break;
      case 1:
        currentServoAngle += 90;
        break;
      case 2:
        currentServoAngle += 180;
        break;
      }
      servoAngleState = (servoAngleState + 1) % 3;
      currentServoAngle %= 180;
      setCCP1ServoAngle(currentServoAngle, 16);
    }

    /*
     * 【功能】按鈕切換 LED 跑馬燈速度（3 顆 LED）
     * State 1: 0.25s, State 2: 0.5s, State 3: 1.0s
     */
    if (enableSequentialLedButton) {
      seqLedSpeedState = (seqLedSpeedState % 3) + 1;
      switch (seqLedSpeedState) {
      case 1: seqLedThreshold = 1; break;  // 0.25s
      case 2: seqLedThreshold = 2; break;  // 0.5s
      case 3: seqLedThreshold = 4; break;  // 1.0s
      }
      seqLedCounter = 0;
      printf("SeqLED State:%d\n", seqLedSpeedState);
    }

    /*
     * 【功能】按鈕切換 LED 跑馬燈速度（4 顆 LED）
     * State 1: 0.25s, State 2: 0.5s, State 3: 0.75s, State 4: 1.0s
     */
    if (enableSequentialLedButton_4) {
      seqLedSpeedState = (seqLedSpeedState % 4) + 1;
      switch (seqLedSpeedState) {
      case 1: seqLedThreshold = 1; break;  // 0.25s
      case 2: seqLedThreshold = 2; break;  // 0.5s
      case 3: seqLedThreshold = 3; break;  // 0.75s
      case 4: seqLedThreshold = 4; break;  // 1.0s
      }
      seqLedCounter = 0;
      printf("SeqLED4 State:%d\n", seqLedSpeedState);
    }

    clearInterrupt_RB0External();
  }

  /*
   * ========================================
   * Timer1 溢位中斷處理
   * ========================================
   * 觸發條件：Timer1 計數器溢位
   * 週期：250ms
   * 用途：LED 跑馬燈計時
   */
  if (interruptByTimer1Overflow()) {
    setTimer1InterruptPeriod(250000, 8);  // 重設週期 250ms

    /*
     * LED 跑馬燈計時處理（3 顆 LED）
     * 使用 RD0, RD1, RD2 依序點亮
     */
    if (enableSequentialLedButton) {
      seqLedCounter++;
      if (seqLedCounter >= seqLedThreshold) {
        seqLedCounter = 0;
        seqLedPosition = (seqLedPosition + 1) % 3;
        digitalWrite(PIN_RD0, seqLedPosition == 0 ? 1 : 0);
        digitalWrite(PIN_RD1, seqLedPosition == 1 ? 1 : 0);
        digitalWrite(PIN_RD2, seqLedPosition == 2 ? 1 : 0);
      }
    }

    /*
     * LED 跑馬燈計時處理（4 顆 LED）
     * 使用 RD0, RD1, RD2, RD3 依序點亮
     */
    if (enableSequentialLedButton_4) {
      seqLedCounter++;
      if (seqLedCounter >= seqLedThreshold) {
        seqLedCounter = 0;
        seqLedPosition = (seqLedPosition + 1) % 4;
        digitalWrite(PIN_RD0, seqLedPosition == 0 ? 1 : 0);
        digitalWrite(PIN_RD1, seqLedPosition == 1 ? 1 : 0);
        digitalWrite(PIN_RD2, seqLedPosition == 2 ? 1 : 0);
        digitalWrite(PIN_RD3, seqLedPosition == 3 ? 1 : 0);
      }
    }

    clearInterrupt_Timer1Overflow();
  }

  /*
   * ========================================
   * Timer3 溢位中斷處理（保留）
   * ========================================
   */
  if (interruptByTimer3Overflow()) {
    clearInterrupt_Timer3Overflow();
  }

  /*
   * ========================================
   * ADC 轉換完成中斷處理
   * ========================================
   * 觸發條件：ADC 完成一次類比轉數位轉換
   * ADC 數值範圍：0-1023（10 位元解析度）
   */
  if (interruptByADConverter()) {
    uint16_t currentADC = getADConverter();
    uint16_t ADC = (lastADC + currentADC) / 2;  // 簡單平均濾波

    // 【功能】ADC 控制伺服馬達角度（0-180°）
    if (enableServoADC) {
      setCCP1ServoAngle((int)(ADC * 180.0 / 1024), 16);
    }

    // 【功能】ADC 值以二進位顯示在 LED（0-15）
    if (enableLedBinaryADC) {
      led4Bit((byte)(ADC * 15.0 / 1023));
    }

    // 【功能】ADC 控制 LED PWM 亮度
    if (enableLedPwmADC) {
      setCCP2PwmDutyCycle((int)(ADC * 4100.0 / 1024), 16);
    }

    // 【功能】ADC 控制 LED 跑馬燈位置（多顆同時亮）
    if (enableLedMarqueeADC) {
      int marqueePosition = (int)(ADC * 8 / (1024 - 4));
      led4Bit((byte)(0b111100001111 >> marqueePosition));
    }

    /*
     * 【功能】ADC 控制 LED 四段式閃爍頻率
     *
     * ADC 範圍分配：
     * ┌──────────┬──────────┬──────────┬──────────┐
     * │  0~255   │ 256~511  │ 512~767  │ 768~1023 │
     * │ 0.25 秒  │ 0.5 秒   │ 0.75 秒  │ 1.0 秒   │
     * └──────────┴──────────┴──────────┴──────────┘
     */
    if (enableLedFlashADC) {
      uint8_t newZone;
      uint16_t newThreshold;

      if (ADC < 256) {
        newZone = 1;
        newThreshold = 50;   // 0.25 秒
      } else if (ADC < 512) {
        newZone = 2;
        newThreshold = 100;  // 0.5 秒
      } else if (ADC < 768) {
        newZone = 3;
        newThreshold = 150;  // 0.75 秒
      } else {
        newZone = 4;
        newThreshold = 200;  // 1.0 秒
      }

      if (newZone != ledFlashZone) {
        ledFlashZone = newZone;
        ledFlashCounter = 0;
      }

      ledFlashThreshold = newThreshold;
      ledFlashCounter++;

      if (ledFlashCounter >= ledFlashThreshold) {
        ledFlashCounter = 0;
        ledFlashState = !ledFlashState;
        setCCP2PwmDutyCycle(ledFlashState ? 1024 : 0, 16);
      }
    }

    /*
     * 【功能】ADC 控制單顆 LED 跑馬燈（左右兩端全滅）
     *
     * 6 個狀態：全滅 → RD0 → RD1 → RD2 → RD3 → 全滅
     */
    if (enableLedMarqueeADC_Single) {
      uint8_t ledPosition = ADC / 171;
      if (ledPosition > 5) ledPosition = 5;

      if (ledPosition == 0 || ledPosition == 5) {
        led4Bit(0b0000);
      } else {
        led4Bit((byte)(1 << (ledPosition - 1)));
      }
    }

    /*
     * 【功能】ADC 映射到 10 個狀態（二進位顯示 0-9）
     */
    if (enable10StateFromADC) {
      uint8_t state = ADC / 103;
      if (state > 9) state = 9;
      led4Bit((byte)state);
    }

    /*
     * 【功能】ADC 映射到 7 個狀態（顯示日期 2025/12/03）
     * 數字序列：2, 0, 2, 5, 12, 0, 3
     */
    if (enable7TodayStateFromADC) {
      static const byte dateDigits[7] = {2, 0, 2, 5, 12, 0, 3};
      uint8_t state = ADC / 147;
      if (state > 6) state = 6;
      led4Bit(dateDigits[state]);
    }

    /*
     * 【功能】ADC 遞增/遞減顯示奇數/偶數
     *
     * 電壓遞增 → 顯示奇數：1, 3, 5, 7, 9, 11, 13, 15
     * 電壓遞減 → 顯示偶數：0, 2, 4, 6, 8, 10, 12, 14
     */
    if (enableEvenOddADC) {
      int16_t adcDiff = (int16_t)currentADC - (int16_t)lastADC;
      if (adcDiff > 5) {
        adcIncreasing = true;
      } else if (adcDiff < -5) {
        adcIncreasing = false;
      }

      uint8_t state = ADC >> 7;  // ADC / 128
      if (state > 7) state = 7;

      byte displayValue = adcIncreasing ? (state * 2 + 1) : (state * 2);
      led4Bit(displayValue);
    }

    lastADC = currentADC;
    clearInterrupt_ADConverter();
  }
}


/*
 * ============================================================================
 * Lo_ISR - 低優先級中斷服務程式
 * ============================================================================
 * 【功能】處理低優先級中斷事件（UART 串列接收）
 *
 * 【說明】
 * 當高優先級中斷發生時，低優先級中斷會被暫停
 * 這確保了時間關鍵的操作（如 ADC、Timer）不會被串列通訊延遲
 */
void __interrupt(low_priority) Lo_ISR(void) {
  if (processSerialReceive())
    return;
}


/*
 * ============================================================================
 * main - 主程式入口點
 * ============================================================================
 * 【執行流程】
 *   1. 系統初始化（時脈、中斷）
 *   2. 外設初始化（按鈕、UART、LED、ADC、伺服馬達、PWM、Timer）
 *   3. 功能旗標設定
 *   4. 變數初始化
 *   5. 主迴圈（ADC 轉換、伺服馬達控制）
 */
int main(void) {

  /* ========================================
   * 第一階段：系統核心初始化
   * ======================================== */
  setIntrnalClock();
  setPortBPullup(PORTB_PULLUP_ENABLE);

  enableInterruptPriorityMode(1);
  enableGlobalInterrupt(1);
  enablePeripheralInterrupt(1);

  /* ========================================
   * 第二階段：按鈕輸入設定 (RB0)
   * ======================================== */
  pinMode(PIN_RB0, PIN_INPUT);
  enableInterrupt_RB0External();

  /* ========================================
   * 第三階段：UART 串列通訊設定
   * ======================================== */
  serialBegin(9600, 0b0);
  __delay_ms(100);
  serialOnReadLine = onReadLine;
  serialOnReadChar = onReadChar;

  /* ========================================
   * 第四階段：LED 輸出設定 (RD0-RD3)
   * ======================================== */
  pinMode(PIN_RD0, PIN_OUTPUT);
  pinMode(PIN_RD1, PIN_OUTPUT);
  pinMode(PIN_RD2, PIN_OUTPUT);
  pinMode(PIN_RD3, PIN_OUTPUT);

  digitalWrite(PIN_RD0, 0);
  digitalWrite(PIN_RD1, 0);
  digitalWrite(PIN_RD2, 0);
  digitalWrite(PIN_RD3, 0);

  /* ========================================
   * 第五階段：ADC 類比輸入設定 (RA0/AN0)
   * ======================================== */
  pinMode(PIN_RA0, PIN_INPUT);
  setANPinVoltageReferenceConfig(0, 0);
  setANPinADConfig(0b1110);
  setANPinAnalogChannelSelect(0);
  enableADConverter();
  enableInterrupt_ADConverter(1);

  /* ========================================
   * 第六階段：伺服馬達設定 (RC2/CCP1)
   * ======================================== */
  pinMode(PIN_RC2, PIN_OUTPUT);
  digitalWrite(PIN_RC2, 0);
  enableTimer2(TIMER2_PRESCALE_16, 0b0000);
  setTimer2InterruptPeriod(4100, 16, 1);
  setCCP1Mode(ECCP_MODE_PWM_HH);
  setCCP1ServoAngle(0, 16);

  /* ========================================
   * 第七階段：PWM LED 設定 (RC1/CCP2)
   * ======================================== */
  pinMode(PIN_RC1, PIN_OUTPUT);
  digitalWrite(PIN_RC1, 0);
  setCCP2Mode(ECCP_MODE_PWM_HH);

  /* ========================================
   * 第八階段：Timer1 定時器設定
   * ======================================== */
  enableTimer1(TIMER1_PRESCALE_8);
  setTimer1InterruptPeriod(250000, 8);  // 250ms
  enableInterrupt_Timer1Overflow(1);

  /* ========================================
   * 第九階段：功能旗標設定
   * ======================================== */
  enableLedBinaryADC = false;
  enableServoADC = false;
  enableLedPwmADC = false;
  enableLedMarqueeADC = false;
  enableBinaryButtonCount = false;
  enableServoTurnRangeSwitchButton = false;
  enableSetServoAngleUart = true;          // ← 啟用 UART 控制伺服馬達
  enableServoTurnAngleButton = false;
  enableLedFlashADC = false;
  enableLedMarqueeADC_Single = false;
  enable10StateFromADC = false;
  enable7TodayStateFromADC = false;
  enableEvenOddADC = false;
  enableSequentialLedButton = false;
  enableSequentialLedButton_4 = false;
  enableUartToBinary = false;              // ← 注意：與上面互斥

  /* ========================================
   * 第十階段：變數初始化
   * ======================================== */
  adcIncreasing = true;
  servoTurnRangeState = 1;
  seqLedSpeedState = 1;
  seqLedPosition = 0;
  seqLedCounter = 0;
  seqLedThreshold = 1;
  ledFlashCounter = 0;
  ledFlashThreshold = 50;
  ledFlashState = false;
  ledFlashZone = 1;
  buttonClickCount = 0;
  currentServoAngle = 0;
  servoAngleState = 0;
  lastADC = 0;

  /* ========================================
   * 第十一階段：初始化完成
   * ======================================== */
  printf("Ready!\n");

  /* ========================================
   * 主迴圈
   * ======================================== */
  while (true) {
    startADConverter();
    __delay_ms(5);

    /*
     * 伺服馬達範圍切換功能（5 種範圍）
     *
     * State 1: 90° ↔ 180°  (上半範圍)
     * State 2: 0° ↔ 180°   (全範圍)
     * State 3: 0° ↔ 90°    (下半範圍)
     * State 4: 45° ↔ 135°  (中間範圍)
     * State 5: Sleep       (停止擺動，固定 90°)
     */
    if (enableServoTurnRangeSwitchButton) {
      switch (servoTurnRangeState) {
      case 1:
        setCCP1ServoAngle(180, 16);
        __delay_ms(600);
        setCCP1ServoAngle(90, 16);
        __delay_ms(600);
        break;
      case 2:
        setCCP1ServoAngle(180, 16);
        __delay_ms(600);
        setCCP1ServoAngle(0, 16);
        __delay_ms(600);
        break;
      case 3:
        setCCP1ServoAngle(90, 16);
        __delay_ms(600);
        setCCP1ServoAngle(0, 16);
        __delay_ms(600);
        break;
      case 4:
        setCCP1ServoAngle(135, 16);
        __delay_ms(600);
        setCCP1ServoAngle(45, 16);
        __delay_ms(600);
        break;
      case 5:
        // Sleep: 停止擺動，固定在 90°
        setCCP1ServoAngle(90, 16);
        __delay_ms(100);
        break;
      }
    }
  }

  return 0;
}
