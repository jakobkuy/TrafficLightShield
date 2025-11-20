/**
  ******************************************************************************
  * @file    shift_register.h
  * @brief   Header file for 74HC595 shift register control
  * @author  Traffic Light Shield Project
  * @date    2025-11-16
  ******************************************************************************
  * @attention
  *
  * This driver controls three daisy-chained 74HC595D shift registers
  * connected to the Traffic Light Shield.
  *
  * Hardware connections:
  *   SR_DS (PB_05)    - Serial Data Input (to U1 pin 14)
  *   SR_SHCP (PC_10)  - Shift Clock (to all pin 11)
  *   SR_STCP (PB_12)  - Storage/Latch Clock (to all pin 12)
  *   SR_Enable (PC_07)- Output Enable, active LOW (to all pin 13)
  *   SR_Reset (PA_09) - Master Reset, active LOW (to all pin 10)
  *
  * Data flow: U1 → U2 → U3 (via Q7S pins)
  * IMPORTANT: Send data in REVERSE order (U3, U2, U1)
  *
  ******************************************************************************
  */

#ifndef INC_SHIFT_REGISTER_H_
#define INC_SHIFT_REGISTER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LED bit positions for each shift register
 *
 * Note: Only Q0-Q5 (6 outputs) are used per register. Q6-Q7 are unused.
 */

// U1 Register (TL1 + PL1)
#define U1_TL1_RED      (1 << 0)  // Q0
#define U1_TL1_YELLOW   (1 << 1)  // Q1
#define U1_TL1_GREEN    (1 << 2)  // Q2
#define U1_PL1_RED      (1 << 3)  // Q3
#define U1_PL1_GREEN    (1 << 4)  // Q4
#define U1_PL1_BLUE     (1 << 5)  // Q5

// U2 Register (TL2 + PL2) - CORRECTED per hardware schematics
#define U2_TL2_RED      (1 << 0)  // Q0
#define U2_TL2_YELLOW   (1 << 1)  // Q1
#define U2_TL2_GREEN    (1 << 2)  // Q2
#define U2_PL2_RED      (1 << 3)  // Q3 (3x LEDs in parallel)
#define U2_PL2_GREEN    (1 << 4)  // Q4 (3x LEDs in parallel)
#define U2_PL2_BLUE     (1 << 5)  // Q5 (3x LEDs in parallel)

// U3 Register (TL3 + TL4) - CORRECTED per hardware schematics
#define U3_TL3_RED      (1 << 0)  // Q0
#define U3_TL3_YELLOW   (1 << 1)  // Q1
#define U3_TL3_GREEN    (1 << 2)  // Q2
#define U3_TL4_RED      (1 << 3)  // Q3
#define U3_TL4_YELLOW   (1 << 4)  // Q4
#define U3_TL4_GREEN    (1 << 5)  // Q5

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize shift registers
 *
 * Sets up GPIO pins and resets all shift registers to known state.
 * Call this once during system initialization.
 */
void ShiftReg_Init(void);

/**
 * @brief Update all LEDs with new data
 *
 * @param u1_data: Data for U1 register (TL1 + PL1)
 * @param u2_data: Data for U2 register (TL2 + PL2)
 * @param u3_data: Data for U3 register (TL3 + TL4)
 *
 * Example:
 *   ShiftReg_Update(U1_TL1_RED, U2_TL2_GREEN, U3_TL4_YELLOW);
 */
void ShiftReg_Update(uint8_t u1_data, uint8_t u2_data, uint8_t u3_data);

/**
 * @brief Clear all LEDs (turn everything off)
 */
void ShiftReg_Clear(void);

/**
 * @brief Enable or disable LED outputs
 *
 * @param enable: true = LEDs on, false = LEDs off (tri-state)
 *
 * Note: This controls the OE# pin (active LOW)
 */
void ShiftReg_EnableOutputs(bool enable);

/**
 * @brief Test function - cycle through all LEDs
 *
 * Useful for hardware verification. Lights up each LED in sequence.
 */
void ShiftReg_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SHIFT_REGISTER_H_ */
