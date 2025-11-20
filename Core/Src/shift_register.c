/**
  ******************************************************************************
  * @file    shift_register.c
  * @brief   Implementation of 74HC595 shift register control
  * @author  Traffic Light Shield Project
  * @date    2025-11-16
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "shift_register.h"

/* Private defines -----------------------------------------------------------*/
#define SHIFT_DELAY_US  1   // Microsecond delay for clock timing

/* Private function prototypes -----------------------------------------------*/
static void ShiftOut_Byte(uint8_t data);
static void Pulse_STCP(void);
static void Delay_us(uint32_t us);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize shift registers
 */
void ShiftReg_Init(void)
{
    // Reset pulse (active LOW)
    HAL_GPIO_WritePin(SR_Reset_GPIO_Port, SR_Reset_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);  // Hold reset for 1ms
    HAL_GPIO_WritePin(SR_Reset_GPIO_Port, SR_Reset_Pin, GPIO_PIN_SET);

    // Enable outputs (OE# LOW = enabled)
    HAL_GPIO_WritePin(SR_Enable_GPIO_Port, SR_Enable_Pin, GPIO_PIN_RESET);

    // Clear all outputs
    ShiftReg_Clear();
}

/**
 * @brief Update all shift registers with new data
 *
 * @param u1_data: Data for U1 (TL1 + PL1)
 * @param u2_data: Data for U2 (TL2 + PL2)
 * @param u3_data: Data for U3 (TL3 + TL4)
 */
void ShiftReg_Update(uint8_t u1_data, uint8_t u2_data, uint8_t u3_data)
{
    // Send data in REVERSE order: U3 → U2 → U1
    // This is because data shifts through the chain
    ShiftOut_Byte(u3_data);
    ShiftOut_Byte(u2_data);
    ShiftOut_Byte(u1_data);

    // Latch data to outputs
    Pulse_STCP();
}

/**
 * @brief Clear all LEDs
 */
void ShiftReg_Clear(void)
{
    ShiftReg_Update(0x00, 0x00, 0x00);
}

/**
 * @brief Enable or disable outputs
 *
 * @param enable: true = outputs enabled, false = outputs disabled (tri-state)
 */
void ShiftReg_EnableOutputs(bool enable)
{
    // OE# is active LOW
    if (enable) {
        HAL_GPIO_WritePin(SR_Enable_GPIO_Port, SR_Enable_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(SR_Enable_GPIO_Port, SR_Enable_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief Test function - cycles through all LEDs
 */
void ShiftReg_Test(void)
{
    uint8_t test_pattern[] = {
        // Test each LED individually
        U1_TL1_RED,
        U1_TL1_YELLOW,
        U1_TL1_GREEN,
        U1_PL1_RED,
        U1_PL1_GREEN,
        U1_PL1_BLUE,
    };

    // Test U1
    for (int i = 0; i < 6; i++) {
        ShiftReg_Update(test_pattern[i], 0x00, 0x00);
        HAL_Delay(300);
    }

    // Test U2
    for (int i = 0; i < 6; i++) {
        ShiftReg_Update(0x00, (1 << i), 0x00);
        HAL_Delay(300);
    }

    // Test U3
    for (int i = 0; i < 6; i++) {
        ShiftReg_Update(0x00, 0x00, (1 << i));
        HAL_Delay(300);
    }

    // Clear all
    ShiftReg_Clear();
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Shift out one byte (8 bits) using bit-banging
 *
 * @param data: Byte to shift out (MSB first)
 */
static void ShiftOut_Byte(uint8_t data)
{
    // Send 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        // Set data bit
        if (data & (1 << i)) {
            HAL_GPIO_WritePin(SR_DS_GPIO_Port, SR_DS_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(SR_DS_GPIO_Port, SR_DS_Pin, GPIO_PIN_RESET);
        }

        // Clock pulse (data is shifted on rising edge)
        HAL_GPIO_WritePin(SR_SHCP_GPIO_Port, SR_SHCP_Pin, GPIO_PIN_RESET);
        Delay_us(SHIFT_DELAY_US);
        HAL_GPIO_WritePin(SR_SHCP_GPIO_Port, SR_SHCP_Pin, GPIO_PIN_SET);
        Delay_us(SHIFT_DELAY_US);
        HAL_GPIO_WritePin(SR_SHCP_GPIO_Port, SR_SHCP_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief Pulse the storage clock to latch data
 */
static void Pulse_STCP(void)
{
    HAL_GPIO_WritePin(SR_STCP_GPIO_Port, SR_STCP_Pin, GPIO_PIN_RESET);
    Delay_us(SHIFT_DELAY_US);
    HAL_GPIO_WritePin(SR_STCP_GPIO_Port, SR_STCP_Pin, GPIO_PIN_SET);
    Delay_us(SHIFT_DELAY_US);
    HAL_GPIO_WritePin(SR_STCP_GPIO_Port, SR_STCP_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Simple microsecond delay (blocking)
 *
 * @param us: Microseconds to delay
 *
 * Note: This is a rough delay. For accurate timing, use a hardware timer.
 * At 80 MHz, each loop iteration is roughly 1/80 MHz = 12.5 ns
 * We need ~80 iterations for 1 microsecond
 */
static void Delay_us(uint32_t us)
{
    // Very rough delay - adjust multiplier based on actual timing
    // With optimization, this might not be accurate
    volatile uint32_t count = us * 80 / 4;  // Rough approximation
    while (count--) {
        __NOP();  // No operation - prevents optimization
    }
}
