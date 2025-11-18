# IS1300 Traffic Light Shield - Technical Reference

## Project Overview
- **Course**: IS1300 Embedded Systems, KTH | **Deadline**: December 19, 2025
- **Board**: Nucleo-L476RG (STM32L476RG MCU) | **Team**: Groups of 2
- **Project**: Traffic intersection with 4 lights (TL1-TL4) + 2 pedestrian crossings (PL1-PL2)
- **Submission Format**: `PRO1_[Your_Canvas_Name]` (e.g., `PRO1_Jakob_Andersson`)

---

## Grading Summary (11 pts max)

| Category | Points | Key Requirements |
|----------|--------|------------------|
| **Planning/Architecture/Testing** | 0-2 | TDD tests + architecture diagrams (BOTH required for points) |
| **Implementation & Complexity** | 0-3 | Task combinations (see below) |
| **Report & Documentation** | 0-2 | Professional report + code comments + module interfaces |
| **Real-Time Tasks (RTOS)** | 0-3 | RTOS Lab mandatory (1pt) + FreeRTOS impl optional (0-2pts, needs pre-approval) |
| **Deadline Bonus** | +1 | Submit before Dec 19, 2025 |
| **Exam Bonus** | +1 | Based on exam performance |

**CRITICAL**: Minimum 1 point in EVERY category or project fails. Minimum 4 points total for grade E.

**Grade Mapping**: A=11pts | B=9-10 | C=7-8 | D=5-6 | E=4

---

## Implementation Tasks (Category 2: 0-3 points)

**Point Allocation (strict combinations)**:
- **1 point**: Task 1 AND 2
- **2 points**: Task 1 AND 2 AND (Task 4 OR 5)
- **3 points**: Task 1 AND 2 AND 3 AND (Task 4 OR 5)

### Task 1: Single Pedestrian Crossing (PL1 only) - REQUIRED
- R1.1: Init: ped red, car green
- R1.2: Button press → blue LED toggles @ `toggleFreq` until crossing green
- R1.3: All car signals crossing → red after `pedestrianDelay` ms (with orange transition)
- R1.4: Ped green for `walkingDelay` ms
- R1.5: Ped red when any car green/orange, green only when all crossing cars red
- R1.6: Car transitions: red→orange(`orangeDelay`)→green, green→orange(`orangeDelay`)→red

**Parameters**: `toggleFreq`, `pedestrianDelay`, `walkingDelay`, `orangeDelay`

### Task 2: Road Crossing Traffic Control (4 lights, no peds) - REQUIRED
- R2.1: Forward/right turns only (no left)
- R2.2: Prevent overlapping car paths
- R2.3: All transitions include orange phase (`orangeDelay` ms)
- R2.4: No cars → direction change every `greenDelay` ms
- R2.5: Stay green if cars active in allowed direction + no waiting cars
- R2.6: Car at red + active cars elsewhere → wait max `redDelayMax` ms
- R2.7: Car at red + no active cars → immediate green transition
- R2.8: Init: vertical (TL1,TL3) green, horizontal (TL2,TL4) red

**Parameters**: `orangeDelay`, `greenDelay`, `redDelayMax`

### Task 3: Complete Traffic System ⭐ (REQUIRED for 3 points)
- R3.1-3.2: Task 1 + 2 for ALL crossings and lights
- R3.3: **Only ONE pedestrian crossing green at a time** (critical!)
- R3.4: **Shift registers MUST use SPI interface** (GPIO bit-banging NOT acceptable)
- R3.5: Right turns allowed when ped crossing on right lane is green

### Task 4: Display & Brightness Control
- R4.1: LED brightness proportional to potentiometer voltage (via PWM on OE#)
- R4.2: OLED display with 8 countdown bars:
  - 2× Remaining `pedestrianDelay` (PL1, PL2)
  - 2× Remaining `walkingDelay` (PL1, PL2)
  - 2× Remaining `greenDelay` (vertical, horizontal)
  - 2× Remaining `redDelay` (vertical, horizontal)
- **Library**: https://github.com/afiskon/stm32-ssd1306

### Task 5: UART Configuration Interface
- Runtime parameter modification via UART2 (115200 baud)
- **4-byte protocol**: [Param ID][0x00][Value MSB][Value LSB]
- **Response**: ACK (0x01) if accepted, NACK (0x00) if rejected
- **Parameters**:
  - 0x01: toggleFreq (1-10 Hz)
  - 0x02: pedestrianDelay (1000-10000 ms)
  - 0x03: walkingDelay (1000-15000 ms)
  - 0x04: orangeDelay (500-3000 ms)
  - 0x05: greenDelay (2000-10000 ms)
  - 0x06: redDelayMax (5000-30000 ms)

---

## Testing Requirements (Category 1: Testing Component)

**For 1 point minimum, must have**:
1. **TDD approach**: Write test FIRST → fail → implement → pass → refactor
2. **Dedicated test file**: `Core/Src/tests.c` with execution function
3. **Test categories**:
   - Basic functionality (happy path: single LED, full cycle, button press)
   - Corner cases (all cars present, no cars, simultaneous buttons, button spam)
   - Boundary tests (min/max delay values)
   - Integration tests (complete cycles, pedestrian stops correct cars)
4. **Test execution**: Called from main.c (e.g., under `#ifdef RUN_TESTS`)
5. **Test results documented in report** with pass/fail table

**Test structure example**:
```c
void RunAllTests(void) {
    uint8_t passed = 0, failed = 0;
    if (Test_TL1_Red_LED_Only()) passed++; else failed++;
    if (Test_EdgeCase_AllCarsPresent()) passed++; else failed++;
    // Print summary
}
```

---

## Architecture Documentation (Category 1: Architecture Component)

**For 1 point minimum, report must include**:

### Hardware Architecture Diagrams:
1. System block diagram (MCU + peripherals)
2. Shift register chain diagram (data flow, U1→U2→U3)
3. Pin connection diagram/table

### Software Architecture Diagrams:
1. Module dependency diagram
2. Traffic light state machine diagram
3. Pedestrian crossing state machine diagram
4. Sequence diagram (≥1 scenario, e.g., pedestrian request)
5. Timing diagram (showing delays)

**All diagrams must have textual descriptions** explaining purpose, components, interactions, data flow, and design decisions.

---

## Report & Documentation Requirements (Category 3)

**Report Quality (1 point)**:
- Well-written, well-structured, relevant references (IEEE/APA format)
- **Required sections**: Title, Abstract, TOC, Introduction, Requirements, Architecture, Implementation, Testing, Results, Discussion, Conclusion, References
- All diagrams numbered, captioned, and referenced in text
- Professional academic tone, no errors

**Code Documentation (1 point)**:
- **Module interfaces described in report** (purpose, functions, dependencies)
- **File headers**: author, date, brief description
- **Function headers**: brief, details, params, return, notes, examples
- **Inline comments**: explain WHY (not what), complex logic, hardware constraints
- **Clear naming conventions**: self-documenting variable/function names

---

## Hardware Architecture

### MCU: STM32L476RG (Nucleo-L476RG Board)
- **Core**: ARM Cortex-M4 with FPU | **Flash**: 1 MB | **SRAM**: 128 KB
- **Clock**: Up to 80 MHz | **Voltage**: 3.3V logic

### Shift Registers: 3× 74HC595D (Daisy-Chained)
- **Per-pin current**: 35 mA max | **Total current**: 70 mA per chip max
- **Data ordering**: Send U3 FIRST → U2 → U1 LAST (reverse order due to daisy chain)
- **Only 6 of 8 outputs used** per register (Q0-Q5, Q6-Q7 unused)
- **Pin functions**:
  - DS (pin 14): Serial data input
  - Q7S (pin 9): Serial data output (daisy chain)
  - SHCP (pin 11): Shift clock (rising edge)
  - STCP (pin 12): Latch clock (rising edge)
  - OE# (pin 13): Output enable (active LOW, use for PWM brightness)
  - MR# (pin 10): Master reset (active LOW, keep HIGH)

---

## Pin Mappings (VERIFIED FROM SCHEMATICS)

### Shift Register Control (Hybrid SPI + GPIO)
```
Signal       MCU Pin   Pin#   Function              Notes
595_DS       PB_05     67     Serial Data           Can use SPI1_MOSI or GPIO
595_SHCP     PC_10     1      Shift Clock           Can use SPI3_SCK or GPIO
595_STCP     PB_12     54     Latch (Storage Clock) GPIO ONLY (not on SPI!)
595_Enable   PC_07     57     Output Enable (OE#)   TIM3_CH2 for PWM brightness
595_Reset    PA_09     59     Master Reset (MR#)    GPIO, initialize HIGH
```

**CRITICAL NOTES**:
- For Task 3 (SPI requirement), use SPI3 with SHCP on PC_10
- STCP is NOT on any SPI peripheral → must use GPIO
- Different SPI peripherals: DS on SPI1, SHCP on SPI3 → recommend GPIO bit-banging for Tasks 1-2
- OE# is active LOW: duty 0%=max brightness, 100%=off

### Car Detection Switches (All GPIO_Input with pull-down)
```
Signal       MCU Pin   Pin#
TL1_Car      PC_04     72
TL2_Car      PB_13     68
TL3_Car      PB_14     66
TL4_Car      PA_10     71
```

### Pedestrian Buttons (All GPIO_Input with pull-down)
```
Signal       MCU Pin   Pin#
PL1_Switch   PA_15     17
PL2_Switch   PB_07     21
```

### Potentiometer (ADC)
```
Signal       MCU Pin   Pin#   ADC Channel
Poti         PB_01     62     ADC1_IN16
```
**NOTE**: PB_01, NOT PC_04 (no conflict with TL1_Car)

### UART Communication (ST-Link Virtual COM)
```
UART_TX      PA_02     -      USART2_TX (115200 baud)
UART_RX      PA_03     -      USART2_RX
```

### I2C Peripherals (Sensors)
```
I2C_SDA      PB_09     -      I2C1_SDA
I2C_SCL      PB_08     -      I2C1_SCL
```

### OLED Display (SPI)
```
SPI_MOSI     PC_12     -      SPI3_MOSI
SPI_SCLK     PC_10     -      SPI3_SCLK (shared with 595_SHCP)
Disp_CS      PA_04     -      GPIO (chip select)
Disp_Reset   PC_14     -      GPIO (reset)
Disp_D/C     PB_00     -      GPIO (data/command)
```

### Debug LEDs
```
USR_LED1     PB_02     -      GPIO_Output
USR_LED2     PA_05     -      GPIO_Output (onboard green LED)
```

---

## LED Output Mapping (Shift Register Outputs)

### Register U1 (First in chain, receives data LAST)
```
Output   Pin    Connected To        Signal Name      Notes
Q0       15     TL1 Red            TL1_Red
Q1       1      TL1 Yellow         TL1_Yellow
Q2       2      TL1 Green          TL1_Green
Q3       3      PL1 Red            PL1_Red
Q4       4      PL1 Green          PL1_Green
Q5       5      PL1 Blue           PL1_Blue
Q6       6      NOT USED           -
Q7       7      NOT USED           -
```

### Register U2 (Middle)
```
Output   Pin    Connected To        Signal Name      Notes
Q0       15     TL2 Red            TL2_Red
Q1       1      TL2 Yellow         TL2_Yellow
Q2       2      TL2 Green          TL2_Green
Q3       3      PL2 Red            PL2_Red          3× LEDs parallel
Q4       4      PL2 Green          PL2_Green        3× LEDs parallel
Q5       5      PL2 Blue           PL2_Blue         3× LEDs parallel
Q6       6      NOT USED           -
Q7       7      NOT USED           -
```
**WARNING**: PL2 has 3× LEDs per color → 3× current draw!

### Register U3 (Last in chain, receives data FIRST)
```
Output   Pin    Connected To        Signal Name      Notes
Q0       15     TL3 Red            TL3_Red
Q1       1      TL3 Yellow         TL3_Yellow
Q2       2      TL3 Green          TL3_Green
Q3       3      TL4 Red            TL4_Red
Q4       4      TL4 Yellow         TL4_Yellow
Q5       5      TL4 Green          TL4_Green
Q6       6      NOT USED           -
Q7       7      NOT USED           -
```

---

## Shift Register Control Implementation

### Method A: GPIO Bit-Banging (Tasks 1, 2)
```c
void shift_byte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(DS_PORT, DS_PIN, (data >> i) & 0x01);
        HAL_GPIO_WritePin(SHCP_PORT, SHCP_PIN, HIGH); // Clock pulse
        HAL_GPIO_WritePin(SHCP_PORT, SHCP_PIN, LOW);
    }
}

void update_leds(uint8_t u3, uint8_t u2, uint8_t u1) {
    shift_byte(u3); shift_byte(u2); shift_byte(u1);  // U3 first!
    HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, HIGH);     // Latch
    HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, LOW);
}
```

### Method B: SPI (Task 3 - REQUIRED)
```c
void update_leds_spi(uint8_t u3, uint8_t u2, uint8_t u1) {
    uint8_t buffer[3] = {u3, u2, u1};
    HAL_SPI_Transmit(&hspi3, buffer, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, HIGH);  // Latch via GPIO
    HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, LOW);
}
```
**SPI Configuration**: Mode 0 (CPOL=0, CPHA=0), MSB first, 1-10 MHz

---

## PWM Brightness Control (Task 4)

Use TIM3_CH2 (PC_07) to control OE# pin for LED brightness:

```c
// Init: TIM3, ~1 kHz, ARR=999
void update_brightness(uint16_t adc_value) {
    uint16_t duty = (adc_value * 999) / 4095;  // Map ADC (0-4095) to duty (0-999)
    duty = 999 - duty;  // Invert: OE# is active LOW
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
}
```

---

## Configuration Parameters

```c
uint32_t toggleFreq = 2;         // Hz (blue LED toggle)
uint32_t pedestrianDelay = 3000; // ms (delay before cars stop)
uint32_t walkingDelay = 5000;    // ms (how long ped can walk)
uint32_t orangeDelay = 1000;     // ms (orange phase duration)
uint32_t greenDelay = 5000;      // ms (green when no cars)
uint32_t redDelayMax = 10000;    // ms (max wait at red)
```

---

## Current Implementation Status

### ✅ Completed
- Pin mapping verified from schematics
- STM32CubeIDE project configured (`TrafficLightShield`)
- System clock: 80 MHz
- GPIO configured (all pins labeled correctly)
- ADC1: IN16 (potentiometer), 12-bit, continuous mode, 247.5 cycle sampling
- TIM3 CH2: PWM, 1 kHz (PSC=79, ARR=999)
- USART2: 115200 baud
- Shift register driver: `Core/Inc/shift_register.h`, `Core/Src/shift_register.c`
- Functions: `ShiftReg_Init()`, `ShiftReg_Update(u1,u2,u3)`, `ShiftReg_Clear()`, `ShiftReg_Test()`
- LED bit definitions: `U1_TL1_RED`, etc.
- Test code: main.c cycles all LEDs red→yellow→green

### 📋 Next Steps
1. **Implement state machines**: traffic_light.c/h, pedestrian.c/h
2. **Add timing module**: timing.c/h for delay tracking
3. **Configure SPI3** (for Task 3)
4. **Integrate OLED** (Task 4): SSD1306 library
5. **Create test suite**: tests.c with TDD approach
6. **Write report**: All diagrams + documentation

---

## Code Organization

```
Core/
├── Inc/
│   ├── shift_register.h      ✅ Created
│   ├── traffic_light.h       📋 TODO
│   ├── pedestrian.h          📋 TODO
│   ├── timing.h              📋 TODO
│   └── tests.h               📋 TODO
└── Src/
    ├── main.c                ✅ Modified (test code)
    ├── shift_register.c      ✅ Created
    ├── traffic_light.c       📋 TODO
    ├── pedestrian.c          📋 TODO
    ├── timing.c              📋 TODO
    └── tests.c               📋 TODO
```

---

## Common Pitfalls & Solutions

1. **LEDs not lighting**: Check data order (U3→U2→U1), current limits (70mA/chip), OE# LOW, MR# HIGH
2. **SPI not working**: STCP is GPIO (not SPI), use correct SPI mode (MODE_0)
3. **PL2 LEDs dim**: Triple LEDs → check 70mA limit, may need to reduce other currents
4. **Button not responding**: Debounce inputs (200ms typical)
5. **Unplugging breaks debug**: On macOS, run `sudo ./stlink-fix.sh` before CubeIDE

---

## Quick Reference

### macOS ST-Link Fix
```bash
sudo ./stlink-fix.sh          # Before opening CubeIDE
# Use CubeIDE normally
sudo ./stlink-restore.sh      # When done debugging
```

### Build Status
- **Compiles**: ✅ 0 errors, 0 warnings
- **Flash usage**: ~2.3% (23,604 bytes / 1 MB)

### Debugging
```c
printf("State: %d\n", state);  // Via UART to ST-Link
```

---

## External Resources

- **SSD1306 Library**: https://github.com/afiskon/stm32-ssd1306
- **Datasheets**: `STM32L476xx.pdf`, `74HC_HCT595.pdf`, `SSD1306.pdf`, etc.
- **Reference Manual**: RM0351 (`en_DM00157440.pdf`)
- **Schematics**: `IS1300_TrafficLight_Schematics.pdf`

---

**Last Updated**: 2025-11-18
**Project Directory**: `/Users/jakob/dev/kth/IS1300/TrafficLightShield/`
**Build**: ✅ Compiling successfully
