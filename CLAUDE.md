# IS1300 Traffic Light Shield Project - Technical Documentation

## Project Overview

This is the IS1300 Embedded Systems Traffic Light Shield project using the STM32L476RG microcontroller (Nucleo-L476RG development board). The project simulates a complete traffic intersection control system with four traffic lights (TL1-TL4) and two pedestrian crossings (PL1-PL2).

**Key Facts:**
- **Course**: IS1300 Embedded Systems, KTH
- **Duration**: 5+ weeks (~120 hours total)
- **Team Size**: Groups of 2
- **Submission Deadline**: December 19, 2025
- **Development Board**: Nucleo-L476RG (STM32L476RG MCU)
- **Shield**: Traffic Light Shield with 74HC595D shift registers

## Hardware Architecture

### Microcontroller: STM32L476RG
- **Core**: ARM Cortex-M4 with FPU
- **Flash**: 1 MB
- **SRAM**: 128 KB
- **Clock**: Up to 80 MHz
- **Voltage**: 3.3V logic levels

### Main Components

#### 1. Three Daisy-Chained 74HC595D Shift Registers (U1, U2, U3)
The system uses three 8-bit shift registers to control all LEDs. **Critical: Only 6 of 8 outputs per register are actually used.**

**Shift Register Specifications:**
- **Per-pin current limit**: 35 mA maximum
- **Total current limit**: 70 mA per chip (all outputs combined)
- **Operating voltage**: 2.0V to 6.0V
- **Max shift frequency**: 100 MHz (typical)

**Pin Functions:**
- `DS` (pin 14): Serial Data Input
- `Q7S` (pin 9): Serial Data Output (for daisy-chaining)
- `SHCP` (pin 11): Shift Register Clock (shift on rising edge)
- `STCP` (pin 12): Storage Register Clock (latch on rising edge)
- `OE#` (pin 13): Output Enable (active LOW)
- `MR#` (pin 10): Master Reset (active LOW)
- `Q0-Q7` (pins 15, 1-7): Parallel outputs

**Connection Chain:**
```
MCU → U1(DS) → U1(Q7S) → U2(DS) → U2(Q7S) → U3(DS) → U3(Q7S)
```

**CRITICAL: Data Ordering**
When sending 24 bits to the daisy chain, **data must be sent in reverse order**:
- First send: U3 data (bits for rightmost register)
- Then send: U2 data (middle register)
- Last send: U1 data (leftmost register)

#### 2. Traffic Lights (TL1, TL2, TL3, TL4)
Each traffic light has 3 LEDs (Red, Yellow, Green) with current-limiting resistors.

**LED Resistor Values:**
- Red LEDs: 180Ω (typical forward voltage ~2.0V)
- Yellow LEDs: 560Ω (typical forward voltage ~2.1V)
- Green LEDs: 820Ω (typical forward voltage ~2.2V)

#### 3. Pedestrian Crossings (PL1, PL2)
Each crossing has:
- Red LED (walking prohibited) - 180Ω resistor
- Green LED (safe to walk) - 180Ω resistor
- Blue LED (button indicator) - 120Ω resistor
- Push button switch (SW5 for PL1, SW6/SW7/SW8 for PL2)

**SPECIAL NOTE FOR PL2:**
PL2 has **triple LEDs per color connected in parallel** (3x red, 3x green, 3x blue). This requires careful current consideration:
- Each color group draws 3x the current of PL1
- Must ensure total current stays within 70mA chip limit

#### 4. Car Detection Switches
Four switches (SW1-SW4) simulate car presence:
- Switch HIGH → car present
- Switch LOW → no car
- Connected to GPIO pins (TL1_Car through TL4_Car)

#### 5. OLED Display (SSD1306)
- **Resolution**: 128x64 pixels monochrome
- **Interface**: SPI
- **Driver IC**: SSD1306
- **Library**: Use existing library from https://github.com/afiskon/stm32-ssd1306

#### 6. Additional Peripherals
- **LIS2DW12TR**: 3-axis accelerometer (I2C interface)
- **SHT20**: Temperature/Humidity sensor (I2C interface)
- **Potentiometer**: Analog input for LED brightness control
- **Joystick**: 5-way control (center + 4 directions)
- **CAN Bus**: Available for communication
- **UART**: Serial communication via ST-Link virtual COM port

## Pin Mappings

### Critical Pin Assignments (VERIFIED BY MANUAL CROSS-CHECK)

#### Shift Register Control (Hybrid SPI + GPIO)
```
Pin Name        MCU Pin    MCU Pin#   Function              Peripheral
─────────────────────────────────────────────────────────────────────────
595_DS          PB_05      Pin 67     Serial Data          SPI1_MOSI / GPIO
595_SHCP        PC_10      Pin 1      Shift Clock          SPI3_SCK
595_STCP        PB_12      Pin 54     Storage Clock (Latch) GPIO (NOT on SPI)
595_Enable      PC_07      Pin 57     Output Enable        GPIO / TIM3_CH2 (PWM)
595_Reset       PA_09      Pin 59     Master Reset         GPIO
```

**IMPORTANT**: This is a **hybrid approach**, not pure SPI:
- `SHCP` can use SPI3_SCK peripheral (PC_10)
- `DS` can use SPI1_MOSI (PB_05) **NOTE: Different SPI peripheral than SHCP!**
- For SPI-based control, recommend using **GPIO bit-banging** to avoid SPI peripheral conflicts
- `STCP` (latch) **must** be controlled via GPIO (PB_12) - it's not on any SPI
- `OE` and `MR` are also GPIO-controlled
- PC_07 (595_Enable) can use TIM3_CH2 for PWM brightness control

#### I2C Peripherals
```
I2C_SDA         PB_09      I2C Data             I2C1_SDA
I2C_SCL         PB_08      I2C Clock            I2C1_SCL
```
Connected to: LIS2DW12TR accelerometer, SHT20 temp/humidity sensor

#### OLED Display (SPI)
```
SPI_MOSI        PC_12      Master Out           SPI3_MOSI
SPI_SCLK        PC_10      SPI Clock            SPI3_SCLK
Disp_CS         PA_04      Chip Select          GPIO
Disp_Reset      PC_14      Reset                GPIO
Disp_Data/Instr PB_00      Data/Command Select  GPIO
```

#### Car Switches (Digital Inputs)
```
Signal Name     MCU Pin    MCU Pin#   Function             Peripheral
───────────────────────────────────────────────────────────────────────
TL1_Car         PC_04      Pin 72     Traffic Light 1      GPIO Input
TL2_Car         PB_13      Pin 68     Traffic Light 2      GPIO Input
TL3_Car         PB_14      Pin 66     Traffic Light 3      GPIO Input
TL4_Car         PA_10      Pin 71     Traffic Light 4      GPIO Input
```

**NOTE**: TL1_Car shares pin PC_04 with Potentiometer - verify actual usage in your implementation!

#### Pedestrian Buttons (Digital Inputs)
```
Signal Name     MCU Pin    MCU Pin#   Function             Peripheral
───────────────────────────────────────────────────────────────────────
PL1_Switch      PA_15      Pin 17     Pedestrian 1         GPIO Input
PL2_Switch      PB_07      Pin 21     Pedestrian 2         GPIO Input
```

#### Potentiometer (Analog Input)
```
Signal Name     MCU Pin    MCU Pin#   Function             Peripheral
───────────────────────────────────────────────────────────────────────
Poti            PB_01      Pin 62     Brightness Control   ADC1_IN16
```

#### Accelerometer Interrupts
```
LIS2DW12TR_Int1 PC_13      Interrupt 1          GPIO Input (EXTI)
LIS2DW12TR_Int2 PC_15      Interrupt 2          GPIO Input (EXTI)
```

#### User LEDs (Debug)
```
USR_LED1        PB_02      User LED 1           GPIO Output
USR_LED2        PA_05      User LED 2 (Green)   GPIO Output (also onboard LED)
```

#### UART Communication
```
UART_TX         PA_02      UART Transmit        USART2_TX
UART_RX         PA_03      UART Receive         USART2_RX
```
Note: USART2 is connected to ST-Link, accessible as virtual COM port on PC

## LED Output Mapping (Shift Register Outputs)

### Register U1 (First in chain, last to receive data)
```
Output   Pin    Connected To        Signal Name
────────────────────────────────────────────────
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
Output   Pin    Connected To        Signal Name
────────────────────────────────────────────────
Q0       15     TL2 Red            TL2_Red
Q1       1      TL2 Yellow         TL2_Yellow
Q2       2      TL2 Green          TL2_Green
Q3       3      PL2 Red (3x)       PL2_Red
Q4       4      PL2 Green (3x)     PL2_Green
Q5       5      PL2 Blue (3x)      PL2_Blue
Q6       6      NOT USED           -
Q7       7      NOT USED           -
```

### Register U3 (Last in chain, first to receive data)
```
Output   Pin    Connected To        Signal Name
────────────────────────────────────────────────
Q0       15     TL3 Red            TL3_Red
Q1       1      TL3 Yellow         TL3_Yellow
Q2       2      TL3 Green          TL3_Green
Q3       3      TL4 Red            TL4_Red
Q4       4      TL4 Yellow         TL4_Yellow
Q5       5      TL4 Green          TL4_Green
Q6       6      NOT USED           -
Q7       7      NOT USED           -
```

## Implementation Tasks (Choose 4 for Full Points)

### Task 1: Single Pedestrian Crossing Control
**Focus**: Upper pedestrian crossing (PL1) only

**Requirements:**
- R1.1: Initialize with pedestrian red, car signal green
- R1.2: Button press → blue LED toggles at `toggleFreq` until crossing turns green
- R1.3: All car signals crossing the crosswalk → red after `pedestrianDelay` ms
- R1.4: Pedestrian signal stays green for `walkingDelay` ms
- R1.5: Pedestrian red when any car signal is green/orange, green otherwise
- R1.6: Car signals transition: red→orange→green or green→orange→red with `orangeDelay` ms

**Parameters:** `toggleFreq`, `pedestrianDelay`, `walkingDelay`, `orangeDelay`

### Task 2: Road Crossing Traffic Control
**Focus**: Four traffic lights, no pedestrian crossings

**Requirements:**
- R2.1: Cars can go forward or turn right only (no left turns)
- R2.2: Traffic lights prevent overlapping car paths
- R2.3: Signal transitions include orange phase (`orangeDelay` ms)
- R2.4: No active cars → allowed direction changes every `greenDelay` ms
- R2.5: Traffic light stays green if cars active in allowed direction and no cars waiting on red
- R2.6: Car at red + active cars elsewhere → wait max `redDelayMax` ms before green
- R2.7: Car at red + no active cars → immediate green transition
- R2.8: Initialize vertical lane green, horizontal lane red

**Parameters:** `orangeDelay`, `greenDelay`, `redDelayMax`

### Task 3: Complete Traffic System
**Combines Tasks 1 and 2**

**Additional Requirements:**
- R3.1: Requirements of Task 1 for each crossing (PL1 and PL2)
- R3.2: Requirements of Task 2 for car crossing
- R3.3: **Only one pedestrian crossing green at a time**
- R3.4: **Shift registers controlled with SPI interface**
- R3.5: Cars allowed to turn right when crossing on right lane is green

**Note:** This task requires SPI implementation for shift registers.

### Task 4: Display and Brightness Control
**Complements Tasks 1, 2, or 3**

**Requirements:**
- R4.1: LED brightness proportional to potentiometer voltage
- R4.2: OLED display shows waiting time bars for:
  - Remaining `pedestrianDelay` (x2, one per crossing)
  - Remaining `walkingDelay` (x2)
  - Remaining `greenDelay` (x2)
  - Remaining `redDelay` (x2)

**Display Format:** Bars that reduce in size as time counts down

**Technical Notes:**
- Use PWM on `595_Enable` (OE# pin) to control brightness
- Use SSD1306 OLED library: https://github.com/afiskon/stm32-ssd1306
- Potentiometer on PC_04 (ADC1_IN13)

## Technical Implementation Guidance

### 1. Shift Register Control

#### Method A: Bit-Banging (GPIO)
```c
// Pseudo-code for GPIO control
void shift_out_byte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        // Set data bit
        GPIO_Write(595_DS, (data >> i) & 0x01);
        
        // Clock pulse
        GPIO_Write(595_SHCP, HIGH);
        delay_us(1);
        GPIO_Write(595_SHCP, LOW);
    }
}

void update_leds(uint8_t u3_data, uint8_t u2_data, uint8_t u1_data) {
    // Send in reverse order: U3 → U2 → U1
    shift_out_byte(u3_data);
    shift_out_byte(u2_data);
    shift_out_byte(u1_data);
    
    // Latch data to outputs
    GPIO_Write(595_STCP, HIGH);
    delay_us(1);
    GPIO_Write(595_STCP, LOW);
}
```

#### Method B: SPI-Based (Recommended for Task 3)
```c
// Pseudo-code for SPI control
void update_leds_spi(uint8_t u3_data, uint8_t u2_data, uint8_t u1_data) {
    uint8_t buffer[3] = {u3_data, u2_data, u1_data};
    
    // Send data via SPI3
    HAL_SPI_Transmit(&hspi3, buffer, 3, HAL_MAX_DELAY);
    
    // Latch via GPIO
    HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, GPIO_PIN_SET);
    HAL_Delay_us(1);
    HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, GPIO_PIN_RESET);
}
```

**SPI Configuration for 74HC595:**
- Mode: SPI_MODE_0 (CPOL=0, CPHA=0)
- Bit order: MSB first
- Speed: Up to 100 MHz (use slower for reliability, e.g., 1-10 MHz)
- Data size: 8 bits

### 2. PWM for Brightness Control

To control LED brightness via potentiometer:

```c
// Configure TIM2 for PWM on OE# pin (595_Enable, PB_04)
// PB_04 can use TIM3_CH1

void setup_brightness_pwm() {
    // Initialize TIM3 Channel 1 for PWM
    // Frequency: ~1 kHz (above flicker threshold)
    // Duty cycle: controlled by potentiometer ADC reading
}

void update_brightness(uint16_t adc_value) {
    // ADC range: 0-4095 (12-bit)
    // PWM duty: 0-100%
    // Note: OE# is active LOW, so invert logic
    
    uint16_t duty = (adc_value * 100) / 4095;
    duty = 100 - duty;  // Invert for active-low OE#
    
    TIM_SetDutyCycle(TIM3, TIM_CHANNEL_1, duty);
}
```

### 3. Timing Considerations

**Orange Phase Timing:**
The orange/yellow light creates a transition period:
```
Red → Orange (orangeDelay ms) → Green
Green → Orange (orangeDelay ms) → Red
```

**Example State Machine:**
```c
typedef enum {
    STATE_RED,
    STATE_RED_TO_GREEN,  // Orange phase
    STATE_GREEN,
    STATE_GREEN_TO_RED   // Orange phase
} TrafficLightState;
```

### 4. Common Pitfalls and Solutions

**Problem 1: LEDs not lighting correctly**
- Check data ordering (reverse order for daisy chain)
- Verify current limits (max 35mA per pin, 70mA total)
- Ensure OE# is LOW to enable outputs
- Check MR# is HIGH (reset is active LOW)

**Problem 2: SPI not working with shift registers**
- Remember STCP is **not** on SPI - must use GPIO
- Set correct SPI mode (MODE_0)
- Use appropriate SPI speed (start slow, 1 MHz)

**Problem 3: PL2 LEDs too dim or not working**
- PL2 has 3x LEDs per color in parallel
- Triple current draw - check 70mA chip limit
- May need to reduce other LED currents

**Problem 4: Pedestrian crossing not responding**
- Debounce button inputs (hardware or software)
- Use interrupts or polling for button detection
- Ensure pull-up/pull-down resistors configured

### 5. Development Strategy

**Incremental Development Approach:**

1. **Week 1**: Basic GPIO and shift register control
   - Get shift registers working (bit-banging)
   - Control single traffic light
   - Test LED output mapping

2. **Week 2**: Implement chosen task logic
   - State machines for traffic control
   - Timing and delays
   - Switch input handling

3. **Week 3**: Additional features
   - SPI implementation (if doing Task 3)
   - OLED display (if doing Task 4)
   - PWM brightness (if doing Task 4)

4. **Week 4**: Integration and testing
   - Combine all features
   - Debug timing issues
   - Edge case testing

5. **Week 5**: Documentation and polish
   - Code cleanup
   - Written report
   - Final testing

## Project Requirements

### Deliverables
- **Source Code**: Due December 19, 2025
- **Written Report**: Due December 19, 2025
- **No live demo required** - submissions tested offline

### Lab Sessions
- 3 lab sessions × 4 hours each
- Use for questions and direct feedback
- Book time with course staff

### Grading
- Must implement **4 implementation tasks** for full complexity points
- Each task has specific requirements (R1.x, R2.x, etc.)
- Modular design recommended for easier integration

## Debugging Tips

### Serial Output for Debugging
Use USART2 (connected to ST-Link):
```c
// Send debug messages to PC via USB virtual COM port
printf("Traffic Light State: %d\n", current_state);
printf("Car detected: TL1=%d, TL2=%d, TL3=%d, TL4=%d\n", 
       tl1_car, tl2_car, tl3_car, tl4_car);
```

### User LEDs
Two user LEDs available for status indication:
- USR_LED1 (PB_02): Custom indicator
- USR_LED2 (PA_05): Onboard green LED

### Oscilloscope/Logic Analyzer
For debugging SPI/shift register communication:
- Monitor SHCP (clock) signal
- Monitor DS (data) signal
- Monitor STCP (latch) signal
- Verify timing relationships

## Reference Documents

Located in project directory:
- `STM32L476xx.pdf` - MCU datasheet
- `UM1724.pdf` - Nucleo-64 board user manual
- `UM2553.pdf` - STM32CubeIDE user manual
- `en_DM00157440.pdf` - STM32L4x5/L4x6 reference manual (RM0351)
- `74HC_HCT595.pdf` - Shift register datasheet
- `IS1300_TrafficLight_Schematics.pdf` - Shield schematics
- `SSD1306.pdf` - OLED display controller datasheet
- `lis2dw12.pdf` - Accelerometer datasheet
- `Sensirion_Datasheet_Humidity_Sensor_SHT20.pdf` - Temp/humidity sensor

## Code Organization Recommendations

```
Project/
├── Core/
│   ├── Src/
│   │   ├── main.c                  # Main application
│   │   ├── shift_register.c        # 74HC595 control
│   │   ├── traffic_light.c         # Traffic light logic
│   │   ├── pedestrian_crossing.c   # Pedestrian crossing logic
│   │   ├── state_machine.c         # State machine implementation
│   │   └── timing.c                # Timing and delays
│   └── Inc/
│       ├── shift_register.h
│       ├── traffic_light.h
│       ├── pedestrian_crossing.h
│       ├── state_machine.h
│       └── timing.h
├── Drivers/
│   └── SSD1306/                    # OLED display library
└── CLAUDE.MD                       # This file
```

## Key Takeaways for Claude Code

1. **This is a real-time embedded system** - timing is critical
2. **Current limits matter** - especially for PL2 with triple LEDs
3. **Data ordering is reversed** - send U3 first, U1 last
4. **Hybrid SPI approach** - STCP is GPIO, not pure SPI
5. **Only 6 of 8 outputs used** per shift register
6. **Modular design is essential** - makes integration easier
7. **Test incrementally** - don't try to implement everything at once
8. **Use HAL libraries** - STM32 HAL makes peripheral configuration easier

## Quick Reference: Common Operations

### Initialize System
1. Configure GPIO pins (inputs for switches, outputs for shift register control)
2. Initialize SPI3 (if using SPI method)
3. Initialize TIM3 for PWM (if brightness control needed)
4. Initialize ADC1 for potentiometer
5. Initialize I2C1 for sensors
6. Initialize USART2 for debugging
7. Reset shift registers (MR# LOW then HIGH)
8. Enable outputs (OE# LOW)

### Update Single Traffic Light
```c
// Example: Set TL1 to Red
uint8_t u1 = 0b00000001;  // Q0 = TL1_Red
uint8_t u2 = 0b00000000;
uint8_t u3 = 0b00000000;
update_leds_spi(u3, u2, u1);
```

### Read Car Detection
```c
bool car_at_tl1 = HAL_GPIO_ReadPin(TL1_Car_PORT, TL1_Car_PIN);
```

### Read Pedestrian Button
```c
// With debouncing
static uint32_t last_press = 0;
bool button_pressed = false;

if (HAL_GPIO_ReadPin(PL1_Switch_PORT, PL1_Switch_PIN)) {
    if (HAL_GetTick() - last_press > 200) {  // 200ms debounce
        button_pressed = true;
        last_press = HAL_GetTick();
    }
}
```

---

**Remember**: This project is about understanding embedded systems fundamentals - state machines, timing, peripheral control, and hardware interfacing. Take time to understand each component before integrating everything together.

**Good luck with your project!** 🚦

---

# PROJECT STATUS - Session 2025-11-16

## ✅ COMPLETED

### 1. Pin Mapping Verification
**ALL pins manually verified from hardware schematics:**

```
SHIFT REGISTERS (GPIO bit-banging approach):
  595_DS      → PB_05 (Pin 67) - GPIO_Output
  595_SHCP    → PC_10 (Pin 1)  - GPIO_Output
  595_STCP    → PB_12 (Pin 54) - GPIO_Output
  595_Enable  → PC_07 (Pin 57) - TIM3_CH2 (PWM)
  595_Reset   → PA_09 (Pin 59) - GPIO_Output (initialized HIGH)

CAR SWITCHES (All GPIO_Input with pull-down):
  TL1_Car     → PC_04 (Pin 72)
  TL2_Car     → PB_13 (Pin 68)
  TL3_Car     → PB_14 (Pin 66)
  TL4_Car     → PA_10 (Pin 71)

PEDESTRIAN BUTTONS (All GPIO_Input with pull-down):
  PL1_Switch  → PA_15 (Pin 17)
  PL2_Switch  → PB_07 (Pin 21)

POTENTIOMETER (ADC):
  Poti        → PB_01 (Pin 62) - ADC1_IN16
```

**IMPORTANT CORRECTION from original documentation:**
- Poti is on **PB_01 (ADC1_IN16)**, NOT PC_04
- This means **NO conflict** between potentiometer and TL1_Car
- All 4 car switches AND potentiometer can be used simultaneously

### 2. STM32CubeIDE Project Configuration

**Project Name:** `TrafficLightShield`
**Location:** `/Users/jakob/dev/kth/IS1300/TrafficLightShield/`
**Board:** NUCLEO-L476RG
**System Clock:** 80 MHz

**Configured Peripherals:**
- ✅ GPIO (all pins configured with proper labels)
- ✅ ADC1 (IN16 for potentiometer, 12-bit, continuous mode, 247.5 cycle sampling)
- ✅ TIM3 (Channel 2 for PWM, 1 kHz frequency, PSC=79, ARR=999)
- ✅ USART2 (115200 baud for debugging via ST-Link)
- ❌ I2C1 (not yet configured - for future sensor work)
- ❌ SPI3 (not yet configured - for future OLED display)

**Build Status:** ✅ Compiles successfully with 0 errors, 0 warnings

### 3. Shift Register Driver Implementation

**Files Created:**
- `Core/Inc/shift_register.h` - Driver header with LED bit definitions
- `Core/Src/shift_register.c` - Driver implementation using GPIO bit-banging

**Key Functions:**
```c
void ShiftReg_Init(void);                     // Initialize and reset shift registers
void ShiftReg_Update(u1, u2, u3);             // Update all LEDs
void ShiftReg_Clear(void);                    // Turn off all LEDs
void ShiftReg_EnableOutputs(bool enable);     // Control OE# pin
void ShiftReg_Test(void);                     // Hardware test sequence
```

**LED Bit Definitions:**
- U1: `U1_TL1_RED`, `U1_TL1_YELLOW`, `U1_TL1_GREEN`, `U1_PL1_RED`, `U1_PL1_GREEN`, `U1_PL1_BLUE`
- U2: `U2_TL2_RED`, `U2_TL2_YELLOW`, `U2_TL2_GREEN`, `U2_TL3_RED`, `U2_TL3_YELLOW`, `U2_TL3_GREEN`
- U3: `U3_TL4_RED`, `U3_TL4_YELLOW`, `U3_TL4_GREEN`, `U3_PL2_RED`, `U3_PL2_GREEN`, `U3_PL2_BLUE`

### 4. Test Code in main.c

**Current Behavior:**
1. On startup: Runs `ShiftReg_Test()` - cycles through each LED individually
2. Main loop: Blinks all traffic lights red → yellow → green (1 second each)

**Code added to USER CODE sections only** - safe for CubeMX regeneration

## 📋 NEXT STEPS

### Immediate (When Hardware Arrives):
1. Connect Nucleo-L476RG to computer via USB
2. Flash the code (click Run/Debug in STM32CubeIDE)
3. Verify LED test sequence works correctly
4. Debug any hardware issues

### Implementation Tasks (Choose 4 for Full Points):

**Priority Order:**
1. **Task 3: Complete Traffic System** (counts as Tasks 1+2+3 combined)
   - Implement traffic light state machine
   - Implement pedestrian crossing logic
   - Ensure only one pedestrian crossing green at a time
   - Allow right turns when appropriate

2. **Task 4: Display and Brightness Control**
   - Configure SPI3 for OLED display (PC_10, PC_12)
   - Integrate SSD1306 library from https://github.com/afiskon/stm32-ssd1306
   - Implement ADC reading for potentiometer brightness control
   - Use TIM3_CH2 PWM to control OE# for LED dimming
   - Display countdown bars for delays

### Code Organization Recommendations:

Create these additional files:
```
Core/Inc/
  traffic_light.h       - Traffic light state machine
  pedestrian.h          - Pedestrian crossing logic
  timing.h              - Delay tracking and timing

Core/Src/
  traffic_light.c
  pedestrian.c
  timing.c
```

### Configuration Still Needed:

**For Task 4 (OLED Display):**
- Configure SPI3 in CubeMX:
  - MOSI: PC_12
  - SCK: PC_10
  - Mode: Full-Duplex Master, 8-bit, MSB first
- Additional GPIO for OLED:
  - Chip Select (need to verify pin)
  - Reset (need to verify pin)
  - D/C Select (need to verify pin)

**For Sensor Integration (Optional):**
- Configure I2C1 in CubeMX:
  - SDA: PB_09
  - SCL: PB_08
  - Speed: 400 kHz (Fast Mode)

## 🔧 DEVELOPMENT WORKFLOW

**Recommended Approach:**
1. **Pin/Peripheral Changes:** Use STM32CubeIDE (.ioc file)
2. **Code Editing:** Use VS Code (faster, better editor)
3. **Building:** Use STM32CubeIDE (hammer icon or Ctrl+B)
4. **Flashing/Debugging:** Use STM32CubeIDE (Run/Debug button)

**IMPORTANT:** Only edit code in `/* USER CODE BEGIN/END */` sections to preserve changes during code regeneration!

## 📊 PROJECT STATISTICS

- **Code Size:** 21,708 bytes (text) + 12 bytes (data) + 1,884 bytes (bss) = 23,604 bytes total
- **Flash Usage:** ~2.3% of 1MB available
- **Files Created:** 2 custom driver files + 1 modified main.c
- **Build Time:** ~1.2 seconds

## 🐛 KNOWN ISSUES / NOTES

1. **Delay_us() function in shift_register.c is approximate** - uses busy-wait loop calibrated for 80 MHz. May need adjustment based on compiler optimization.

2. **PWM Polarity:** OE# is active LOW, so:
   - Duty 100% (CCR=999) = OE# HIGH = LEDs OFF
   - Duty 0% (CCR=0) = OE# LOW = LEDs ON (max brightness)

3. **Current Implementation:** GPIO bit-banging for shift registers. Can be upgraded to SPI later if needed, but requires handling the hybrid SPI1/SPI3 issue.

4. **LED Output Mapping Note:** Check block_diagram.txt for detailed Q0-Q5 to LED connections if discrepancies arise.

## 📚 REFERENCE FILES IN PROJECT

- `CLAUDE.md` - This file (project documentation)
- `block_diagram.txt` - Detailed hardware block diagram
- `TrafficLightShield.ioc` - CubeMX configuration (hardware setup)
- Reference PDFs in parent directory (datasheets, schematics, manuals)

---

**Session End:** 2025-11-16
**Status:** Project initialized and compiling successfully. Ready for hardware testing and feature implementation.
