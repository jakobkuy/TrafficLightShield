# IS1300 Traffic Light Project - ULTIMATE CHECKLIST

**Purpose**: Complete task-by-task checklist for IS1300 project, covering setup, implementation, testing, documentation, and submission.  
**Target Grade**: Aiming for maximum points (11/11)  
**Deadline**: December 19, 2025  
**Project Name Format**: `PRO1_Jakob_Kuylenstierna` (CRITICAL!)

---

## ⚠️ CRITICAL SUCCESS FACTORS

- [ ] **Project MUST be named** `PRO1_Jakob_Kuylenstierna` (automated grading will fail otherwise)
- [ ] **Minimum 1 point in EVERY category** or project fails (even with high total)
- [ ] **Both testing AND architecture** required for Category 1 points
- [ ] **RTOS Lab completion** mandatory (1 point minimum in Category 4)
- [ ] **Submit before Dec 19, 2025** for +1 bonus point
- [ ] **Use SPI for shift registers in Task 3** (GPIO bit-banging fails Task 3 requirement)

---

## 📋 PHASE 0: PROJECT SETUP & ENVIRONMENT

### 0.1 STM32CubeIDE Project Configuration
- [ ] Create new STM32CubeIDE project for Nucleo-L476RG
- [ ] **Set project name**: `PRO1_Jakob_Kuylenstierna` (use exact Canvas name)
- [ ] Configure system clock to 80 MHz (maximum performance)
- [ ] Enable ST-Link debug interface (SWD)
- [ ] Verify project compiles with 0 errors, 0 warnings

**Why**: Proper project naming is CRITICAL for automated grading. 80 MHz provides optimal performance for timing-critical operations.

### 0.2 Version Control Setup
- [ ] Initialize Git repository in project root
- [ ] Create `.gitignore` (exclude Debug/, Release/, .settings/)
- [ ] Make initial commit with base project
- [ ] Create development branch for experimentation

**Why**: Version control allows safe experimentation, rollback capability, and demonstrates professional development practices.

### 0.3 Documentation Structure
- [ ] Verify CLAUDE.md exists and is current
- [ ] Create PROJECT_ROADMAP.md (development timeline)
- [ ] Create TESTING_LOG.md (TDD test results)
- [ ] Create ARCHITECTURE_NOTES.md (design decisions)

**Why**: Organized documentation supports report writing and demonstrates systematic planning (Category 1 requirement).

---

## 📋 PHASE 1: HARDWARE CONFIGURATION (GPIO, Peripherals)

### 1.1 GPIO Pin Configuration (via STM32CubeMX)
- [ ] **Shift Register Control Pins**:
  - [ ] PB_05 (595_DS) - GPIO_Output (initial LOW)
  - [ ] PC_10 (595_SHCP) - GPIO_Output or SPI3_SCK
  - [ ] PB_12 (595_STCP) - GPIO_Output (latch, NOT on SPI!)
  - [ ] PC_07 (595_Enable/OE#) - TIM3_CH2 for PWM
  - [ ] PA_09 (595_Reset/MR#) - GPIO_Output (initialize HIGH)
  
- [ ] **Car Detection Switches** (GPIO_Input, Pull-down):
  - [ ] PC_04 (TL1_Car)
  - [ ] PB_13 (TL2_Car)
  - [ ] PB_14 (TL3_Car)
  - [ ] PA_10 (TL4_Car)

- [ ] **Pedestrian Buttons** (GPIO_Input, Pull-down):
  - [ ] PA_15 (PL1_Switch)
  - [ ] PB_07 (PL2_Switch)

- [ ] **Debug LEDs** (GPIO_Output):
  - [ ] PB_02 (USR_LED1)
  - [ ] PA_05 (USR_LED2 - onboard green)

- [ ] Label all pins in CubeMX with exact names from CLAUDE.md

**Why**: Proper GPIO configuration ensures hardware reliability. Pull-downs prevent floating inputs. Labeling prevents wiring errors.

**Concept**: GPIO (General Purpose Input/Output) allows the MCU to read digital signals (switches) or control digital outputs (LEDs, shift registers).

### 1.2 ADC Configuration (Potentiometer for Brightness)
- [ ] Configure ADC1, Channel 16 (PB_01)
- [ ] Resolution: 12-bit (0-4095 range)
- [ ] Sampling time: 247.5 cycles (accurate reading)
- [ ] Continuous conversion mode (always updating)
- [ ] Enable ADC in main.c: `HAL_ADC_Start(&hadc1)`

**Why**: ADC reads analog voltage (0-3.3V) and converts to digital value for LED brightness control (Task 4).

**Concept**: ADC (Analog-to-Digital Converter) samples analog signals and produces digital numbers the MCU can process.

### 1.3 Timer/PWM Configuration (Brightness Control)
- [ ] Configure TIM3, Channel 2 (PC_07 - OE# pin)
- [ ] PWM frequency: ~1 kHz (PSC=79, ARR=999 at 80MHz)
- [ ] Duty cycle: 0-999 (0% = max brightness, 100% = off)
- [ ] Enable timer: `HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2)`
- [ ] Test: Verify PWM signal with oscilloscope or changing duty cycles

**Why**: PWM controls shift register OE# pin (active LOW) to vary LED brightness smoothly.

**Concept**: PWM (Pulse Width Modulation) rapidly switches a signal on/off to control average power. Duty cycle is the % of time signal is HIGH.

### 1.4 UART Configuration (Configuration Interface - Task 5)
- [ ] Configure USART2 (PA_02=TX, PA_03=RX)
- [ ] Baud rate: 115200 (standard for STM32 virtual COM)
- [ ] Data bits: 8, Stop bits: 1, Parity: None
- [ ] Enable UART: `HAL_UART_Receive_IT(&huart2, rx_buffer, 4)` for 4-byte protocol
- [ ] Test: Send/receive data via terminal (PuTTY, screen, minicom)

**Why**: UART allows runtime parameter changes without recompiling code (Task 5 requirement).

**Concept**: UART (Universal Asynchronous Receiver-Transmitter) is serial communication. Asynchronous means no shared clock - timing via agreed baud rate.

### 1.5 SPI Configuration (Shift Registers - Task 3 REQUIRED)
- [ ] Configure SPI3 (PC_10=SCLK, PC_12=MOSI for OLED)
- [ ] Mode: 0 (CPOL=0, CPHA=0) - idle LOW, sample on rising edge
- [ ] Speed: 1-10 MHz (74HC595 supports up to 25 MHz)
- [ ] MSB first (shift register expects MSB)
- [ ] Note: STCP (latch) is GPIO, NOT part of SPI
- [ ] Test: `HAL_SPI_Transmit(&hspi3, data, 3, timeout)`

**Why**: Task 3 REQUIRES SPI for shift register control. GPIO bit-banging fails this requirement.

**Concept**: SPI (Serial Peripheral Interface) is synchronous serial communication - shared clock for precise timing. Shift registers use SPI to receive LED data.

### 1.6 I2C Configuration (Sensors - Optional)
- [ ] Configure I2C1 (PB_08=SCL, PB_09=SDA)
- [ ] Speed: 100 kHz (Standard mode) or 400 kHz (Fast mode)
- [ ] Addressing: 7-bit
- [ ] Note: Used for LIS2DW12 accelerometer, SHT20 temp/humidity
- [ ] Not required for minimum grade, but demonstrates complexity

**Why**: I2C sensors could add extra functionality or be used in RTOS task proposal.

**Concept**: I2C (Inter-Integrated Circuit) is 2-wire protocol allowing multiple devices on same bus. Each device has unique address.

### 1.7 OLED SPI Configuration (Display - Task 4)
- [ ] SPI3 already configured (shared with shift register clock)
- [ ] Configure GPIO for OLED control:
  - [ ] PA_04 (Disp_CS) - Chip Select
  - [ ] PC_14 (Disp_Reset) - Hardware reset
  - [ ] PB_00 (Disp_D/C) - Data/Command select
- [ ] Note: PC_10 shared between 595_SHCP and OLED_SCLK (acceptable)

**Why**: OLED uses SPI for fast display updates (Task 4). GPIO controls display operation mode.

### 1.8 Hardware Test & Verification
- [ ] Compile project: 0 errors, 0 warnings
- [ ] Flash to board: verify upload success
- [ ] Test each GPIO output (toggle LEDs manually)
- [ ] Test each GPIO input (read button states)
- [ ] Measure ADC values (read potentiometer)
- [ ] Verify PWM output with oscilloscope or LED
- [ ] Test UART echo (send character, receive same)
- [ ] Document test results in TESTING_LOG.md

**Why**: Early hardware validation catches wiring errors before implementing complex logic.

---

## 📋 PHASE 2: SHIFT REGISTER DRIVER (Foundation)

### 2.1 Shift Register Header File (`Core/Inc/shift_register.h`)
- [ ] Create file with include guards
- [ ] Define bit positions for each LED (U1_TL1_RED, etc.)
- [ ] Declare functions:
  - [ ] `void ShiftReg_Init(void)` - Initialize pins, clear registers
  - [ ] `void ShiftReg_Update(uint8_t u1, uint8_t u2, uint8_t u3)` - Send data
  - [ ] `void ShiftReg_Clear(void)` - Turn off all LEDs
  - [ ] `void ShiftReg_Test(void)` - Cycle through all LEDs
- [ ] Add comprehensive comments explaining purpose

**Why**: Header file provides clean interface for LED control, hiding hardware details from application code.

### 2.2 Shift Register Implementation (`Core/Src/shift_register.c`)
- [ ] **Method A: GPIO Bit-Banging** (for Tasks 1-2):
  ```c
  void shift_byte_gpio(uint8_t data) {
      for (int i = 7; i >= 0; i--) {
          HAL_GPIO_WritePin(DS_PORT, DS_PIN, (data >> i) & 0x01);
          HAL_GPIO_WritePin(SHCP_PORT, SHCP_PIN, GPIO_PIN_SET);   // Clock HIGH
          HAL_GPIO_WritePin(SHCP_PORT, SHCP_PIN, GPIO_PIN_RESET); // Clock LOW
      }
  }
  ```

- [ ] **Method B: SPI Transmission** (for Task 3 - REQUIRED):
  ```c
  void shift_register_spi(uint8_t u1, uint8_t u2, uint8_t u3) {
      uint8_t buffer[3] = {u3, u2, u1};  // U3 FIRST (daisy chain order)
      HAL_SPI_Transmit(&hspi3, buffer, 3, HAL_MAX_DELAY);
      // Latch data (STCP pulse) - still GPIO
      HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, GPIO_PIN_RESET);
  }
  ```

- [ ] Implement `ShiftReg_Init()`: Set MR# HIGH, OE# LOW, clear all registers
- [ ] Implement `ShiftReg_Clear()`: Send 0x00 to all three registers
- [ ] Implement `ShiftReg_Test()`: Cycle red→yellow→green for each traffic light

**Why**: Modular driver allows easy switching between GPIO and SPI. U3-U2-U1 order is CRITICAL due to daisy-chain topology.

**Concept - Shift Registers**: 74HC595 stores 8 bits in a chain. Data shifts through on each clock pulse. STCP (latch) transfers shift register to output register, updating LEDs simultaneously.

**Concept - Daisy Chain**: U1 output connects to U2 input, U2 output to U3 input. When sending 24 bits, first 8 go to U3, next 8 to U2, last 8 to U1.

### 2.3 LED Bit Mapping (Critical for Correctness)
- [ ] Define bit masks in header:
  ```c
  // Register U1 (TL1 + PL1)
  #define U1_TL1_RED    (1 << 0)
  #define U1_TL1_YELLOW (1 << 1)
  #define U1_TL1_GREEN  (1 << 2)
  #define U1_PL1_RED    (1 << 3)
  #define U1_PL1_GREEN  (1 << 4)
  #define U1_PL1_BLUE   (1 << 5)
  
  // Register U2 (TL2 + PL2)
  #define U2_TL2_RED    (1 << 0)
  #define U2_TL2_YELLOW (1 << 1)
  #define U2_TL2_GREEN  (1 << 2)
  #define U2_PL2_RED    (1 << 3)
  #define U2_PL2_GREEN  (1 << 4)
  #define U2_PL2_BLUE   (1 << 5)
  
  // Register U3 (TL3 + TL4)
  #define U3_TL3_RED    (1 << 0)
  #define U3_TL3_YELLOW (1 << 1)
  #define U3_TL3_GREEN  (1 << 2)
  #define U3_TL4_RED    (1 << 3)
  #define U3_TL4_YELLOW (1 << 4)
  #define U3_TL4_GREEN  (1 << 5)
  ```

- [ ] Test each LED individually: Turn on ONLY one LED at a time
- [ ] Verify correct mapping matches physical hardware
- [ ] Document any discrepancies in ARCHITECTURE_NOTES.md

**Why**: Incorrect bit mapping causes wrong LEDs to light. Individual testing isolates hardware vs software issues.

### 2.4 Shift Register Testing
- [ ] **Test 1**: Turn on each LED individually (24 tests total)
- [ ] **Test 2**: Turn on all red LEDs simultaneously
- [ ] **Test 3**: Cycle traffic light colors (red→yellow→green→red)
- [ ] **Test 4**: Verify GPIO and SPI methods produce identical results
- [ ] **Test 5**: Check current draw with multimeter (<70mA per register)
- [ ] Document all test results with photos/videos
- [ ] Create test function callable from main() for debugging

**Why**: Comprehensive testing catches hardware issues early. SPI vs GPIO equivalence proves both implementations work.

---

## 📋 PHASE 3: TEST DRIVEN DEVELOPMENT (TDD) SETUP

### 3.1 Understanding TDD Methodology
- [ ] Read about TDD cycle: **Red → Green → Refactor**
  - **Red**: Write test that fails (feature doesn't exist yet)
  - **Green**: Write minimal code to make test pass
  - **Refactor**: Improve code quality without changing behavior
- [ ] Understand why TDD is required: Demonstrates planning, catches bugs early, documents expected behavior
- [ ] Review example from PingPong lab

**Why**: TDD forces thinking about requirements BEFORE coding. Tests serve as executable specifications.

**Concept**: TDD inverts normal development - requirements → tests → implementation (not implementation → tests).

### 3.2 Test Infrastructure Setup
- [ ] Create `Core/Inc/tests.h`:
  ```c
  #ifndef TESTS_H
  #define TESTS_H
  
  #include <stdint.h>
  #include <stdbool.h>
  
  // Test execution
  void RunAllTests(void);
  
  // Test categories
  bool Test_ShiftRegister_Basic(void);
  bool Test_ShiftRegister_AllLEDs(void);
  bool Test_TrafficLight_Initialization(void);
  bool Test_TrafficLight_RedToGreen(void);
  bool Test_Pedestrian_ButtonPress(void);
  bool Test_Pedestrian_BlueToggle(void);
  bool Test_EdgeCase_AllCars(void);
  bool Test_EdgeCase_NoCars(void);
  bool Test_Integration_FullCycle(void);
  
  #endif
  ```

- [ ] Create `Core/Src/tests.c` with test runner:
  ```c
  void RunAllTests(void) {
      uint8_t passed = 0, failed = 0;
      
      printf("\n=== STARTING TEST SUITE ===\n");
      
      if (Test_ShiftRegister_Basic()) passed++; else failed++;
      if (Test_TrafficLight_Initialization()) passed++; else failed++;
      // ... more tests
      
      printf("\n=== TEST SUMMARY ===\n");
      printf("Passed: %d\n", passed);
      printf("Failed: %d\n", failed);
      printf("Total:  %d\n", passed + failed);
  }
  ```

- [ ] Add `#define RUN_TESTS` to main.c for conditional test execution
- [ ] Implement UART printf for test output (retarget printf to UART)

**Why**: Centralized test runner makes it easy to execute all tests. Conditional compilation allows tests in debug, disabled in production.

### 3.3 Test Categories Planning
- [ ] **Basic Functionality Tests** (happy path):
  - [ ] Single LED turns on/off
  - [ ] Traffic light completes full cycle (red→yellow→green)
  - [ ] Pedestrian button triggers crossing sequence
  - [ ] All timing parameters respected

- [ ] **Corner Case Tests** (edge conditions):
  - [ ] All car switches pressed simultaneously
  - [ ] No car switches pressed (default behavior)
  - [ ] Both pedestrian buttons pressed at once
  - [ ] Button spam (rapid repeated presses)
  - [ ] Pedestrian request during orange phase

- [ ] **Boundary Tests** (limits):
  - [ ] Minimum delay values (1 ms, 1 Hz)
  - [ ] Maximum delay values (30000 ms, 10 Hz)
  - [ ] Invalid parameter rejection (UART protocol)

- [ ] **Integration Tests** (system behavior):
  - [ ] Complete intersection cycle with cars and pedestrians
  - [ ] Verify no conflicting green lights
  - [ ] Pedestrian crossing stops correct cars
  - [ ] Right turn allowed when pedestrian crossing green

**Why**: Different test categories ensure comprehensive coverage. Corner cases catch bugs normal testing misses.

### 3.4 Writing Testable Code
- [ ] Use state machines (makes states testable)
- [ ] Separate logic from hardware (dependency injection)
- [ ] Create getter functions for internal state:
  ```c
  TrafficLightState GetTrafficLightState(uint8_t light_id);
  bool IsPedestrianCrossingGreen(uint8_t crossing_id);
  uint32_t GetRemainingDelay(DelayType type);
  ```

- [ ] Mock hardware inputs for testing:
  ```c
  void SetMockCarPresent(uint8_t light_id, bool present);
  void SetMockButtonPressed(uint8_t crossing_id, bool pressed);
  ```

**Why**: Testable code separates concerns. Getter functions allow verifying internal state. Mocking enables testing without physical hardware.

### 3.5 Example TDD Workflow
- [ ] **Step 1 (RED)**: Write test for traffic light initialization
  ```c
  bool Test_TrafficLight_Initialization(void) {
      TrafficLight_Init();
      
      // Vertical (TL1, TL3) should start green
      assert(GetTrafficLightState(TL1) == STATE_GREEN);
      assert(GetTrafficLightState(TL3) == STATE_GREEN);
      
      // Horizontal (TL2, TL4) should start red
      assert(GetTrafficLightState(TL2) == STATE_RED);
      assert(GetTrafficLightState(TL4) == STATE_RED);
      
      return true;  // All assertions passed
  }
  ```
  - Run test → FAIL (TrafficLight_Init() doesn't exist)

- [ ] **Step 2 (GREEN)**: Implement minimal code to pass test
  ```c
  void TrafficLight_Init(void) {
      trafficLights[TL1].state = STATE_GREEN;
      trafficLights[TL3].state = STATE_GREEN;
      trafficLights[TL2].state = STATE_RED;
      trafficLights[TL4].state = STATE_RED;
  }
  ```
  - Run test → PASS

- [ ] **Step 3 (REFACTOR)**: Improve code quality
  ```c
  void TrafficLight_Init(void) {
      SetVerticalLights(STATE_GREEN);
      SetHorizontalLights(STATE_RED);
  }
  ```
  - Run test → PASS (behavior unchanged)

- [ ] Repeat for next requirement

**Why**: TDD ensures every feature has tests. Refactoring with tests provides safety net.

### 3.6 Test Documentation
- [ ] Create TESTING_LOG.md with table:
  ```markdown
  | Test Name | Category | Status | Date | Notes |
  |-----------|----------|--------|------|-------|
  | Test_ShiftRegister_Basic | Basic | PASS | 2025-11-20 | All LEDs working |
  | Test_EdgeCase_AllCars | Corner | FAIL | 2025-11-20 | Priority issue |
  ```

- [ ] Document test failures and fixes
- [ ] Include test output logs (copy from UART)
- [ ] Add screenshots of physical hardware behavior
- [ ] Summary statistics (X passed, Y failed, Z total)

**Why**: Test documentation demonstrates systematic approach (Category 1 requirement). Report needs test results table.

---

## 📋 PHASE 4: ARCHITECTURE & DESIGN

### 4.1 State Machine Design (Core Logic)
- [ ] **Traffic Light State Machine**:
  - States: RED, RED_ORANGE, GREEN, ORANGE
  - Transitions: Timed (orange/green delays) or car-triggered
  - Inputs: Car switches, timing, pedestrian state
  - Outputs: LED states, allow/block directions
  
- [ ] **Pedestrian Crossing State Machine**:
  - States: IDLE, WAITING (blue blinking), CROSSING (green)
  - Transitions: Button press, timer completion
  - Inputs: Button, timer, traffic light state
  - Outputs: Pedestrian LEDs, blue blink request

- [ ] Draw state diagrams (one per state machine)
- [ ] Label all states, transitions, guards, and actions
- [ ] Document state diagrams in report with explanations

**Why**: State machines provide clear, testable logic structure. Diagrams communicate design to others and support Category 1 points.

**Concept**: State machine has discrete states, transitions between states based on conditions, actions on entry/exit/transition.

### 4.2 Module Architecture
- [ ] Design module hierarchy:
  ```
  main.c
    â"œâ"€â"€ shift_register.c  (hardware abstraction)
    â"œâ"€â"€ traffic_light.c   (state machine + logic)
    â"œâ"€â"€ pedestrian.c      (state machine + logic)
    â"œâ"€â"€ timing.c          (delay tracking)
    â"œâ"€â"€ uart_protocol.c   (Task 5 - parameter updates)
    â"œâ"€â"€ oled_display.c    (Task 4 - countdown bars)
    â""â"€â"€ tests.c           (TDD test suite)
  ```

- [ ] Define clear interfaces between modules (function APIs)
- [ ] Document module responsibilities in ARCHITECTURE_NOTES.md
- [ ] Create module dependency diagram

**Why**: Modular design enables parallel development, testing, and maintenance. Clear interfaces prevent coupling.

### 4.3 Timing Architecture
- [ ] Use SysTick (1 ms tick) or FreeRTOS ticks for timing
- [ ] Track multiple simultaneous timers:
  - Orange phase timers (per traffic light)
  - Green phase timer (intersection-wide)
  - Red delay timer (per traffic light)
  - Pedestrian delays (per crossing)
  - Blue LED toggle timer (per crossing)

- [ ] Implement non-blocking timing (no `HAL_Delay()` in main loop)
- [ ] Create timing module:
  ```c
  typedef struct {
      uint32_t start_tick;
      uint32_t duration_ms;
      bool active;
  } Timer;
  
  void Timer_Start(Timer *timer, uint32_t duration_ms);
  bool Timer_IsExpired(Timer *timer);
  uint32_t Timer_GetRemaining(Timer *timer);
  ```

**Why**: Non-blocking timing allows multiple concurrent operations. Timer abstraction simplifies state machine logic.

**Concept**: Blocking delays (`HAL_Delay()`) freeze entire system. Non-blocking timing checks elapsed time each loop iteration.

### 4.4 Data Flow Design
- [ ] Input flow: Switches → Debounce → State machines → LED states
- [ ] Output flow: LED states → Shift register bytes → SPI/GPIO → Hardware
- [ ] Create sequence diagram showing typical pedestrian request:
  1. Button press detected
  2. Blue LED starts blinking
  3. Timer expires (pedestrianDelay)
  4. Traffic lights transition to red (via orange)
  5. Pedestrian light turns green
  6. Pedestrian crosses (walkingDelay)
  7. Pedestrian light turns red
  8. Traffic resumes

**Why**: Sequence diagrams show time-based interactions between components (Category 1 architecture requirement).

### 4.5 Hardware Architecture Documentation
- [ ] Create system block diagram:
  - MCU (STM32L476RG)
  - Shift registers (3× 74HC595D in daisy chain)
  - Traffic lights (TL1-TL4, each with R/Y/G LEDs)
  - Pedestrian crossings (PL1-PL2, each with R/G/B LEDs)
  - Sensors (car switches, pedestrian buttons, potentiometer)
  - Peripherals (OLED, UART, I2C sensors)

- [ ] Create shift register chain diagram showing data flow:
  - MCU → U3 (TL3/TL4) → U2 (TL2/PL2) → U1 (TL1/PL1)
  - Label pin connections (DS, SHCP, STCP, Q7S)

- [ ] Create pin connection table (already in CLAUDE.md)

**Why**: Hardware diagrams demonstrate understanding of system (Category 1 requirement). Essential for report.

### 4.6 Architecture Review Checklist
- [ ] All diagrams use consistent notation (UML or similar)
- [ ] Every diagram has title, legend, and explanation
- [ ] Module interfaces clearly defined
- [ ] No circular dependencies between modules
- [ ] State machines have no unreachable states
- [ ] Timing constraints documented
- [ ] Hardware current limits checked (70mA per shift register)

---

## 📋 PHASE 5: IMPLEMENTATION - TASK 1 (Single Pedestrian Crossing)

### 5.1 Task 1 Requirements Analysis
- [ ] Read and understand all R1.1-R1.6 requirements
- [ ] Identify key behaviors:
  - Init: Cars green, pedestrian red
  - Button → blue blink @ toggleFreq
  - After pedestrianDelay → cars red (via orange)
  - Ped green for walkingDelay
  - Ped red when cars green/orange
  - Car transitions always include orange phase

- [ ] List parameters: toggleFreq, pedestrianDelay, walkingDelay, orangeDelay

**Why**: Clear requirements understanding prevents implementation errors. Task 1 is REQUIRED for any points in Category 2.

### 5.2 TDD: Write Tests FIRST (Task 1)
- [ ] Test: Initialization (ped red, TL2/TL4 green)
- [ ] Test: Button press starts blue blinking
- [ ] Test: Blue stops blinking when pedestrian green
- [ ] Test: pedestrianDelay timing accuracy
- [ ] Test: Cars transition through orange
- [ ] Test: walkingDelay timing accuracy
- [ ] Test: Pedestrian red when cars green/orange
- [ ] Run all tests → All FAIL (nothing implemented yet)

**Why**: TDD requirement (Category 1). Tests define expected behavior before coding.

### 5.3 Pedestrian Module Implementation (`pedestrian.c/h`)
- [ ] Define pedestrian state enum:
  ```c
  typedef enum {
      PED_IDLE,          // Red, waiting for button
      PED_WAITING,       // Red, blue blinking, timer running
      PED_CROSSING       // Green, walking
  } PedestrianState;
  ```

- [ ] Implement state machine:
  ```c
  void Pedestrian_Update(PedestrianCrossing *ped, uint32_t current_tick) {
      switch (ped->state) {
          case PED_IDLE:
              if (ButtonPressed(ped->button_pin)) {
                  ped->state = PED_WAITING;
                  Timer_Start(&ped->request_timer, pedestrianDelay);
                  ped->blue_active = true;
              }
              break;
          
          case PED_WAITING:
              // Toggle blue LED at toggleFreq
              if (Timer_IsExpired(&ped->blue_timer)) {
                  ToggleBlueLED(ped->id);
                  Timer_Start(&ped->blue_timer, 500 / toggleFreq);
              }
              
              // Check if cars are ready (all red)
              if (Timer_IsExpired(&ped->request_timer) && AllCarsRed(ped->crossing_cars)) {
                  ped->state = PED_CROSSING;
                  ped->blue_active = false;
                  Timer_Start(&ped->walking_timer, walkingDelay);
              }
              break;
          
          case PED_CROSSING:
              if (Timer_IsExpired(&ped->walking_timer)) {
                  ped->state = PED_IDLE;
              }
              break;
      }
  }
  ```

- [ ] Implement helper functions:
  - `bool ButtonPressed(GPIO_Pin)` with debouncing
  - `bool AllCarsRed(car_list)` checks traffic light states
  - `void UpdatePedestrianLEDs(ped)` sets R/G/B based on state

**Why**: State machine provides clear logic. Helper functions simplify main logic and enable testing.

### 5.4 Traffic Light Module (Simplified for Task 1)
- [ ] Implement basic state machine for TL2 and TL4 (horizontal lights):
  ```c
  typedef enum {
      TL_GREEN,
      TL_ORANGE,
      TL_RED,
      TL_RED_ORANGE  // Transition from red to green
  } TrafficLightState;
  ```

- [ ] Implement transition logic responding to pedestrian requests:
  - Green → Orange (when pedestrian waiting + delay expired)
  - Orange → Red (after orangeDelay)
  - Red → Red-Orange → Green (after pedestrian done)

- [ ] Coordinate with pedestrian module:
  - Pedestrian can only go green when TL2 and TL4 are red
  - Cars can't turn green while pedestrian is green

**Why**: Traffic lights must coordinate with pedestrians to meet safety requirements (R1.5).

### 5.5 Main Loop Integration (Task 1)
- [ ] Initialize all modules in main():
  ```c
  int main(void) {
      HAL_Init();
      SystemClock_Config();
      MX_GPIO_Init();
      // ... other peripheral inits
      
      ShiftReg_Init();
      Pedestrian_Init(&pl1, PL1_BUTTON_PIN, PL1_ID);
      TrafficLight_Init(&tl2, TL2_CAR_PIN, TL2_ID);
      TrafficLight_Init(&tl4, TL4_CAR_PIN, TL4_ID);
      
      while (1) {
          uint32_t tick = HAL_GetTick();
          
          Pedestrian_Update(&pl1, tick);
          TrafficLight_Update(&tl2, tick);
          TrafficLight_Update(&tl4, tick);
          
          UpdateAllLEDs();  // Convert states to shift register bytes
      }
  }
  ```

- [ ] Implement `UpdateAllLEDs()` to combine all states into 3 register bytes

**Why**: Main loop orchestrates all modules. Non-blocking updates allow concurrent operations.

### 5.6 Task 1 Testing & Verification
- [ ] Run TDD tests → All should PASS now
- [ ] Physical hardware test:
  - [ ] Press button → blue starts blinking
  - [ ] After pedestrianDelay → cars turn orange then red
  - [ ] Pedestrian green for walkingDelay
  - [ ] Pedestrian red when cars green
- [ ] Test parameter variations:
  - [ ] toggleFreq: 1 Hz, 5 Hz, 10 Hz
  - [ ] pedestrianDelay: 1s, 3s, 10s
  - [ ] walkingDelay: 1s, 5s, 15s
  - [ ] orangeDelay: 0.5s, 1s, 3s
- [ ] Record test results with video/photos
- [ ] Update TESTING_LOG.md with results

**Why**: Physical testing validates logic works on real hardware. Parameter testing ensures flexibility.

### 5.7 Task 1 Completion Checklist
- [ ] All R1.1-R1.6 requirements met
- [ ] TDD tests passing
- [ ] Physical hardware validated
- [ ] Code commented with function headers
- [ ] Git commit: "Complete Task 1 - Single Pedestrian Crossing"

---

## 📋 PHASE 6: IMPLEMENTATION - TASK 2 (Traffic Control)

### 6.1 Task 2 Requirements Analysis
- [ ] Read R2.1-R2.8 requirements carefully
- [ ] Understand traffic flow rules:
  - TL1 (North): Can go straight (South) or right (West)
  - TL2 (East): Can go straight (West) or right (North)
  - TL3 (South): Can go straight (North) or right (East)
  - TL4 (West): Can go straight (East) or right (South)
  - NO left turns (would cross opposing traffic)

- [ ] Identify conflicting paths (can't both be green):
  - North-South (TL1/TL3) conflicts with East-West (TL2/TL4)
  - Right turns generally OK if no conflicting pedestrians

- [ ] List parameters: orangeDelay, greenDelay, redDelayMax

**Why**: Understanding traffic flow prevents safety violations (overlapping green lights).

### 6.2 TDD: Write Tests FIRST (Task 2)
- [ ] Test: Initialization (TL1/TL3 green, TL2/TL4 red)
- [ ] Test: No conflicting green lights
- [ ] Test: All transitions include orange phase
- [ ] Test: Green changes after greenDelay with no cars
- [ ] Test: Green stays if cars present in same direction
- [ ] Test: Red light with waiting car triggers after redDelayMax
- [ ] Test: Red light with no other cars immediately transitions
- [ ] Test: Car at red waits if cars active elsewhere
- [ ] Run all tests → FAIL (Task 2 not implemented)

**Why**: Tests ensure all R2.x requirements covered.

### 6.3 Traffic Direction Management
- [ ] Define direction enum:
  ```c
  typedef enum {
      DIR_NONE,
      DIR_NORTH_SOUTH,  // TL1 and TL3 green
      DIR_EAST_WEST     // TL2 and TL4 green
  } TrafficDirection;
  ```

- [ ] Implement direction change logic:
  ```c
  void TrafficControl_Update(uint32_t tick) {
      bool cars_NS = CarPresent(TL1) || CarPresent(TL3);
      bool cars_EW = CarPresent(TL2) || CarPresent(TL4);
      
      switch (current_direction) {
          case DIR_NORTH_SOUTH:
              // R2.5: Stay green if cars active and no waiting cars
              if (cars_NS && !cars_EW) {
                  // Keep green
              }
              // R2.4: Change after greenDelay if no cars
              else if (Timer_IsExpired(&green_timer)) {
                  RequestDirectionChange(DIR_EAST_WEST);
              }
              // R2.6: Change if cars waiting at red
              else if (cars_EW && Timer_IsExpired(&red_timer)) {
                  RequestDirectionChange(DIR_EAST_WEST);
              }
              break;
          
          case DIR_EAST_WEST:
              // Similar logic for east-west direction
              break;
      }
  }
  ```

- [ ] Implement direction change with orange phase:
  ```c
  void RequestDirectionChange(TrafficDirection new_dir) {
      // R2.3: All transitions include orange phase
      SetAllGreenToOrange();
      Timer_Start(&orange_timer, orangeDelay);
      pending_direction = new_dir;
  }
  
  void CompleteDirectionChange(void) {
      if (Timer_IsExpired(&orange_timer)) {
          SetAllOrangeToRed();
          Timer_Start(&red_orange_timer, orangeDelay);
          
          if (Timer_IsExpired(&red_orange_timer)) {
              // New direction goes red-orange then green
              SetDirectionToGreen(pending_direction);
              current_direction = pending_direction;
              Timer_Start(&green_timer, greenDelay);
          }
      }
  }
  ```

**Why**: Centralized direction management ensures no conflicting green lights (R2.2).

### 6.4 Car Detection with Debouncing
- [ ] Implement input debouncing:
  ```c
  bool CarPresent(uint8_t light_id) {
      GPIO_PinState current = HAL_GPIO_ReadPin(car_pins[light_id].port, 
                                                 car_pins[light_id].pin);
      
      if (current != car_last_state[light_id]) {
          car_debounce_timer[light_id] = HAL_GetTick();
          car_last_state[light_id] = current;
      }
      
      // Only consider stable after DEBOUNCE_TIME (e.g., 50ms)
      if ((HAL_GetTick() - car_debounce_timer[light_id]) > DEBOUNCE_TIME) {
          return (current == GPIO_PIN_SET);
      }
      
      return car_stable_state[light_id];  // Return last stable state
  }
  ```

**Why**: Mechanical switches bounce (rapid on/off transitions). Debouncing prevents false detections.

**Concept**: Debouncing waits for input to be stable for minimum time before considering it valid.

### 6.5 Red Delay and Priority Logic
- [ ] Implement R2.6 (max wait at red):
  ```c
  void UpdateRedDelay(uint8_t light_id) {
      if (TrafficLightIsRed(light_id) && CarPresent(light_id)) {
          if (!red_delay_active[light_id]) {
              Timer_Start(&red_delay_timer[light_id], redDelayMax);
              red_delay_active[light_id] = true;
          }
          
          if (Timer_IsExpired(&red_delay_timer[light_id])) {
              // Force direction change (priority)
              RequestDirectionChange(GetDirectionForLight(light_id));
          }
      } else {
          red_delay_active[light_id] = false;
      }
  }
  ```

- [ ] Implement R2.7 (immediate green if no active cars):
  ```c
  if (CarPresent(red_light) && NoActiveOtherCars()) {
      RequestDirectionChange(GetDirectionForLight(red_light));
  }
  ```

**Why**: R2.6 ensures waiting cars don't wait forever. R2.7 optimizes traffic flow when intersection is mostly empty.

### 6.6 Task 2 Testing & Verification
- [ ] Run TDD tests → All should PASS
- [ ] Physical hardware test scenarios:
  - [ ] **Scenario 1**: No cars → direction changes every greenDelay
  - [ ] **Scenario 2**: Car on TL1, green stays while car present
  - [ ] **Scenario 3**: Car on TL1 (green), car arrives at TL2 (red), TL2 waits max redDelayMax
  - [ ] **Scenario 4**: Car on TL2 (red), no cars elsewhere, immediate green
  - [ ] **Scenario 5**: Cars on TL1 and TL3 (both green), verify no TL2/TL4 green
  - [ ] **Scenario 6**: All transitions include orange phase
- [ ] Record test videos for report
- [ ] Update TESTING_LOG.md

**Why**: Scenario testing validates complex timing interactions.

### 6.7 Task 2 Completion Checklist
- [ ] All R2.1-R2.8 requirements met
- [ ] TDD tests passing
- [ ] Physical hardware validated
- [ ] No conflicting green lights possible
- [ ] Git commit: "Complete Task 2 - Traffic Control"

---

## 📋 PHASE 7: IMPLEMENTATION - TASK 3 (Complete System)

### 7.1 Task 3 Requirements Analysis
- [ ] R3.1: Task 1 requirements for BOTH pedestrian crossings (PL1 and PL2)
- [ ] R3.2: Task 2 requirements for ALL four traffic lights
- [ ] R3.3: **CRITICAL** - Only ONE pedestrian crossing green at a time
- [ ] R3.4: **CRITICAL** - Shift registers MUST use SPI (not GPIO)
- [ ] R3.5: Right turns allowed when crossing on right lane is green

**Why**: Task 3 combines everything. R3.3 prevents pedestrian collisions. R3.4 is strict grading requirement.

### 7.2 SPI Migration (R3.4 - REQUIRED)
- [ ] Verify SPI3 configured in CubeMX
- [ ] Update shift_register.c to use SPI instead of GPIO:
  ```c
  void ShiftReg_Update(uint8_t u1, uint8_t u2, uint8_t u3) {
      uint8_t buffer[3] = {u3, u2, u1};  // U3 sent first
      HAL_SPI_Transmit(&hspi3, buffer, 3, HAL_MAX_DELAY);
      
      // Latch still uses GPIO (STCP not on SPI)
      HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, GPIO_PIN_SET);
      HAL_Delay(1);  // Brief delay for setup time
      HAL_GPIO_WritePin(STCP_PORT, STCP_PIN, GPIO_PIN_RESET);
  }
  ```

- [ ] Test SPI transmission with oscilloscope:
  - [ ] Verify clock frequency (1-10 MHz)
  - [ ] Verify data timing (setup/hold times met)
  - [ ] Compare GPIO vs SPI output → should be identical

**Why**: Task 3 REQUIRES SPI. Grading will check for `HAL_SPI_Transmit()` usage. GPIO bit-banging fails Task 3.

### 7.3 Dual Pedestrian Crossing Coordination (R3.3)
- [ ] Modify pedestrian module to support mutual exclusion:
  ```c
  typedef struct {
      PedestrianCrossing crossings[2];  // PL1, PL2
      uint8_t active_crossing;          // Which one is green (or NONE)
  } PedestrianSystem;
  
  void PedestrianSystem_Update(PedestrianSystem *sys, uint32_t tick) {
      // R3.3: Only ONE crossing can be green
      for (int i = 0; i < 2; i++) {
          PedestrianCrossing *ped = &sys->crossings[i];
          
          // Block crossing request if other crossing active
          if (ped->state == PED_WAITING && sys->active_crossing != NONE && sys->active_crossing != i) {
              // Keep waiting, can't proceed yet
              continue;
          }
          
          // Normal update
          Pedestrian_Update(ped, tick);
          
          // Track active crossing
          if (ped->state == PED_CROSSING) {
              sys->active_crossing = i;
          } else if (sys->active_crossing == i) {
              sys->active_crossing = NONE;
          }
      }
  }
  ```

- [ ] Add priority system if both buttons pressed:
  - [ ] Option A: First-come-first-served (timestamp button presses)
  - [ ] Option B: Alternate between crossings
  - [ ] Document chosen strategy in report

**Why**: R3.3 is safety-critical. Two pedestrians crossing simultaneously could collide.

### 7.4 Right Turn Rules (R3.5)
- [ ] Implement right turn logic:
  ```c
  bool CanTurnRight(uint8_t light_id) {
      // Identify which pedestrian crossing is on the right
      uint8_t right_crossing = GetRightCrossing(light_id);
      
      // Right turn allowed if:
      // 1. This light is green OR
      // 2. Pedestrian crossing on right is green
      return (TrafficLightIsGreen(light_id) || 
              PedestrianIsGreen(right_crossing));
  }
  ```

- [ ] Update LED display to show right turn allowed:
  - [ ] Could use existing green LED
  - [ ] Or document rule in report (behavior not visually indicated)

**Why**: R3.5 allows traffic optimization - right turns don't conflict with pedestrians on the right.

### 7.5 Full System Integration
- [ ] Integrate all four traffic lights:
  ```c
  TrafficLight tl1, tl2, tl3, tl4;
  TrafficControl traffic_control;  // Manages direction changes
  ```

- [ ] Integrate both pedestrian crossings:
  ```c
  PedestrianSystem pedestrian_system;
  ```

- [ ] Coordinate traffic and pedestrians:
  ```c
  void SystemUpdate(uint32_t tick) {
      // Update traffic lights
      TrafficControl_Update(&traffic_control, tick);
      
      // Update pedestrians (may request traffic changes)
      PedestrianSystem_Update(&pedestrian_system, tick);
      
      // Apply pedestrian constraints to traffic
      if (pedestrian_system.active_crossing != NONE) {
          BlockConflictingTrafficLights(&traffic_control, 
                                        pedestrian_system.active_crossing);
      }
      
      // Update all hardware
      UpdateAllLEDs();
  }
  ```

**Why**: Centralized coordination ensures all requirements met simultaneously.

### 7.6 Task 3 Comprehensive Testing
- [ ] **Test 1**: PL1 button → only TL2/TL4 turn red
- [ ] **Test 2**: PL2 button → only TL1/TL3 turn red
- [ ] **Test 3**: Both buttons pressed → only one crossing green
- [ ] **Test 4**: Verify SPI usage (check with debugger/oscilloscope)
- [ ] **Test 5**: Right turn scenarios:
  - [ ] TL1 at red, PL1 green → TL1 can turn right onto TL4's path
  - [ ] TL2 at red, PL2 green → TL2 can turn right onto TL1's path
- [ ] **Test 6**: All 4 traffic lights cycle correctly
- [ ] **Test 7**: No conflicting green lights ever (all combinations)
- [ ] **Test 8**: Complex scenario: cars all directions + both pedestrians

- [ ] Record comprehensive test video (10+ minutes)
- [ ] Document ALL test cases in TESTING_LOG.md
- [ ] Create test summary table for report

**Why**: Task 3 is most complex - extensive testing essential. Video evidence supports report.

### 7.7 Task 3 Completion Checklist
- [ ] All R3.1-R3.5 requirements met
- [ ] SPI verified (not GPIO)
- [ ] Only one pedestrian crossing green at a time
- [ ] TDD tests passing for all scenarios
- [ ] Physical hardware validated
- [ ] Git commit: "Complete Task 3 - Full System with SPI"
- [ ] **Category 2: 3 points unlocked** (if Task 4 or 5 also completed)

---

## 📋 PHASE 8: RTOS LAB (MANDATORY - Category 4)

### 8.1 FreeRTOS Lab Completion
- [ ] Complete RTOS lab assignment (separate from project)
- [ ] Understand FreeRTOS concepts:
  - **Tasks**: Independent threads of execution
  - **Priorities**: Higher priority tasks preempt lower priority
  - **Scheduling**: Preemptive priority-based
  - **Blocking**: Tasks can wait without wasting CPU
  - **Semaphores/Mutexes**: Synchronization primitives
  - **Queues**: Inter-task communication

- [ ] Complete lab exercises:
  - [ ] Task creation and priorities
  - [ ] Timing measurements
  - [ ] Deadline analysis
  - [ ] Utilization calculations

- [ ] Document lab results in lab report
- [ ] Submit lab for grading

**Why**: RTOS lab is MANDATORY for 1 point in Category 4. Failing lab = failing project.

**Concept - RTOS**: Real-Time Operating System manages multiple tasks with timing guarantees. FreeRTOS is popular open-source RTOS for embedded systems.

### 8.2 Understanding RTOS Concepts for Project
- [ ] **Task**: Independent execution unit with own stack
  ```c
  void TrafficLightTask(void *argument) {
      for (;;) {  // Infinite loop
          // Update traffic light state
          osDelay(10);  // Yield to other tasks
      }
  }
  ```

- [ ] **Priority**: Determines which task runs when multiple are ready
  - Higher priority = runs first
  - Same priority = time-sliced (round-robin)

- [ ] **Blocking**: Task waits for event without consuming CPU
  ```c
  osDelay(1000);  // Block for 1000ms, CPU free for other tasks
  osQueueReceive(queue, &data, portMAX_DELAY);  // Block until data available
  ```

- [ ] **Preemption**: Higher priority task interrupts lower priority
  - Ensures time-critical tasks meet deadlines

**Why**: Understanding RTOS enables intelligent task design for 2-3 additional points in Category 4.

### 8.3 RTOS Integration Planning (Optional 2-3 points)
- [ ] **DECIDE**: Will you use FreeRTOS in project?
  - **Option A**: No RTOS (simple main loop) → 1 point Category 4 (lab only)
  - **Option B**: FreeRTOS integration → 2-3 points Category 4 (must propose and get approved)

- [ ] If choosing RTOS integration:
  - [ ] Create proposal document (before implementing!)
  - [ ] Define tasks and priorities
  - [ ] Identify synchronization needs
  - [ ] Schedule meeting with teacher for approval
  - [ ] Get written approval before proceeding

**Why**: RTOS proposal must be pre-approved. Quality of design determines 2 vs 3 points.

### 8.4 RTOS Integration Proposal (if pursuing extra points)
- [ ] **Proposed Task Structure** (example):
  ```
  Task                     Priority    Period    Purpose
  ────────────────────────────────────────────────────────
  TrafficLightTask         Normal      10ms      Update traffic state machines
  PedestrianTask           Normal      10ms      Update pedestrian state machines
  InputScanTask            High        20ms      Read buttons/switches with debouncing
  LEDUpdateTask            Normal      50ms      Update shift registers
  OLEDDisplayTask          Low         100ms     Update OLED countdown bars
  UARTProtocolTask         Normal      Event     Process UART parameter updates
  ```

- [ ] **Synchronization Mechanisms**:
  - Semaphore: Signal button press from ISR to InputScanTask
  - Mutex: Protect shared LED state during updates
  - Queue: Send parameter updates from UART to application tasks
  - Event Groups: Signal pedestrian requests to traffic controller

- [ ] **Justification**: Why RTOS improves design:
  - Cleaner separation of concerns (one task per module)
  - Easier timing management (osDelay instead of manual timers)
  - Priority ensures critical tasks meet deadlines
  - Blocking allows efficient CPU usage

- [ ] Submit proposal to teacher with diagrams
- [ ] Wait for approval before implementing

**Why**: Pre-approval required. Teacher evaluates if proposal demonstrates RTOS understanding.

### 8.5 RTOS Implementation (if approved)
- [ ] Enable FreeRTOS in CubeMX (CMSIS-RTOS v2 API)
- [ ] Configure heap size (increase if tasks fail to create)
- [ ] Create tasks in `MX_FREERTOS_Init()`:
  ```c
  osThreadAttr_t traffic_task_attr = {
      .name = "TrafficLight",
      .priority = osPriorityNormal,
      .stack_size = 256 * 4
  };
  osThreadNew(TrafficLightTask, NULL, &traffic_task_attr);
  ```

- [ ] Implement task functions with infinite loops
- [ ] Add synchronization primitives:
  ```c
  osSemaphoreId_t button_sem;
  button_sem = osSemaphoreNew(1, 0, NULL);
  
  // In button ISR
  osSemaphoreRelease(button_sem);
  
  // In InputScanTask
  osSemaphoreAcquire(button_sem, osWaitForever);
  ```

- [ ] Test task priorities (higher priority should preempt)
- [ ] Verify timing with oscilloscope or logic analyzer
- [ ] Document RTOS usage in report with diagrams

**Why**: RTOS implementation must demonstrate correct usage of tasks, priorities, and synchronization.

### 8.6 RTOS Testing & Validation
- [ ] Verify task creation (debugger shows all tasks)
- [ ] Test priority preemption (high priority interrupts low)
- [ ] Measure CPU utilization (should be <100%)
- [ ] Check for deadlocks (tasks should never permanently block)
- [ ] Verify timing constraints met (deadlines satisfied)
- [ ] Document test results with screenshots/traces

**Why**: RTOS correctness is critical for 2-3 points. Testing proves implementation works.

### 8.7 RTOS Completion Checklist
- [ ] RTOS lab completed and submitted → 1 point
- [ ] If RTOS in project:
  - [ ] Proposal approved by teacher
  - [ ] Tasks correctly implemented
  - [ ] Synchronization working
  - [ ] Timing validated
  - [ ] Report documents RTOS design
  - [ ] → 2-3 points (depending on quality)

---

## 📋 PHASE 9: OPTIONAL TASKS (Task 4 OR 5 for Extra Points)

### 9.1 Deciding Between Task 4 and Task 5
- [ ] **Task 4** (OLED + Brightness):
  - Pros: Visual feedback, user-friendly, demonstrates SPI/ADC/PWM
  - Cons: Requires external library integration, display programming
  - Effort: Medium-high (library integration + countdown logic)

- [ ] **Task 5** (UART Protocol):
  - Pros: Professional interface, demonstrates protocol design, easier testing
  - Cons: Less visually impressive, requires terminal program
  - Effort: Medium (protocol state machine + parameter validation)

- [ ] Choose ONE (both not needed for max points)
- [ ] Document decision rationale in project notes

**Why**: Either Task 4 or 5 required for 2+ points in Category 2. Choose based on interests and time.

### 9.2 Task 4: OLED Display Implementation
- [ ] **Step 1**: Integrate SSD1306 library
  - [ ] Download from https://github.com/afiskon/stm32-ssd1306
  - [ ] Add files to project: `ssd1306.c`, `ssd1306.h`, `fonts.c`, `fonts.h`
  - [ ] Configure SPI3 for OLED (already done for shift registers)
  - [ ] Initialize OLED in main():
    ```c
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    ```

- [ ] **Step 2**: Implement countdown bars
  ```c
  void DrawCountdownBar(uint8_t x, uint8_t y, uint8_t width, 
                        uint32_t current, uint32_t max) {
      uint8_t bar_width = (width * current) / max;
      ssd1306_DrawRectangle(x, y, x + bar_width, y + 4, White);
  }
  
  void UpdateOLEDDisplay(void) {
      ssd1306_Fill(Black);
      
      // Labels
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("P1 W1 P2 W2", Font_6x8, White);
      ssd1306_SetCursor(0, 20);
      ssd1306_WriteString("G1 R1 G2 R2", Font_6x8, White);
      
      // Countdown bars (8 total)
      DrawCountdownBar(0, 10, 30, GetRemaining(PL1_PED_DELAY), pedestrianDelay);
      DrawCountdownBar(32, 10, 30, GetRemaining(PL1_WALK_DELAY), walkingDelay);
      // ... 6 more bars
      
      ssd1306_UpdateScreen();
  }
  ```

- [ ] **Step 3**: Implement ADC brightness control
  ```c
  void UpdateBrightness(void) {
      uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
      uint16_t duty = 999 - ((adc_value * 999) / 4095);  // Invert (OE# active LOW)
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
  }
  ```

- [ ] **Step 4**: Test display
  - [ ] Verify OLED initializes (display lights up)
  - [ ] Test countdown bars decrease correctly
  - [ ] Test potentiometer changes brightness smoothly
  - [ ] Verify display update rate (100ms typical)

- [ ] Document OLED integration in report with photos

**Why**: Task 4 demonstrates multiple peripherals (SPI, ADC, PWM, display). Visual feedback impressive for demonstrations.

### 9.3 Task 5: UART Configuration Interface
- [ ] **Step 1**: Define protocol
  ```c
  // 4-byte packet: [Param ID][0x00][Value MSB][Value LSB]
  typedef enum {
      PARAM_TOGGLE_FREQ = 0x01,      // 1-10 Hz
      PARAM_PEDESTRIAN_DELAY = 0x02, // 1000-10000 ms
      PARAM_WALKING_DELAY = 0x03,    // 1000-15000 ms
      PARAM_ORANGE_DELAY = 0x04,     // 500-3000 ms
      PARAM_GREEN_DELAY = 0x05,      // 2000-10000 ms
      PARAM_RED_DELAY_MAX = 0x06     // 5000-30000 ms
  } ParameterID;
  ```

- [ ] **Step 2**: Implement UART receive
  ```c
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
      if (huart == &huart2) {
          ProcessUARTPacket(rx_buffer);
          HAL_UART_Receive_IT(&huart2, rx_buffer, 4);  // Re-enable
      }
  }
  
  void ProcessUARTPacket(uint8_t *packet) {
      uint8_t param_id = packet[0];
      uint16_t value = (packet[2] << 8) | packet[3];
      
      if (packet[1] != 0x00) {
          SendNACK();  // Invalid packet format
          return;
      }
      
      if (ValidateParameter(param_id, value)) {
          UpdateParameter(param_id, value);
          SendACK();
      } else {
          SendNACK();
      }
  }
  ```

- [ ] **Step 3**: Implement parameter validation
  ```c
  bool ValidateParameter(uint8_t param_id, uint16_t value) {
      switch (param_id) {
          case PARAM_TOGGLE_FREQ:
              return (value >= 1 && value <= 10);
          case PARAM_PEDESTRIAN_DELAY:
              return (value >= 1000 && value <= 10000);
          // ... other parameters
          default:
              return false;  // Unknown parameter
      }
  }
  ```

- [ ] **Step 4**: Test protocol
  - [ ] Send valid packets via terminal → verify ACK (0x01)
  - [ ] Send invalid packets → verify NACK (0x00)
  - [ ] Test boundary values (min, max, out-of-range)
  - [ ] Verify parameters update in real-time
  - [ ] Document protocol with examples in report

**Why**: Task 5 demonstrates protocol design and error handling. Professional interface for configuration.

### 9.4 Optional Task Completion Checklist
- [ ] Task 4 OR Task 5 completed (not both required)
- [ ] Requirements R4.1-R4.2 OR R5.1-R5.x met
- [ ] Testing completed and documented
- [ ] Git commit: "Complete Task 4/5 - [Description]"
- [ ] **Category 2: 2 points unlocked** (Task 1 + 2 + Task 4/5)
- [ ] OR with Task 3: **3 points unlocked** (Task 1 + 2 + 3 + Task 4/5)

---

## 📋 PHASE 10: REPORT WRITING

### 10.1 Report Structure Setup
- [ ] Create report document (LaTeX, Word, or Google Docs)
- [ ] Set up template with required sections:
  1. Title Page (project name, student info, date, course)
  2. Abstract (150-250 words summary)
  3. Table of Contents (auto-generated)
  4. Introduction
  5. Requirements
  6. Architecture (hardware + software)
  7. Implementation
  8. Testing & Validation
  9. Results & Discussion
  10. Conclusion
  11. References
  12. Appendices (code, schematics)

- [ ] Configure numbering, headers, footers
- [ ] Set up figure/table caption numbering

**Why**: Professional structure demonstrates organization (Category 3 requirement).

### 10.2 Writing the Introduction
- [ ] **Problem Statement**: Describe traffic light control challenges
  - Safety (no conflicting green lights)
  - Efficiency (minimize wait times)
  - Pedestrian priority (accessible crossings)

- [ ] **Project Objectives**: What are you building?
  - Intelligent traffic intersection with 4 lights + 2 pedestrian crossings
  - State machine control with timing constraints
  - SPI-controlled shift registers
  - Optional features (OLED, UART, RTOS)

- [ ] **Background**: Brief embedded systems context
  - Real-time systems
  - State machines
  - Communication protocols (SPI, UART, I2C)

- [ ] **Report Structure**: Overview of report sections

- [ ] Keep introduction concise (1-2 pages)

**Why**: Introduction sets context and motivates project. Should be understandable to someone unfamiliar with course.

### 10.3 Requirements Section
- [ ] List ALL implemented tasks (Task 1, 2, 3, and 4 or 5)
- [ ] Present requirements in clear format:
  - R1.1: Initial state specification
  - R1.2: Button response behavior
  - ... etc.

- [ ] Group requirements by task
- [ ] Add brief explanation for each requirement
- [ ] Note any assumptions or clarifications

**Why**: Requirements section serves as contract - what system MUST do. Essential for testing validation.

### 10.4 Architecture Section (CRITICAL for Category 1)
- [ ] **Hardware Architecture**:
  - [ ] System block diagram (MCU + all peripherals)
  - [ ] Shift register daisy chain diagram
  - [ ] Pin connection table
  - [ ] Component specifications (current limits, voltage levels)
  
- [ ] **Software Architecture**:
  - [ ] Module dependency diagram
  - [ ] Traffic light state machine diagram (with all states/transitions)
  - [ ] Pedestrian crossing state machine diagram
  - [ ] Sequence diagram (pedestrian request scenario)
  - [ ] Timing diagram (showing orange/green/walking delays)

- [ ] **For each diagram**:
  - [ ] Professional drawing (use draw.io, Visio, or LaTeX TikZ)
  - [ ] Numbered caption (Figure 1: System Block Diagram)
  - [ ] Referenced in text ("...as shown in Figure 1...")
  - [ ] Textual explanation (1-2 paragraphs describing purpose, components, interactions)

- [ ] **Module Interfaces**:
  - [ ] Document each module's purpose
  - [ ] List key functions with brief descriptions
  - [ ] Show dependencies between modules

**Why**: Architecture section demonstrates planning and design thinking. Diagrams are REQUIRED for Category 1 points. Quality and explanation matter.

### 10.5 Implementation Section
- [ ] Describe implementation approach:
  - [ ] Development methodology (TDD)
  - [ ] Tools used (STM32CubeIDE, Git, etc.)
  - [ ] Key challenges encountered
  - [ ] Solutions to challenges

- [ ] Highlight interesting implementation details:
  - [ ] Daisy chain data ordering (U3→U2→U1)
  - [ ] Non-blocking timing approach
  - [ ] SPI configuration for shift registers
  - [ ] State machine transition logic

- [ ] **Code snippets** (select carefully):
  - [ ] Show interesting algorithms (e.g., direction change logic)
  - [ ] Include comments in snippets
  - [ ] Keep snippets short (<20 lines)
  - [ ] Reference full code in appendix

- [ ] If using RTOS:
  - [ ] Describe task structure and priorities
  - [ ] Explain synchronization mechanisms
  - [ ] Justify design decisions

**Why**: Implementation section shows HOW you built the system. Code snippets support explanation.

### 10.6 Testing & Validation Section (CRITICAL for Category 1)
- [ ] **Test Strategy**:
  - [ ] Explain TDD approach
  - [ ] Describe test categories (basic, corner cases, boundary, integration)
  - [ ] Justify test selection

- [ ] **Test Results**:
  - [ ] Create comprehensive test results table:
    ```
    | Test ID | Test Name | Category | Result | Date | Notes |
    |---------|-----------|----------|--------|------|-------|
    | T1.1 | Init State | Basic | PASS | 2025-11-20 | All LEDs correct |
    | T2.1 | No Cars | Basic | PASS | 2025-11-20 | Changes every greenDelay |
    | T3.1 | Dual Ped | Corner | PASS | 2025-11-21 | Only one green |
    ```

  - [ ] Include statistics (X% passed, Y failed, Z total)
  - [ ] Show test output logs (from UART)

- [ ] **Physical Validation**:
  - [ ] Document hardware testing
  - [ ] Include photos of setup
  - [ ] Reference test videos (if submitted separately)

- [ ] **Issues Found & Fixed**:
  - [ ] Document bugs discovered during testing
  - [ ] Explain root causes
  - [ ] Describe fixes implemented

**Why**: Testing section proves system works. Test results table is REQUIRED for Category 1. Transparency about bugs demonstrates maturity.

### 10.7 Results & Discussion
- [ ] **Results**:
  - [ ] Summarize what was achieved
  - [ ] List all implemented requirements (R1.1-R1.6, R2.1-R2.8, etc.)
  - [ ] Quantify performance (timing accuracy, response times)

- [ ] **Discussion**:
  - [ ] Compare actual vs intended behavior
  - [ ] Discuss limitations (if any)
  - [ ] Reflect on design decisions (what worked well, what could improve)
  - [ ] Alternative approaches considered

- [ ] **Lessons Learned**:
  - [ ] Technical insights gained
  - [ ] Development process reflections
  - [ ] How would you approach differently next time?

**Why**: Discussion demonstrates critical thinking. Reflects on project experience beyond just "it works."

### 10.8 Conclusion
- [ ] Summarize project accomplishments (2-3 paragraphs)
- [ ] Restate how objectives were met
- [ ] Mention future work possibilities:
  - More complex traffic patterns
  - Machine learning for adaptive timing
  - IoT connectivity for remote monitoring
  - Additional sensors (camera for pedestrian detection)

- [ ] Keep conclusion concise (1 page max)

**Why**: Conclusion provides closure and demonstrates broader perspective.

### 10.9 References
- [ ] Cite ALL external resources:
  - [ ] Datasheets (STM32L476, 74HC595, SSD1306, etc.)
  - [ ] Reference manuals (RM0351)
  - [ ] Libraries (SSD1306 library)
  - [ ] Standards (SPI, UART, I2C specifications)
  - [ ] Academic papers (if any)
  - [ ] Online resources (tutorials, forums)

- [ ] Use consistent citation format (IEEE or APA)
- [ ] Ensure every reference is cited in text
- [ ] Include URLs and access dates for online resources

**Why**: References demonstrate research and give credit. Professional practice.

### 10.10 Appendices
- [ ] **Appendix A**: Complete source code (or link to Git repository)
- [ ] **Appendix B**: Schematics (if modified from provided)
- [ ] **Appendix C**: Test logs (full UART output)
- [ ] **Appendix D**: Additional diagrams (if too many for main text)
- [ ] **Appendix E**: Bill of materials (BOM)

**Why**: Appendices provide detailed information without cluttering main text.

### 10.11 Code Documentation (CRITICAL for Category 3)
- [ ] **File Headers** (every .c and .h file):
  ```c
  /**
   * @file    traffic_light.c
   * @brief   Traffic light state machine implementation
   * @author  Jakob Kuylenstierna
   * @date    2025-11-20
   * @version 1.0
   * 
   * This module implements the state machine for traffic light control,
   * including transitions between red, orange, and green states with
   * appropriate timing delays.
   */
  ```

- [ ] **Function Headers** (every function):
  ```c
  /**
   * @brief   Updates traffic light state machine
   * @details Checks timers, car presence, and pedestrian requests to
   *          determine appropriate state transitions. Implements all
   *          requirements from R2.1-R2.8.
   * 
   * @param[in]  light   Pointer to traffic light structure
   * @param[in]  tick    Current system tick (ms)
   * 
   * @return     void
   * 
   * @note       Must be called every iteration of main loop
   * @warning    Do not call before TrafficLight_Init()
   * 
   * @example
   * TrafficLight_Update(&tl1, HAL_GetTick());
   */
  void TrafficLight_Update(TrafficLight *light, uint32_t tick);
  ```

- [ ] **Inline Comments**:
  - [ ] Explain WHY, not what (code shows what)
  - [ ] Document hardware constraints
  - [ ] Note timing-critical sections
  - [ ] Reference requirements (e.g., "// R2.3: Orange phase required")

- [ ] **Naming Conventions**:
  - [ ] Functions: `Module_Action()` (e.g., `TrafficLight_Update()`)
  - [ ] Variables: snake_case (e.g., `pedestrian_delay`)
  - [ ] Constants: UPPER_CASE (e.g., `MAX_DELAY`)
  - [ ] Types: PascalCase (e.g., `TrafficLightState`)

- [ ] In report:
  - [ ] Describe module interfaces
  - [ ] List key functions with purposes
  - [ ] Show module dependencies

**Why**: Code documentation is REQUIRED for 1 point in Category 3. Good comments enable maintenance and demonstrate professionalism.

### 10.12 Report Quality Checklist
- [ ] **Writing**:
  - [ ] Clear, professional language
  - [ ] No grammatical errors (use spell-check)
  - [ ] Consistent terminology throughout
  - [ ] Active voice preferred (passive OK sometimes)
  - [ ] Technical accuracy

- [ ] **Formatting**:
  - [ ] Consistent fonts, spacing, margins
  - [ ] Numbered sections and subsections
  - [ ] All figures/tables numbered and captioned
  - [ ] Page numbers
  - [ ] Professional appearance

- [ ] **Completeness**:
  - [ ] All required sections present
  - [ ] All diagrams included
  - [ ] All references cited
  - [ ] Code commented
  - [ ] Test results documented

- [ ] **Length**: Typically 15-30 pages (quality over quantity)

- [ ] Final proofread (read entire report start to finish)
- [ ] Peer review (if working in team)
- [ ] Export to PDF for submission

**Why**: Report quality is REQUIRED for 1 point in Category 3. Attention to detail demonstrates professionalism.

---

## 📋 PHASE 11: FINAL INTEGRATION & TESTING

### 11.1 System Integration Verification
- [ ] All tasks completed and integrated:
  - [ ] Task 1: Single pedestrian crossing ✓
  - [ ] Task 2: Traffic control ✓
  - [ ] Task 3: Full system with SPI ✓
  - [ ] Task 4 or 5: Optional feature ✓

- [ ] Run complete test suite:
  - [ ] All TDD tests pass
  - [ ] Physical hardware tests pass
  - [ ] No regression (old features still work)

- [ ] Performance validation:
  - [ ] Timing accuracy ±10ms
  - [ ] No LED flickering
  - [ ] Button response <200ms
  - [ ] OLED update >10 FPS (if Task 4)
  - [ ] UART response <100ms (if Task 5)

**Why**: Final integration ensures all parts work together. No surprises at submission.

### 11.2 Edge Case & Stress Testing
- [ ] **Stress tests**:
  - [ ] Rapid button presses (100+ times)
  - [ ] All switches pressed continuously for 1 hour
  - [ ] Power cycle mid-operation → recovers correctly
  - [ ] Extreme parameter values (min/max)

- [ ] **Edge cases**:
  - [ ] Button pressed exactly during state transition
  - [ ] Multiple cars arrive simultaneously
  - [ ] Pedestrian request during orange phase
  - [ ] Both pedestrians request simultaneously

- [ ] Document any failures and fixes

**Why**: Stress testing reveals race conditions and timing bugs not found in normal testing.

### 11.3 Code Review & Cleanup
- [ ] Remove debug code (#ifdef DEBUG for any that remains)
- [ ] Remove unused variables/functions
- [ ] Verify all warnings resolved (0 warnings)
- [ ] Check for magic numbers (replace with named constants)
- [ ] Verify consistent code style
- [ ] Update comments (remove incorrect/outdated ones)
- [ ] Final build: 0 errors, 0 warnings

**Why**: Clean code demonstrates professionalism. Reviewers notice attention to detail.

### 11.4 Documentation Review
- [ ] Verify CLAUDE.md is current
- [ ] Update PROJECT_ROADMAP.md with final status
- [ ] Verify TESTING_LOG.md is complete
- [ ] Review README.md (add setup instructions)
- [ ] Check all file headers are present and accurate
- [ ] Verify function documentation is complete

**Why**: Documentation demonstrates systematic approach. Essential for report writing.

### 11.5 Git Repository Finalization
- [ ] Create final commit: "Project complete - ready for submission"
- [ ] Tag release: `git tag -a v1.0 -m "Submission version"`
- [ ] Push to remote (if using GitHub/GitLab)
- [ ] Verify repository is clean (no uncommitted changes)
- [ ] Export project as ZIP (backup)

**Why**: Version control demonstrates professional workflow. Tagged release marks submission version.

---

## 📋 PHASE 12: SUBMISSION PREPARATION

### 12.1 Source Code Preparation
- [ ] **Verify project name**: `PRO1_Jakob_Kuylenstierna` (CRITICAL!)
- [ ] Clean build:
  ```bash
  # In STM32CubeIDE
  Project → Clean → Clean all projects
  Project → Build All
  ```
  - [ ] 0 errors, 0 warnings

- [ ] Export project:
  - [ ] File → Export → General → Archive File
  - [ ] Include all source files (.c, .h)
  - [ ] Include project files (.project, .cproject)
  - [ ] Exclude Debug/Release folders (large, not needed)
  - [ ] Name: `PRO1_Jakob_Kuylenstierna_SourceCode.zip`

**Why**: Automated grading checks project name. Including Debug/ wastes space.

### 12.2 Binary Preparation
- [ ] Build Release configuration (optimized):
  - [ ] Right-click project → Build Configurations → Set Active → Release
  - [ ] Project → Build All

- [ ] Locate binary file:
  - [ ] `Release/PRO1_Jakob_Kuylenstierna.elf` (ELF format)
  - [ ] Or `Release/PRO1_Jakob_Kuylenstierna.bin` (binary format)

- [ ] Test binary on clean hardware:
  - [ ] Flash via STM32CubeProgrammer
  - [ ] Verify functionality (full test suite)

- [ ] Copy binary to submission folder:
  - [ ] Name: `PRO1_Jakob_Kuylenstierna.bin`

**Why**: Binary allows teachers to flash and test without building. Release build ensures optimized code.

### 12.3 Report Finalization
- [ ] Final report checklist:
  - [ ] All sections complete
  - [ ] All diagrams included and numbered
  - [ ] All references cited
  - [ ] Test results table included
  - [ ] Code snippets included
  - [ ] Appendices complete
  - [ ] PDF exported (not editable format)

- [ ] File naming:
  - [ ] Name: `PRO1_Jakob_Kuylenstierna_Report.pdf`

- [ ] Verify PDF:
  - [ ] All pages present
  - [ ] Figures display correctly
  - [ ] Links work (if any)
  - [ ] File size reasonable (<20 MB)

**Why**: PDF ensures consistent formatting across systems. Proper naming simplifies grading.

### 12.4 Submission Package Assembly
- [ ] Create submission folder structure:
  ```
  PRO1_Jakob_Kuylenstierna/
  ├── PRO1_Jakob_Kuylenstierna_SourceCode.zip   (Full project)
  ├── PRO1_Jakob_Kuylenstierna.bin              (Compiled binary)
  ├── PRO1_Jakob_Kuylenstierna_Report.pdf       (Written report)
  └── README.txt                            (Brief instructions)
  ```

- [ ] Create README.txt:
  ```
  IS1300 Traffic Light Project
  Student: Jakob Kuylenstierna
  Date: 2025-12-19
  
  Files:
  - PRO1_Jakob_Kuylenstierna_SourceCode.zip: Complete STM32CubeIDE project
  - PRO1_Jakob_Kuylenstierna.bin: Compiled binary (flash to address 0x08000000)
  - PRO1_Jakob_Kuylenstierna_Report.pdf: Project report (XX pages)
  
  Implemented Tasks:
  - Task 1: Single pedestrian crossing ✓
  - Task 2: Traffic control ✓
  - Task 3: Full system with SPI ✓
  - Task 4/5: [OLED Display / UART Protocol] ✓
  
  RTOS: [Yes/No]
  
  Special Notes:
  - [Any important information for grading]
  ```

- [ ] Compress submission folder:
  - [ ] Name: `PRO1_Jakob_Kuylenstierna_FINAL.zip`

**Why**: Organized submission demonstrates professionalism. README provides quick overview for graders.

### 12.5 Pre-Submission Verification
- [ ] **Checklist verification**:
  - [ ] Project name correct in all files
  - [ ] All three required files present (code, binary, report)
  - [ ] Binary tested on hardware
  - [ ] Report complete and proofread
  - [ ] File sizes reasonable (<50 MB total)

- [ ] **Grading criteria verification**:
  - [ ] Category 1 (Planning/Architecture/Testing): 2 points
    - [ ] TDD tests present and documented ✓
    - [ ] Architecture diagrams in report ✓
  
  - [ ] Category 2 (Implementation): 3 points
    - [ ] Task 1 complete ✓
    - [ ] Task 2 complete ✓
    - [ ] Task 3 complete ✓
    - [ ] Task 4 or 5 complete ✓
  
  - [ ] Category 3 (Report/Documentation): 2 points
    - [ ] Report well-written and structured ✓
    - [ ] Code commented and modules described ✓
  
  - [ ] Category 4 (RTOS): 1-3 points
    - [ ] RTOS Lab complete ✓
    - [ ] (Optional) FreeRTOS in project ✓
  
  - [ ] Deadline bonus: +1 point
    - [ ] Submitting before Dec 19, 2025 ✓

- [ ] **Expected grade**: [Calculate total points → map to grade]

**Why**: Final verification catches mistakes before submission. Ensures minimum points in each category.

### 12.6 Submission Process
- [ ] Read submission instructions on Canvas
- [ ] Upload to designated platform (Canvas, email, etc.)
- [ ] Verify upload successful (check file sizes, download to verify)
- [ ] Note submission timestamp (before deadline!)
- [ ] Keep local backup copy
- [ ] Email confirmation (if required)

**Why**: Proper submission ensures grading. Timestamp proves deadline met.

### 12.7 Post-Submission Tasks
- [ ] Backup all files to external drive
- [ ] Archive Git repository
- [ ] Celebrate completion! 🎉
- [ ] (Optional) Continue with Task 4/5 for learning
- [ ] (Optional) Enhance project with personal additions
- [ ] Prepare for exam (review RTOS concepts)

**Why**: Backup prevents data loss. Project can be portfolio piece for future job applications.

---

## 📋 PHASE 13: GRADING EXPECTATIONS

### 13.1 Category 1: Planning/Architecture/Testing (0-2 points)
**For 1 point (minimum)**:
- [ ] TDD tests present and functional
- [ ] Architecture diagrams in report (hardware + software)
- [ ] **Both required** (tests alone insufficient, diagrams alone insufficient)

**For 2 points**:
- [ ] Comprehensive test strategy documented
- [ ] Tests cover basic, corner, and boundary cases
- [ ] Multiple architecture diagrams with detailed explanations
- [ ] Professional diagram quality

**Common mistakes to avoid**:
- Only having tests OR diagrams (need both for ANY points)
- Tests that don't follow TDD (written after implementation)
- Diagrams without explanations
- Poor diagram quality (hand-drawn, unclear)

### 13.2 Category 2: Implementation (0-3 points)
**Points allocation**:
- 1 point: Task 1 AND Task 2
- 2 points: Task 1 AND Task 2 AND (Task 4 OR Task 5)
- 3 points: Task 1 AND Task 2 AND Task 3 AND (Task 4 OR Task 5)

**Critical requirements**:
- [ ] Task 3 MUST use SPI (GPIO fails Task 3)
- [ ] Task 3 MUST enforce "only one pedestrian green" (R3.3)
- [ ] All timing requirements met
- [ ] No safety violations (conflicting green lights)

**Common mistakes to avoid**:
- GPIO bit-banging in Task 3 (instant fail for Task 3)
- Both pedestrians green simultaneously (fails R3.3)
- Orange phase missing (violates R1.6, R2.3)
- Incorrect daisy chain order (LEDs wrong)

### 13.3 Category 3: Report/Documentation (0-2 points)
**For 1 point (report quality)**:
- [ ] Well-written, well-structured report
- [ ] All required sections present
- [ ] Diagrams numbered and referenced
- [ ] Relevant references cited
- [ ] Professional appearance

**For 2nd point (code documentation)**:
- [ ] Module interfaces described in report
- [ ] File headers present
- [ ] Function headers present
- [ ] Inline comments explain WHY
- [ ] Consistent naming conventions

**Common mistakes to avoid**:
- Missing sections (especially testing)
- Diagrams not referenced in text
- No code comments
- Poor writing quality (grammatical errors)
- Missing references

### 13.4 Category 4: Real-Time Tasks (0-3 points)
**For 1 point (minimum)**:
- [ ] RTOS Lab completed
- [ ] **Mandatory** - project fails without this

**For 2-3 points (FreeRTOS in project)**:
- [ ] Proposal submitted and approved BEFORE implementation
- [ ] FreeRTOS correctly integrated
- [ ] Multiple tasks with appropriate priorities
- [ ] Synchronization mechanisms (semaphores, mutexes, queues)
- [ ] Timing requirements met
- [ ] Report documents RTOS design

**2 vs 3 points**:
- 2 points: Basic RTOS usage (tasks, delays)
- 3 points: Advanced usage (priorities, synchronization, demonstrated understanding)

**Common mistakes to avoid**:
- Not completing RTOS Lab (instant project fail)
- Implementing RTOS without approval (no points)
- Poor task design (everything in one task)
- No synchronization (race conditions)
- Incorrect priorities (deadlines missed)

### 13.5 Minimum Requirements for Passing (Grade E)
- [ ] **CRITICAL**: Minimum 1 point in EVERY category
- [ ] Minimum 4 points total
- [ ] Project name correct (automated grading)
- [ ] All required files submitted

**Example passing configuration (E grade)**:
- Category 1: 1 point (minimal tests + diagrams)
- Category 2: 1 point (Task 1 + Task 2 only)
- Category 3: 1 point (basic report, no code docs)
- Category 4: 1 point (RTOS Lab only)
- Total: 4 points = Grade E

### 13.6 Target Configuration (A grade)
- [ ] Category 1: 2 points (comprehensive tests + detailed architecture)
- [ ] Category 2: 3 points (Tasks 1+2+3 + Task 4/5)
- [ ] Category 3: 2 points (excellent report + full code docs)
- [ ] Category 4: 3 points (RTOS Lab + advanced FreeRTOS integration)
- [ ] Deadline bonus: +1 point
- [ ] Exam bonus: +1 point (possible)
- [ ] Total: 11 points = Grade A

**Why pursue maximum points?**
- Demonstrates mastery of embedded systems
- Portfolio piece for job applications
- Deeper learning of RTOS concepts
- Pride in accomplishment

---

## 📋 QUICK REFERENCE: CRITICAL REMINDERS

### ⚠️ INSTANT FAIL CONDITIONS
1. **Project name wrong**: MUST be `PRO1_Jakob_Kuylenstierna`
2. **RTOS Lab not completed**: Automatic project fail
3. **0 points in any category**: Project fails even with high total
4. **Late submission**: No deadline bonus (but can still pass)
5. **Plagiarism**: Disciplinary action

### 🔑 KEY SUCCESS FACTORS
1. **Start early**: 120 hours of work expected
2. **Test continuously**: Don't wait until end
3. **Document as you go**: Don't postpone report writing
4. **Ask questions**: Use lab sessions for help
5. **Follow checklist**: Systematic approach prevents mistakes

### 📊 Time Allocation Suggestion
- Week 1: Setup + Phase 1-2 (hardware config, shift registers) - 25 hours
- Week 2: Phase 3-4 (TDD setup, architecture) - 25 hours
- Week 3: Phase 5-6 (Task 1, Task 2) - 30 hours
- Week 4: Phase 7-9 (Task 3, RTOS, Task 4/5) - 25 hours
- Week 5: Phase 10-12 (Report, testing, submission) - 15 hours
- Total: 120 hours

### 🎯 Prioritization for Limited Time
**If short on time, prioritize**:
1. RTOS Lab (MANDATORY)
2. Tasks 1+2 (minimum implementation points)
3. Basic tests + basic diagrams (minimum Category 1)
4. Basic report (minimum Category 3)
5. Task 3 (if time allows - significant jump to 3 points)
6. Task 4 or 5 (if time allows - 2 points achievable)
7. RTOS integration (if time allows - 2-3 points)

**Minimum viable project (Grade E)**:
- RTOS Lab ✓
- Task 1 + 2 ✓
- Minimal tests + diagrams ✓
- Basic report ✓
- Total: 4 points = Grade E (Passing)

---

## 📝 FINAL CHECKLIST SUMMARY

### Pre-Submission Verification
- [ ] Project name: `PRO1_Jakob_Kuylenstierna` ✓
- [ ] Category 1: ≥1 point (tests + architecture) ✓
- [ ] Category 2: ≥1 point (Task 1 + Task 2 minimum) ✓
- [ ] Category 3: ≥1 point (report quality) ✓
- [ ] Category 4: ≥1 point (RTOS Lab) ✓
- [ ] Total points: ≥4 (minimum for Grade E) ✓
- [ ] Submission before Dec 19, 2025 ✓

### Submission Files
- [ ] Source code ZIP file ✓
- [ ] Compiled binary (.bin or .elf) ✓
- [ ] Report PDF file ✓
- [ ] README with instructions ✓

### Quality Assurance
- [ ] Code compiles: 0 errors, 0 warnings ✓
- [ ] Binary tested on hardware ✓
- [ ] All TDD tests pass ✓
- [ ] Report proofread ✓
- [ ] All diagrams included ✓

---

**END OF CHECKLIST**

This checklist covers the complete project from setup to submission. Use it as a living document - check off items as you complete them. Good luck with your project! 🚀

**Last Updated**: 2025-11-20  
**Version**: 1.0  
**Author**: Claude (for Jakob's IS1300 Project)
