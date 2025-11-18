# IS1300 Traffic Light Shield Project

KTH Embedded Systems course project - Traffic intersection control system using STM32L476RG.

---

## CRITICAL: ST-Link macOS Fix Required

**If you're on macOS and having issues with ST-Link detection in STM32 CubeIDE, YOU MUST RUN THIS BEFORE EVERY SESSION:**

```bash
cd /Users/jakob/dev/kth/IS1300/TrafficLightShield
sudo ./stlink-fix.sh
```

**After you're done working:**
```bash
sudo ./stlink-restore.sh
```

**Full documentation:** See [STLINK-MACOS-FIX.md](STLINK-MACOS-FIX.md) for complete details on the issue and solution.

**The Problem:** macOS's USB driver claims exclusive ownership of ST-Link, blocking CubeIDE access.
**The Solution:** Temporarily suspend the USB daemon before opening CubeIDE.

---

## Hardware

- **MCU**: STM32L476RG (Nucleo-L476RG board)
- **Shield**: Custom Traffic Light Shield with 74HC595D shift registers
- **Components**:
  - 4 traffic lights (TL1-TL4) with Red/Yellow/Green LEDs
  - 2 pedestrian crossings (PL1-PL2) with Red/Green/Blue LEDs
  - 4 car detection switches
  - 2 pedestrian buttons
  - OLED display (SSD1306)
  - Sensors (accelerometer, temp/humidity)
  - Potentiometer for brightness control

## Project Structure

```
TrafficLightShield/
├── Core/
│   ├── Inc/
│   │   ├── shift_register.h    # 74HC595 shift register driver
│   │   └── ...
│   └── Src/
│       ├── main.c              # Main application
│       ├── shift_register.c    # Shift register implementation
│       └── ...
├── CLAUDE.md                   # Detailed technical documentation
├── block_diagram.txt           # Hardware connection diagram
└── TrafficLightShield.ioc      # STM32CubeMX configuration
```

## Current Status

### Completed
- ✅ STM32CubeIDE project setup
- ✅ Pin configuration and verification
- ✅ Shift register driver (GPIO bit-banging)
- ✅ ADC for potentiometer (ADC1_IN16)
- ✅ TIM3 PWM for brightness control
- ✅ USART2 for debugging
- ✅ Basic LED test sequence

### To Do
- [ ] Traffic light state machine (Task 2/3)
- [ ] Pedestrian crossing logic (Task 1/3)
- [ ] OLED display integration (Task 4)
- [ ] Brightness control via potentiometer (Task 4)
- [ ] Complete traffic system integration (Task 3)

## Building and Flashing

1. Open `TrafficLightShield.ioc` in STM32CubeIDE
2. Build: `Project > Build Project` or `Ctrl+B`
3. Flash: Connect Nucleo board via USB and click `Run` or `Debug`

## Technical Details

See [CLAUDE.md](CLAUDE.md) for comprehensive technical documentation including:
- Detailed pin mappings
- Shift register control strategies
- Implementation tasks and requirements
- Hardware specifications
- Timing considerations

## Course Information

- **Course**: IS1300 Embedded Systems, KTH
- **Deadline**: December 19, 2025
- **Requirements**: Implement 4 tasks for full points

## License

Educational project for KTH IS1300 course.
