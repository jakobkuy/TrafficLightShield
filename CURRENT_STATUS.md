# IS1300 Traffic Light Project - Current Status

**Last Updated**: 2025-11-20
**Current Phase**: Ready to Start Task 1 (Pedestrian Crossing)
**Project Name**: `PRO1_Jakob_Kuylenstierna` 

---

## 🎯 Current Focus

**Working On**: Setting up foundation for Task 1 implementation
**Next Milestone**: Complete Task 1 (Single Pedestrian Crossing)
**Deadline**: Project due Dec 19, 2025 | RTOS Lab due earlier

---

## ✅ Completed (As of 2025-11-20)

### Hardware & Configuration
- ✅ STM32CubeIDE project configured (80 MHz, Nucleo-L476RG)
- ✅ All GPIO pins configured and labeled correctly
- ✅ ADC1 configured (PB_01, 12-bit, continuous mode)
- ✅ TIM3 PWM configured (PC_07, 1 kHz for brightness control)
- ✅ USART2 configured (115200 baud for debug/Task 5)
- ✅ Pull-down resistors added to all input pins (car switches, buttons)

### Code Modules
- ✅ `shift_register.h/c` - Complete and tested
  - GPIO bit-banging implementation working
  - LED bit definitions correct for all 3 registers
  - Test function validates hardware
- ✅ PWM brightness control initialized and set to max brightness
- ✅ All hardware fixes applied (4 critical issues resolved)

### Documentation
- ✅ CLAUDE.md - Comprehensive technical reference
- ✅ PROJECT_HANDBOOK.md - Full checklist (renamed from PROJECT_CHECKLIST.md)
- ✅ Git repository initialized with clean history

---

## 🔄 In Progress / Immediate Next Steps

### Architecture Decisions Needed
- [ ] **Timing approach**: HAL_GetTick() polling vs dedicated timer interrupts?
  - Recommendation: Start with HAL_GetTick() (simpler), optimize later if needed

- [ ] **State machine architecture**: Centralized controller vs distributed modules?
  - Recommendation: Distributed (traffic_light.c, pedestrian.c with coordinator)

- [ ] **Button debouncing**: 50ms vs 200ms threshold?
  - Recommendation: 200ms (per CLAUDE.md specification, reduces false triggers)

### Modules to Create
1. **timing.c/h** - Non-blocking delay management (NEEDED FOR EVERYTHING)
2. **pedestrian.c/h** - Pedestrian crossing state machine (Task 1)
3. **traffic_light.c/h** - Traffic light state machine (Task 2)
4. **tests.c/h** - TDD test suite (Category 1 requirement)

---

## ⚠️ Blockers & Questions

### Open Questions
1. Do you want to use interrupts for button presses, or poll in main loop?
2. Should we implement a simple coordinator module, or handle coordination in main.c?
3. For TDD: Should tests run on hardware, or create mock layer for unit testing?

### Pending Decisions
- Task 4 vs Task 5? (Need to pick one for 2+ points in Category 2)
- FreeRTOS integration? (Worth 2-3 points but needs pre-approval)

---

## 📋 Grade Projection (Current: 0/11 points)

| Category | Current | Target | Status |
|----------|---------|--------|--------|
| **Cat 1**: Planning/Architecture/Testing | 0 | 2 | Need tests + diagrams |
| **Cat 2**: Implementation | 0 | 3 | Need Task 1+2+3+Task4/5 |
| **Cat 3**: Report/Documentation | 0 | 2 | Need report + code docs |
| **Cat 4**: RTOS | 0 | 1-3 | Need RTOS Lab minimum |
| **Deadline Bonus** | 0 | +1 | Submit before Dec 19 |
| **TOTAL** | **0** | **11 (A)** | - |

### Path to Grade A (11 points)
- [ ] Complete RTOS Lab → +1 point (MANDATORY)
- [ ] Implement Task 1, 2, 3, and Task 4 or 5 → +3 points
- [ ] Create comprehensive tests + architecture diagrams → +2 points
- [ ] Write excellent report with full code docs → +2 points
- [ ] (Optional) FreeRTOS integration → +2 points (vs +1 for lab only)
- [ ] Submit before deadline → +1 bonus point

**Minimum for Grade E (4 points)**: Task 1+2 (1pt) + Basic tests/diagrams (1pt) + Basic report (1pt) + RTOS Lab (1pt)

---

## 🚀 Next 3 Priority Tasks

1. **Create timing.c/h module** (~30 min)
   - Non-blocking timer structure
   - Start, check expiry, get remaining time
   - Foundation for all state machines

2. **Set up TDD infrastructure** (~1 hour)
   - Create tests.c/h files
   - Implement test runner with UART output
   - Write first test (shift register basic functionality)

3. **Design Task 1 state machine** (~30 min)
   - Draw state diagram (PED_IDLE, PED_WAITING, PED_CROSSING)
   - Define transitions and timing
   - Get approval before implementing

---

## 🎓 RTOS Lab Status

- [ ] RTOS Lab started
- [ ] Lab exercises completed
- [ ] Lab report written
- [ ] Lab submitted

⚠️ **CRITICAL**: RTOS Lab is MANDATORY. Project fails without it!

---

## 📝 Notes & Decisions

### Recent Decisions (2025-11-20)
- ✅ Use GPIO bit-banging for Tasks 1-2 (simpler)
- ✅ Will migrate to SPI for Task 3 (requirement R3.4)
- ✅ PWM brightness control ready for Task 4 if we choose it
- ✅ UART ready for Task 5 if we choose it

### Known Issues
- None currently (all 4 critical issues from code review fixed)

### Hardware Notes
- ST-Link working after USB adapter replacement
- Using GDB server (not OpenOCD)
- All LEDs tested and working correctly

---

## 📂 Project Structure

```
TrafficLightShield/
├── Core/
│   ├── Inc/
│   │   ├── shift_register.h        ✅ Complete
│   │   ├── timing.h                📋 TODO (Priority 1)
│   │   ├── pedestrian.h            📋 TODO (Task 1)
│   │   ├── traffic_light.h         📋 TODO (Task 2)
│   │   └── tests.h                 📋 TODO (TDD)
│   └── Src/
│       ├── main.c                  ✅ Complete (clean loop)
│       ├── shift_register.c        ✅ Complete
│       ├── timing.c                📋 TODO
│       ├── pedestrian.c            📋 TODO
│       ├── traffic_light.c         📋 TODO
│       └── tests.c                 📋 TODO
├── CLAUDE.md                       ✅ Technical reference
├── PROJECT_HANDBOOK.md             ✅ Complete checklist (renamed)
├── CURRENT_STATUS.md               ✅ This file (active status)
└── README.md                       📋 TODO (for submission)
```

---

## 💡 Working with Claude (AI Collaboration)

### How to Start a Session
1. Tell me what you want to work on (e.g., "Let's implement Task 1")
2. I'll read this file to understand current status
3. I'll use TodoWrite to track active tasks
4. I'll reference CLAUDE.md for technical details
5. I'll reference PROJECT_HANDBOOK.md only when needed (grading, deep reference)

### How I'll Track Progress
- **TodoWrite tool** - Active task management (you'll see in UI)
- **This file** - Updated after major milestones
- **Git commits** - Mark completion of features

### When to Update This File
- After completing a major module
- When making important decisions
- Before/after long breaks (context refresh)
- Weekly status review

---

**End of Status File**

*Keep this file lean (<150 lines). Move completed items to git history. Archive old notes.*
