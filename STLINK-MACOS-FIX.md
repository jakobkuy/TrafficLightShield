# ST-Link macOS Fix - Complete Guide

## Quick Start Workflow

```bash
# 1. Connect ST-Link to your Mac
# 2. Suspend USB daemon
sudo ./stlink-fix.sh

# 3. Use STM32CubeIDE (program, debug, etc.)
# 4. When finished, restore USB daemon
sudo ./stlink-restore.sh
```

**⚠️ Important:** Keep `usbd` suspended the entire time you're using ST-Link. Unplugging/replugging will require running the fix script again.

---

## The Problem

macOS's `AppleUSBCDCCompositeDevice` driver claims **exclusive ownership** of the ST-Link, preventing STM32CubeIDE from accessing it for programming and debugging.

**Smoking Gun Evidence:**
```
"UsbExclusiveOwner" = "AppleUSBCDCCompositeDevice"
```

### Why This Happens

1. **ST-Link is a composite USB device** with multiple interfaces:
   - Debug interface (for GDB/JTAG/SWD)
   - Virtual COM port (for UART passthrough)

2. **macOS aggressively claims the COM port** when the device is plugged in

3. **The USB daemon (`usbd`) won't share access** with other applications

4. **STM32CubeIDE can't connect** because macOS already owns the device

This is a **macOS-specific issue** - Windows and Linux handle composite USB devices differently and allow shared access.

---

## The Solution

Running `sudo killall -STOP usbd` temporarily **suspends** (not kills) the USB daemon, preventing macOS from claiming the device and allowing CubeIDE to access it.

### Why Suspend Instead of Kill?

- Killing `usbd` would break **all** USB devices system-wide
- Suspending (`-STOP` signal) pauses the process while keeping it in memory
- Other USB devices continue to work (mostly - see caveats below)
- Easily reversible with `killall -CONT usbd`

---

## Detailed Usage Instructions

### Step 1: Run the Fix Script BEFORE Opening CubeIDE

```bash
cd /Users/jakob/dev/kth/IS1300/TrafficLightShield
sudo ./stlink-fix.sh
```

**What this script does:**
- Checks if ST-Link is connected via `ioreg`
- Suspends USB daemon (`usbd`)
- Displays next steps and reminder

**Expected output:**
```
🔧 ST-Link macOS Fix
====================

✅ ST-Link detected

Suspending USB daemon to release ST-Link...
✅ USB daemon suspended

📌 Next steps:
   1. Launch STM32 CubeIDE
   2. Connect to your ST-Link
   3. Once connected, you can resume usbd with: sudo killall -CONT usbd
```

### Step 2: Open STM32CubeIDE

Now launch STM32CubeIDE and use it normally:
- Program your board
- Debug with breakpoints
- Flash firmware
- Monitor serial output

The IDE should detect the ST-Link immediately.

### Step 3: When Done, Restore USB Daemon

```bash
sudo ./stlink-restore.sh
```

**Why this matters:**
- Other USB devices may not work properly while `usbd` is suspended
- Restoring returns your Mac to normal USB operation
- Always restore when finished debugging

---

## Manual Commands (If Scripts Don't Work)

### Fix (Before CubeIDE):
```bash
sudo killall -STOP usbd
```

### Restore (After CubeIDE):
```bash
sudo killall -CONT usbd
```

### Alternative: Suspend Before Plugging In
```bash
# Unplug ST-Link first
sudo killall -STOP usbd
# Now plug in ST-Link
# Open CubeIDE
```

This can help if the device was already claimed before suspension.

---

## Verification Commands

### Check if ST-Link is Detected:
```bash
ioreg -p IOUSB -w0 -l | grep -i "st-link\|stm32\|stmicro" -A 5 -B 5
```

**Look for:**
- `"USB Product Name" = "STM32 STLink"`
- `"USB Vendor Name" = "STMicroelectronics"`
- `"UsbExclusiveOwner"` - if this exists, macOS has claimed it

**Example output when working:**
```
"USB Product Name" = "STM32 STLink"
"USB Vendor Name" = "STMicroelectronics"
"USB Serial Number" = "066BFF505250827867072829"
"idVendor" = 0x0483
"idProduct" = 0x3752
```

### Check USB Daemon Status:
```bash
ps aux | grep usbd
```

**If suspended:** Process will show status `T` (stopped)
**If running:** Process will show status `S` (sleeping) or `R` (running)

### Check System USB Information:
```bash
system_profiler SPUSBDataType | grep -A 10 "STM32"
```

---

## Troubleshooting

### ST-Link Still Not Detected in CubeIDE?

**1. Unplug and replug ST-Link after running fix script**
   - Device may need to re-enumerate without macOS claiming it

**2. Try suspending BEFORE plugging in:**
   ```bash
   # Unplug ST-Link
   sudo killall -STOP usbd
   # Now plug in ST-Link
   # Open CubeIDE
   ```

**3. Check for multiple ST-Link devices:**
   ```bash
   system_profiler SPUSBDataType | grep -A 10 "STM32"
   ```
   Only one should be connected.

**4. Kill CubeIDE completely and restart:**
   ```bash
   killall -9 stm32cubeidec
   sudo ./stlink-fix.sh
   # Launch CubeIDE again
   ```

**5. Check USB hub/cable:**
   - Try connecting directly to Mac (not through hub)
   - Try a different USB-C cable
   - Some hubs can cause issues with USB device enumeration

**6. Verify scripts are executable:**
   ```bash
   ls -l stlink-fix.sh stlink-restore.sh
   ```
   Should show `-rwxr-xr-x`. If not:
   ```bash
   chmod +x stlink-fix.sh stlink-restore.sh
   ```

### Other USB Devices Not Working?

You probably forgot to restore `usbd`. Run:
```bash
sudo ./stlink-restore.sh
```

### LD1 Not Flashing on Nucleo Board?

**Normal behavior:**
- **Green flashing slowly:** Enumerated, communication OK
- **Red flashing:** Communication error
- **Green+Red alternating:** After successful fix - board is ready

If no flashing:
- Check USB cable connection
- Verify power (some cables are charge-only, not data)
- Try a different USB port

---

## Technical Deep Dive

### What Exactly Is Happening?

**Normal State (ST-Link won't work):**
1. ST-Link plugged in
2. macOS USB stack (`usbd`) detects device
3. Loads `AppleUSBCDCCompositeDevice` driver
4. Driver claims "exclusive" access to the device
5. STM32CubeIDE cannot access device (macOS owns it)

**With Fix Applied:**
1. Run `sudo killall -STOP usbd`
2. USB daemon freezes (paused, not killed)
3. Prevents daemon from claiming new devices
4. Plug in ST-Link or replug if already connected
5. CubeIDE can grab the ST-Link first (no competition)
6. Programming and debugging work normally

**Why Unplugging Breaks It:**
When you unplug/replug the ST-Link:
1. Device disconnects from USB bus
2. If you resumed `usbd`, it's running again
3. macOS immediately claims the device on reconnection
4. Back to square one - CubeIDE can't access it

**Solution:** Keep `usbd` suspended the entire debug session.

### USB Device Hierarchy
```
macOS USB Stack
├── usbd (USB daemon)
│   ├── AppleUSBCDCCompositeDevice (CLAIMS ST-Link)
│   ├── Other USB drivers
│   └── Device enumeration
└── User-space applications
    └── STM32CubeIDE (BLOCKED by exclusive ownership)
```

### Alternative Approaches (Why They Don't Work)

**Kernel Extensions (kexts):**
- Could blacklist ST-Link from macOS drivers
- Requires disabling System Integrity Protection (SIP)
- macOS has heavily restricted kexts since Big Sur
- Not recommended for security reasons

**libusb detachment:**
- STM32CubeIDE tries `detach_kernel_driver()`
- macOS System Integrity Protection blocks this
- Works on Linux, not on macOS

**Permanent solutions:**
- None exist that don't compromise macOS security
- Suspending `usbd` is the most reliable workaround

---

## Additional Troubleshooting (Less Common Issues)

### STM32CubeProgrammer Path Issues

If you see errors about missing STM32CubeProgrammer, the GDB server may be looking in the wrong location.

**Expected structure:**
```
/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/
  └── STM32_Programmer_CLI/
      └── bin/
          └── STM32_Programmer_CLI (executable)
```

**Actual structure on some installations:**
```
/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/
  └── STM32CubeProgrammer.app/
      └── Contents/
          └── Resources/
              └── bin/
                  └── STM32_Programmer_CLI (executable)
```

**Fix Option 1: Use OpenOCD Instead**
1. `Run` → `Debug Configurations...` → `Debugger` tab
2. Change "Debug probe" from `ST-LINK (ST-LINK GDB server)` to `ST-LINK (OpenOCD)`
3. Apply and debug

**Fix Option 2: Reinstall STM32CubeProgrammer**
1. Uninstall current version
2. Download fresh from: https://www.st.com/en/development-tools/stm32cubeprog.html
3. Install to default location
4. Restart STM32CubeIDE

**Fix Option 3: Create Symbolic Link**
```bash
cd /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/
mkdir -p STM32_Programmer_CLI/bin
cd STM32_Programmer_CLI/bin
ln -s ../../STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI STM32_Programmer_CLI
```

### Check Installed Versions

**ST-LINK GDB Server Location:**
```
/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/
  com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.macos64_*/
  tools/bin/ST-LINK_gdbserver
```

**Check version:**
```bash
find /Applications/STM32CubeIDE.app -name "ST-LINK_gdbserver" -exec {} --version \;
```

**OpenOCD location:**
```bash
find /Applications/STM32CubeIDE.app -name "openocd" -type f
```

---

## Quick Reference Table

| Task | Command |
|------|---------|
| Fix before CubeIDE | `sudo ./stlink-fix.sh` |
| Restore after done | `sudo ./stlink-restore.sh` |
| Manual suspend | `sudo killall -STOP usbd` |
| Manual resume | `sudo killall -CONT usbd` |
| Check if detected | `ioreg -p IOUSB -w0 -l \| grep -i "stm32"` |
| Check daemon status | `ps aux \| grep usbd` |
| System USB info | `system_profiler SPUSBDataType \| grep -A 10 "STM32"` |

---

## Files in This Directory

- **`stlink-fix.sh`** - Run this before using CubeIDE (suspends USB daemon)
- **`stlink-restore.sh`** - Run this when done with ST-Link (restores USB daemon)
- **`STLINK-MACOS-FIX.md`** - This file (complete documentation)

---

## Your Breakthrough Moment

**What you discovered that worked:**
1. Ran `sudo killall -STOP usbd`
2. LD1 on Nucleo board started flashing green and red
3. CubeIDE detected ST-Link immediately
4. Programming and debugging worked perfectly

**This documentation ensures you can reproduce that success every time!**

---

## System Information Reference

**Your Hardware:**
- Board: NUCLEO-L476RG (STM32L476RG MCU)
- ST-Link Serial: 066BFF505250827867072829
- Vendor ID: 0x0483 (STMicroelectronics)
- Product ID: 0x3752
- Mac: Apple Silicon (arm64)
- USB Hub: Dell DA20 Adapter (may cause issues, try direct connection if problems occur)

**Software:**
- STM32CubeIDE: Installed (version from Eclipse plugins timestamps)
- STM32CubeProgrammer: Installed at `/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer`
- ST-LINK GDB Server: v7.11.0

---

## Known Limitations

- **Other USB devices may misbehave** while `usbd` is suspended
- **Don't plug/unplug other USB devices** during debug session
- **Keyboard and mouse usually work fine** (HID devices have separate handling)
- **External drives may not mount** while suspended
- **Always restore USB daemon** when finished to return to normal operation

---

## Final Tips

✅ **Do This:**
- Run fix script BEFORE opening CubeIDE
- Keep ST-Link connected throughout debug session
- Restore USB daemon when finished

❌ **Don't Do This:**
- Don't unplug/replug ST-Link while debugging
- Don't forget to restore USB daemon
- Don't run fix script after CubeIDE is already open

---

Last Updated: 2025-11-18
macOS ST-Link Fix - Tested and Working
