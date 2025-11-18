#!/bin/bash
# ST-Link Fix for STM32 CubeIDE on macOS
# This prevents macOS CDC driver from claiming exclusive ownership

echo "🔧 ST-Link macOS Fix"
echo "===================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ Please run with sudo: sudo ./stlink-fix.sh"
    exit 1
fi

# Check if ST-Link is connected
if ioreg -p IOUSB -w0 -l | grep -q "STM32 STLink"; then
    echo "✅ ST-Link detected"
else
    echo "⚠️  ST-Link not detected. Please connect your device."
    exit 1
fi

echo ""
echo "Suspending USB daemon to release ST-Link..."
killall -STOP usbd

echo "✅ USB daemon suspended"
echo ""
echo "📌 Next steps:"
echo "   1. Launch STM32 CubeIDE"
echo "   2. Connect to your ST-Link"
echo "   3. Once connected, you can resume usbd with: sudo killall -CONT usbd"
echo ""
echo "💡 To resume USB daemon later:"
echo "   sudo killall -CONT usbd"
echo ""
echo "⚠️  Note: Other USB devices may not work properly while usbd is suspended"
