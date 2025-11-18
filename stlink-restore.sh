#!/bin/bash
# Restore USB daemon after ST-Link work

echo "🔄 Restoring USB daemon..."

if [ "$EUID" -ne 0 ]; then
    echo "❌ Please run with sudo: sudo ./stlink-restore.sh"
    exit 1
fi

killall -CONT usbd
echo "✅ USB daemon resumed - all USB devices should work normally"
