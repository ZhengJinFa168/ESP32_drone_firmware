#!/bin/bash
# scripts/flash.sh – Build, upload, and monitor

echo "🚀 Building firmware..."
pio run

echo "📦 Uploading to ESP32..."
pio run --target upload

echo "📡 Opening serial monitor..."
pio device monitor