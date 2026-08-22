#!/bin/bash
# scripts/clean.sh – Clean build artifacts

echo "🧹 Cleaning..."
pio run --target clean
rm -rf .pio/build
echo "✅ Clean complete."