#!/bin/bash
# Patch ros-i2cpwmboard so it compiles on systems where i2c_smbus_* are
# declared in <i2c/smbus.h> (e.g. Ubuntu ARM, some Raspberry Pi images).
# Run from repo root after: git submodule update --init --recursive

set -e
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CPP="$REPO_ROOT/ros-i2cpwmboard/src/i2cpwm_controller.cpp"

if [ ! -f "$CPP" ]; then
  echo "Not found: $CPP"
  echo "Run from repo root and ensure submodule is updated: git submodule update --init --recursive"
  exit 1
fi

if grep -q 'i2c/smbus.h' "$CPP"; then
  echo "Already patched: $CPP"
  exit 0
fi

# Add #include <i2c/smbus.h> inside extern "C" { after <linux/i2c-dev.h>
sed -i.bak 's|#include <linux/i2c-dev.h>|#include <linux/i2c-dev.h>\n#include <i2c/smbus.h>|' "$CPP"
echo "Patched: $CPP (backup: ${CPP}.bak)"
rm -f "${CPP}.bak"
