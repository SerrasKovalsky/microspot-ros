#!/bin/bash
# Patch ros-i2cpwmboard so it compiles and links on systems where i2c_smbus_*
# are in <i2c/smbus.h> and libi2c (e.g. Ubuntu ARM, Magni, some Raspberry Pi).
# Run from repo root after: git submodule update --init --recursive

set -e
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CPP="$REPO_ROOT/ros-i2cpwmboard/src/i2cpwm_controller.cpp"
CMAKE="$REPO_ROOT/ros-i2cpwmboard/CMakeLists.txt"

if [ ! -f "$CPP" ] || [ ! -f "$CMAKE" ]; then
  echo "Not found: ros-i2cpwmboard (expected $REPO_ROOT/ros-i2cpwmboard/...)"
  echo "Run from repo root and: git submodule update --init --recursive"
  exit 1
fi

# 1) Add #include <i2c/smbus.h> so i2c_smbus_* are declared
if ! grep -q 'i2c/smbus.h' "$CPP"; then
  sed -i.bak 's|#include <linux/i2c-dev.h>|#include <linux/i2c-dev.h>\n#include <i2c/smbus.h>|' "$CPP"
  rm -f "${CPP}.bak"
  echo "Patched: $CPP (added i2c/smbus.h)"
fi

# 2) Ensure CMake links libi2c: add multiarch link path and link by name (avoids find_library issues on ARM)
if ! grep -q 'link_directories.*arm-linux-gnueabihf\|link_directories.*CMAKE_LIBRARY_ARCHITECTURE' "$CMAKE"; then
  # Remove find_library block if present (revert to simple link by name)
  sed -i.bak '/find_library(I2C_LIB/,/endif()/d' "$CMAKE"
  sed -i.bak2 's| \${I2C_LIB})| i2c)|' "$CMAKE"
  # Add multiarch link path after existing link_directories so -li2c finds libi2c.so
  sed -i.bak3 '/link_directories(.*catkin_LIBRARY_DIRS)/a\
link_directories(/usr/lib/arm-linux-gnueabihf /usr/lib/x86_64-linux-gnu /usr/lib)
' "$CMAKE"
  rm -f "${CMAKE}.bak" "${CMAKE}.bak2" "${CMAKE}.bak3"
  echo "Patched: $CMAKE (link_directories + link i2c)"
fi

echo "Done. If link still fails, install: sudo apt-get install libi2c-dev"
