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

# 2) Ensure CMake finds and links libi2c (fixes undefined reference at link)
if ! grep -q 'find_library.*I2C_LIB' "$CMAKE"; then
  sed -i.bak '/^link_directories(/a\
find_library(I2C_LIB NAMES i2c PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} /usr/lib)\
if(NOT I2C_LIB)\
  message(FATAL_ERROR "libi2c not found. Install: sudo apt-get install libi2c-dev")\
endif()
' "$CMAKE"
  sed -i.bak2 's| i2c)| ${I2C_LIB})|' "$CMAKE"
  rm -f "${CMAKE}.bak" "${CMAKE}.bak2"
  echo "Patched: $CMAKE (find_library + link libi2c)"
elif ! grep -q 'PATHS.*CMAKE_LIBRARY_ARCHITECTURE' "$CMAKE"; then
  # Already patched but without PATHS; add multiarch path so ARM finds libi2c
  sed -i.bak 's|find_library(I2C_LIB NAMES i2c)|find_library(I2C_LIB NAMES i2c PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} /usr/lib)|' "$CMAKE"
  rm -f "${CMAKE}.bak"
  echo "Patched: $CMAKE (added PATHS for multiarch libi2c)"
fi

echo "Done. If link still fails, install: sudo apt-get install libi2c-dev"
