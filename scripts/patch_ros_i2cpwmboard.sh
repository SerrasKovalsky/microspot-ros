#!/bin/bash
# Patch ros-i2cpwmboard so it compiles on Ubuntu ARM/Magni where libi2c may not link.
# Uses our own i2c_smbus_* via ioctl (no libi2c dependency).
# Run from repo root after: git submodule update --init --recursive

set -e
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SUBMOD="$REPO_ROOT/ros-i2cpwmboard"
CPP="$SUBMOD/src/i2cpwm_controller.cpp"
CMAKE="$SUBMOD/CMakeLists.txt"
STUB_C="$REPO_ROOT/scripts/i2c_smbus_ioctl.c"

if [ ! -f "$CPP" ] || [ ! -f "$CMAKE" ]; then
  echo "Not found: ros-i2cpwmboard (expected $REPO_ROOT/ros-i2cpwmboard/...)"
  echo "Run from repo root and: git submodule update --init --recursive"
  exit 1
fi

# 1) Use our ioctl-based i2c_smbus_* (no libi2c): add .c and declare in .cpp
if [ ! -f "$SUBMOD/src/i2c_smbus_ioctl.c" ]; then
  cp "$STUB_C" "$SUBMOD/src/i2c_smbus_ioctl.c"
  echo "Added: $SUBMOD/src/i2c_smbus_ioctl.c"
fi
# Remove i2c/smbus.h and add extern "C" declarations so .cpp uses our .c
if grep -q 'i2c/smbus.h' "$CPP"; then
  sed -i.bak '/#include <i2c\/smbus.h>/d' "$CPP"
  # After "extern \"C\" {" and "#include <linux/i2c-dev.h>", add declarations
  sed -i.bak2 '/#include <linux\/i2c-dev.h>/a\
extern int i2c_smbus_read_byte_data(int file, unsigned char command);\
extern int i2c_smbus_write_byte_data(int file, unsigned char command, unsigned char value);
' "$CPP"
  rm -f "${CPP}.bak" "${CPP}.bak2"
  echo "Patched: $CPP (use ioctl impl, removed i2c/smbus.h)"
fi

# 2) CMake: add i2c_smbus_ioctl.c to build and drop -li2c
if ! grep -q 'i2c_smbus_ioctl.c' "$CMAKE"; then
  sed -i.bak 's|add_executable(i2cpwm_board src/i2cpwm_controller.cpp)|add_executable(i2cpwm_board src/i2cpwm_controller.cpp src/i2c_smbus_ioctl.c)|' "$CMAKE"
  rm -f "${CMAKE}.bak"
  echo "Patched: $CMAKE (add i2c_smbus_ioctl.c)"
fi
# Remove libi2c from link (we provide our own impl)
if grep -q 'target_link_libraries(i2cpwm_board.*i2c\|libi2c.so' "$CMAKE"; then
  sed -i.bak 's| i2c)|)|' "$CMAKE"
  sed -i.bak2 's| /usr/lib/arm-linux-gnueabihf/libi2c.so)|)|' "$CMAKE"
  sed -i.bak3 's| /usr/lib/x86_64-linux-gnu/libi2c.so)|)|' "$CMAKE"
  rm -f "${CMAKE}.bak" "${CMAKE}.bak2" "${CMAKE}.bak3"
  echo "Patched: $CMAKE (removed -li2c)"
fi

echo "Done. Rebuild with: cd ~/catkin_ws && rm -rf build/microspot-ros/ros-i2cpwmboard && catkin_make"
