/*
 * Minimal i2c_smbus_* implementation via I2C_SMBUS ioctl.
 * Use this when libi2c is not available or does not link (e.g. Ubuntu ARM/Magni).
 * Requires only <linux/i2c-dev.h> and <sys/ioctl.h>.
 */
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

int i2c_smbus_read_byte_data(int file, unsigned char command)
{
  union i2c_smbus_data data;
  struct i2c_smbus_ioctl_data args;
  args.read_write = I2C_SMBUS_READ;
  args.command = command;
  args.size = I2C_SMBUS_BYTE_DATA;
  args.data = &data;
  if (ioctl(file, I2C_SMBUS, &args) < 0)
    return -1;
  return 0x0FF & data.byte;
}

int i2c_smbus_write_byte_data(int file, unsigned char command, unsigned char value)
{
  union i2c_smbus_data data;
  struct i2c_smbus_ioctl_data args;
  data.byte = value;
  args.read_write = I2C_SMBUS_WRITE;
  args.command = command;
  args.size = I2C_SMBUS_BYTE_DATA;
  args.data = &data;
  return ioctl(file, I2C_SMBUS, &args);
}
