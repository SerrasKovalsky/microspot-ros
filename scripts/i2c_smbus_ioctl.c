/*
 * Minimal i2c_smbus_* implementation via I2C_SMBUS ioctl.
 * Use this when libi2c is not available or does not link (e.g. Ubuntu ARM/Magni).
 * Self-contained: defines types/constants if not in system headers.
 */
#include <sys/ioctl.h>
#include <unistd.h>

/* linux/i2c-dev.h provides I2C_SMBUS and struct i2c_smbus_ioctl_data */
#include <linux/i2c-dev.h>

/* Types/constants sometimes live in <i2c/smbus.h> or <linux/i2c.h>; define if missing */
#ifndef I2C_SMBUS_READ
# define I2C_SMBUS_READ  1
#endif
#ifndef I2C_SMBUS_WRITE
# define I2C_SMBUS_WRITE 0
#endif
#ifndef I2C_SMBUS_BYTE_DATA
# define I2C_SMBUS_BYTE_DATA 2
#endif
#ifndef _I2C_SMBUS_DATA_DEFINED
# define _I2C_SMBUS_DATA_DEFINED
union i2c_smbus_data {
  unsigned char byte;
  unsigned short word;
  unsigned char block[34];
};
#endif

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
