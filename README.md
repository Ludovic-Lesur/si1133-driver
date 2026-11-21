# Description

This repository contains the **SI1133** light and UV index sensor driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **si1133-driver** | **embedded-utils** |
|:---:|:---:|
| [sw3.1](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw3.1) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |
| [sw3.0](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw3.0) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |
| [sw2.0](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw2.0) | >= [sw7.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw7.0) |
| [sw1.7](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.7) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.6](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.6) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.5](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.5) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.4](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.4) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.3](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.3) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.2](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.2) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.1](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.1) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.0](https://github.com/Ludovic-Lesur/si1133-driver/releases/tag/sw1.0) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `SI1133_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `si1133_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `SI1133_DRIVER_DISABLE` | `defined` / `undefined` | Disable the SI1133 driver. |
| `SI1133_DRIVER_I2C_ERROR_BASE_LAST` | `<value>` | Last error base of the low level I2C driver. |
| `SI1133_DRIVER_DELAY_ERROR_BASE_LAST` | `<value>` | Last error base of the low level delay driver. |
| `SI1133_DRIVER_CALIBRATION_MODE` | `0` / `1` / `2` | Sensor calibration mode: `0` = disabled, `1` = fixed, `2` = dynamic. |
| `SI1133_DRIVER_CALIBRATION_LUX_GAIN_NUMERATOR` | `<value>` | Light gain numerator (fixed calibration mode only). |
| `SI1133_DRIVER_CALIBRATION_LUX_GAIN_DENOMINATOR` | `<value>` | Light gain denominator (fixed calibration mode only). |
| `SI1133_DRIVER_CALIBRATION_UVI_GAIN_NUMERATOR` | `<value>` | UV index gain numerator (fixed calibration mode only). |
| `SI1133_DRIVER_CALIBRATION_UVI_GAIN_DENOMINATOR` | `<value>` | UV index gain denominator (fixed calibration mode only). |

# Build

A static library can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="<toolchain_file_path>" \
      -DTOOLCHAIN_PATH="<arm-none-eabi-gcc_path>" \
      -DTYPES_PATH="<types_file_path>" \
      -DEMBEDDED_UTILS_PATH="<embedded-utils_path>" \
      -DSI1133_DRIVER_I2C_ERROR_BASE_LAST=0 \
      -DSI1133_DRIVER_DELAY_ERROR_BASE_LAST=0 \
      -DSI1133_DRIVER_CALIBRATION_MODE=0 \
      -DSI1133_DRIVER_CALIBRATION_LUX_GAIN_NUMERATOR=1 \
      -DSI1133_DRIVER_CALIBRATION_LUX_GAIN_DENOMINATOR=1 \
      -DSI1133_DRIVER_CALIBRATION_UVI_GAIN_NUMERATOR=1 \
      -DSI1133_DRIVER_CALIBRATION_UVI_GAIN_DENOMINATOR=1 \
      -G "Unix Makefiles" ..
make all
```
