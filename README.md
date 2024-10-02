# Description

This repository contains the **SI1133** UV index sensor driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **si1133-driver** | **embedded-utils** |
|:---:|:---:|
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
