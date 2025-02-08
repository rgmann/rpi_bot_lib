# Welcome to `rpi_bot_lib`!

This repository provides a collection of wrappers for protocols and devices that are useful when building Raspberry PI based robots. You can build and install this project as a shared library or directly include it into another project as a submodule.

## Dependencies

This project tries to minimize external dependencies as much as possible. You just need to install [meson](https://mesonbuild.com/Quick-guide.html).

## Getting Started

The build generates a shared object file, which you can then link other projects against.

 1. `meson setup build`
 1. `ninja -C build`

## Available Wrappers

`using namespace rpi_bot_lib;`

This project includes the following interfaces and device wrappers.

### `I2cInterface`

`#include "i2c_interface.h"`

This class wraps the Raspberry PI I2C interace, making it easy to acquire, read, and write to the bus. I2cInterface follows the singleton pattern, so you must use the supplied factory function to create/access the interface. The interface is **not** thread safe, so you must ensure that externally if you intend to access the interface across multiple thread contexts.

### `Adxl345Controller`

`#include "adxl345_controller.h"`

This class wraps the ADXL345 three-access accelerometer peripheral (see [spec sheet](docs/adxl345_datasheet.pdf)).

