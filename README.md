# Welcome to HASS Cansat team 2023 project

This project is made by ITIS Enrico Fermi students. Commits are released to prepare the final result for the Cansat Europe Competition 2023 edition.
The sony spresense overcomes every difficulty that stands in the way of it. Powered ARM® Cortex®-M4F x 6 cores (yes, thats a lot for IoT). That's why we chose Nuttx to support that deployment.

# Guide

In the hass-team folder, you can find the "user-space" applications that have been written by the HASS team. These applications work similarly to the ones you might write with an Arduino.

The operating system used in this project is NUTTX, which is a real-time operating system (RTOS) that provides a rich set of features for embedded systems. NUTTX is POSIX compliant, which means it adheres to the Portable Operating System Interface (POSIX) standard, which is a set of guidelines that defines a common API for Unix-like operating systems.

In the kernel of NUTTX, we have developed a driver for the RFM95 module, which is our LoRa module, using SPI. This driver is a character driver that communicates with the character device, such as a serial port or a terminal. The POSIX standard defines the interface for character drivers, which includes the following methods: register(), open(), read(), and write(). These methods allow user-space applications to interact with the low-level hardware, such as the RFM95 module, through the kernel.

We have also written the "flight" application in the user-space, which refers to the code that was flashed before the launch. The "flight" application uses the POSIX standard driver methods to interact with peripherals such as RFM95 module, bmp280 barometer, mpu6050, gyroscope/accelerometer, veml6070 uv sensor, isx012 camera.. and collect data during the flight.
Contact

If you have any questions, please feel free to contact Lorenzo Borghi at info.borghilorenzo@gmail.com.

# Submodules

```
spresense                  - This repository
|-- nuttx                  - NuttX original kernel + SPRESENSE port
|-- sdk
|   `-- apps               - NuttX original application + SPRESENSE port
`-- externals
    `-- nnablart
      `-- nnabla-c-runtime - Neural Network Runtime library
```

## Update SDK2.x.x to SDK3.x.x

The URL of the submodule (nuttx, sdk/apps) in the spresense repository has been changed since SDK3.0.0. If you cloned the repository before SDK2.x, please run the following instructions to update the URL of the submodule.

```
$ cd spresense
$ git fetch origin
$ git checkout <remote branch>
$ git submodule sync
$ git submodule update
```
* `<remote branch>`
  * For master branch: `origin/master`
  * For develop branch: `origin/develop`

# Spresense SDK build instructions

Build instructions are documented at [Spresense SDK Getting Started Guide](https://developer.sony.com/develop/spresense/docs/sdk_set_up_en.html).

## Prerequisites

Install the necessary packages and GCC ARM toolchain for cross-compilation.
```
$ wget https://raw.githubusercontent.com/sonydevworld/spresense/master/install-tools.sh
$ bash install-tools.sh
```

## Build

Go to the folder where you cloned the {SDK_FULL}, and enter the `sdk` folder name:
``` bash
$ cd spresense/sdk
```
Set up the SDK configuration
``` bash
$ tools/config.py examples/hello
```
Build the example image:
``` bash
$ make
```

A `nuttx.spk` file appears in the `sdk` folder when this step has successfully finished.
This file is the final result and can be flashed into the your board.

# Using docker

A pre-compiled docker container is available with all the pre-requisite that is needed in order to build the Spresense SDK.

In order to start using it simply type:

```
$ source spresense_env.sh
```

This script will create an alias `spresense` which should proceed the regular SDK build scripts and Make commands.

Examples:
```
SpresenseSDK: $ spresense tools/config.py examples/hello
SpresenseSDK: $ spresense make
```

[twitter spresense handle]: https://img.shields.io/twitter/follow/SpresensebySony?style=social&logo=twitter
[twitter spresense badge]: https://twitter.com/intent/follow?screen_name=SpresensebySony
[twitter devworld handle]: https://img.shields.io/twitter/follow/SonyDevWorld?style=social&logo=twitter
[twitter devworld badge]: https://twitter.com/intent/follow?screen_name=SonyDevWorld
