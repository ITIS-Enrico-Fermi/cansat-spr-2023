# Welcome to HASS Cansat team 2023 project

This project is made by ITIS Enrico Fermi students. Commits are released to prepare the final result for the Cansat Europe Competition 2023 edition.
The sony spresense overcomes every difficulty that stands in the way of it. Powered ARM® Cortex®-M4F x 6 cores (yes, thats a lot for IoT). That's why we chose Nuttx to support that deployment.

## Submodules

```
cansat-spr-2023            - This repository
|-- nuttx                  - NuttX original kernel + SPRESENSE port + HASS team custom drivers
|-- sdk
|   `-- apps               - NuttX original application + SPRESENSE port
`-- externals
    `-- nnablart
      `-- nnabla-c-runtime - Neural Network Runtime library
```

## Architecture

This firmware is intended to be run in a environment without human intervention.

The enabled and used sensors can be found in the [bringup program cxd56_bringup.c](nuttx/boards/arm/cxd56xx/spresense/src/cxd56_bringup.c)
that is run each time the system is turned on.

## Prerequisites

Install the necessary packages and GCC ARM toolchain for cross-compilation.
```
wget https://raw.githubusercontent.com/sonydevworld/spresense/master/install-tools.sh
bash install-tools.sh
```

> Zsh is not officialy supported. It doesn't work when using `spr-*` commands.

## Build

Go to the folder where you cloned the {SDK_FULL}, and enter the `sdk` folder name:
``` bash
cd spresense/sdk
```
Set up the SDK configuration
``` bash
tools/config.py examples/hello
```
Build the example image:
``` bash
make
```

To accelerate the build, use the `-j` option of Make eventually followed by the number of cores in your CPU:

```bash
make -j
```

otherwise Make will use the system CPU cores without any limitations; that corresponds to the processor count in `/proc/cpuinfo` on a Linux machine.

A `nuttx.spk` file appears in the `sdk` folder when this step has successfully finished.
This file is the final result and can be flashed into the your board.

## Using docker

A pre-compiled docker container is available with all the pre-requisite that is needed in order to build the Spresense SDK.

In order to start using it simply type:

```
source spresense_env.sh
```

This script will create an alias `spresense` which should proceed the regular SDK build scripts and Make commands.

Examples:
```
SpresenseSDK: $ spresense tools/config.py examples/hello
SpresenseSDK: $ spresense make
```
