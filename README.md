# core2-pio-template
Template for the **CORE2** board to use with [Mbed OS 5.15 RTOS](https://os.mbed.com/docs/mbed-os/v5.15/introduction/index.html).

## Prerequisites
You need to install following tools:
* [Visual Studio Code IDE](https://code.visualstudio.com/)

### Required Visual Studio Code extensions
* [Microsoft C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) (`ms-vscode.cpptools`)
* [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) (`platformio.platformio-ide`)

## Speed up build process
https://docs.platformio.org/en/latest/frameworks/mbed.html#ignoring-particular-components

In directory `~/.platformio/packages/framework-mbed/features` create file called `.mbedignore` with the following content:

```
cellular/*
cryptocell/*
deprecated_warnings/*
lorawan/*
lwipstack/*
nanostack/*
netsocket/*
nfc/*
unsupported/*
```

## Build firmware
Use `PlatformIO: Build` task.

## Uploading firmware

### Uploading firmware using ST-Link

Attach ST-LINK programmer to the board and use `PlatformIO: Upload` task.

### Uploading firmware using `core2-flasher`
Connect **CORE2** to your computer via micro-usb port and use the `core2-flasher` tool (from this repo) to upload a generated `firmware.hex` file. You will find the `firmware.hex` file in the `.pio/build/core2` directory.

To flash firmware using `core2-flasher` run:
```bash
./core2-flasher .pio/build/core2/firmware.hex
```

## CORE2 pinout
This board pinout is described in `src/TARGET_CORE2/PinNames.h` file. Pin names defined in that file are similar to ones used by `hFramework` and should be easily identifiable. Peripheral functions available for each pin are described in `PeripheralPins.c` file.

![CORE2 PINOUT](https://husarion.com/docs/assets/img/core2-hardware/cheatsheet_small.jpg)