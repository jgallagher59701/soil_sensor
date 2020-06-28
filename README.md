
# Soil Sensor

This repository holds software for a simple sensor network that can
functionin remote and harsh envroments while still providing a near
real-time data feed.  The sensors measure basic parameters of soil:
tempterature and moisture.

The software (and hardware designs) here are for the sensor (leaf)
nodes in a start topology network. These sensor nods use LoRa to
communicate with a master node.

## Cloning

This uses a fork of the RTCLibExtended code from Fabio Cuomo, included
in the 'lib' directory as a git submodule. Clone the code using:

  git clone --recursive https://github.com/jgallagher59701/soil_sensor

If you clone without '--recursive' then use:

  git submodule update --init

After a plain clone.

Then, init the _platformio_ environment using:

  pio init --ide clion

## Building

This software can be built using platformio, both the command line and
CLion IDE are supported; using another IDE should be simple. The software
uses the the Arduino pro mini.

Build using the CLion ide or on the command line using:

`pio run`

## Dependencies

* LowPower - Low-Power_ID38 - `platformio lib install 38` or `pio lib install "Low-Power"`
* RTClibExtended_ID6170 - `... install 6170` _But see the code in 'lib' here. Use that instead._
* SeeSaw - Adafruit seesaw Library_ID1890	- `pio lib install "Adafruit seesaw Library"`
* RadioHead - RadioHead_ID124 - `pio lib install "RadioHead"`
* SDFat - SD card library - `pio lib install "SdFat"`

* RadioHead documentation: 
* https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html, 
* https://www.airspayce.com/mikem/arduino/RadioHead/classRHReliableDatagram.html

## Notes about LoRa and SDFat
### From: https://github.com/greiman/SdFat/issues/92

You must use [SPI transactions](https://www.arduino.cc/en/Tutorial/SPITransaction) 
to use SPI in interrupts routines.

With the new SPI library, configure each SPI device once as an SPISettings object.
Also, if that device will be called from an interrupt, say so with 
SPI.usingInterrupt(interruptNumber). To communicate with a specific SPI device, 
use SPI.beginTransaction which automatically uses the settings you declared for 
that device. In addition, it will disable any interrupts that use SPI for the 
duration of the transaction. Once you are finished, use SPI.endTransaction()
which re-enables any SPI-using interrupts.

This will block the radio driver but allow interrupts to occur for millis and 
drivers that don't use SPI.

Fix the radio driver with [usingInterrupt](https://www.arduino.cc/en/Reference/SPIusingInterrupt)!

Edit:
[Here](https://github.com/PaulStoffregen/RadioHead) is an example radio driver with a fix. 
Don't know if it would work in your case.

## PlatformIO information

https://docs.platformio.org/en/latest/ide/vscode.html#quick-start

## Keywords

Lora, Satellite modem, Arduino, pro8MHzatmega328
