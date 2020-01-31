
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
CLion IDE are supported; using another IDE should be simple. It is
based on the Arduino pro mini.

Build using the CLion ide or on the command line using:

  pio run

## Dependencies

LowPower - Low-Power_ID38 - platformio lib install 38
SeeSaw - Adafruit seesaw Library_ID1890	- ... install 1890
RadioHead - RadioHead_ID124 - ... install 124

## Keywords

Lora, Satellite modem, Arduino, pro8MHzatmega328
