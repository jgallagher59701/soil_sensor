
# Soil Sensor

This repository holds software for a simple sensor network that can
functionin remote and harsh envroments while still providing a near
real-time data feed.  The sensors measure basic parameters of soil:
tempterature and moisture.

The software (and hardware designs) here are for the sensor (leaf)
nodes in a start topology network. These sensor nods use LoRa to
communicate with a master node.

## Building

This software can be built using platformio, both the command line and
CLion IDE are supported; using another IDE should be simple. It is
based on the Arduino pro mini.

## Dependencies

LowPower - Low-Power_ID38 - platformio lib install 38
RTCLibExtended - RTClibExtended_ID6170 - ... install 6170
SeeSaw - Adafruit seesaw Library_ID1890	- ... install 1890
RadioHead

## Keywords

Lora, Satellite modem, Arduino, pro8MHzatmega328
