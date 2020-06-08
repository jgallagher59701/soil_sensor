
#include "blink.h"

#define SOIL_SENSOR_ADDR 0x36

void soil_moisture_setup() {
    // Initialize the soil sensor
    new_blink_times(STATUS, SOIL_MOISTURE_STATUS, START);

    if (!soil_moisture.begin(SOIL_SENSOR_ADDR)) {
        IO(Serial.println(F("ERROR! seesaw not found")));
        new_blink_times(STATUS, SOIL_MOISTURE_STATUS, ERROR_OCCURRED);
    } else {
        IO(Serial.print(F("seesaw started! version: ")));
        IO(Serial.println(soil_moisture.getVersion(), HEX));
    }

    new_blink_times(STATUS, SOIL_MOISTURE_STATUS, COMPLETED);
}

// Return the temperature of the soil moisture sensor. Not very precise (+/- 1C)
float get_soil_moisture_temp() {
    return soil_moisture.getTemp();
}

// The value of the seesaw capacitive touch pin. An integer between 0 and 1023
// This waits for the sensor to stabilize.
uint16_t get_soil_moisture_value() {
    uint16_t cap_value = 0xFFFF;
    uint16_t new_cap_value = soil_moisture.touchRead(0);

    while (cap_value != new_cap_value) {
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
        cap_value = new_cap_value;
        new_cap_value = soil_moisture.touchRead(0);
    }

    return cap_value;
}

