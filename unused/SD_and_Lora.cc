//
// Created by James Gallagher on 6/6/20.
// From: https://arduino.stackexchange.com/questions/69927/sd-card-and-lora-module-arduino-uno
// all of your problem can be solved through a simple trick ... Just follow it...
//
// Use Sdfat library insted of SD
// just follow SPI datasheet..
// follow below code ... it works with LORA AND SD both together

#include <SdFat.h>
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

SdFat sd;
SdFile myFile;

#define chipSelect 4

#define RFM95_CS 10 //
#define RFM95_RST 9
#define RFM95_INT 2

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS,RFM95_INT);
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(rf95, 2);

void setup() {

    Serial.begin(9600);
    while (!Serial) {}  // wait for Leonardo

    delay(100);

    // Initialize SdFat or print a detailed error message and halt
    // Use half speed like the native library.
    // change to SPI_FULL_SPEED for more performance.
    pinMode(RFM95_CS, OUTPUT);
    digitalWrite(RFM95_CS, HIGH);
    digitalWrite(4, LOW);

    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();

/*

/*
 * //Open file and write  ...
 *
  // open the file for write at end like the Native SD library
  if (!myFile.open("mrinmoy.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }


  // if the file opened okay, write to it:
  Serial.print("Writing to test.txt...");
  myFile.println("testing 1, 2, 3.");


  // close the file:
  myFile.close();
  Serial.println("done.");
  */

    /*
    // open the file for reading....

    if (!myFile.open("mrinmoy.txt", O_READ))
    {
      sd.errorHalt("opening test.txt for read failed");
    }

    Serial.println("mrinmoy.txt:");


    // read from the file until there's nothing else in it:
    int data;

    while ((data = myFile.read()) >= 0) Serial.write(data);
    // close the file:
    myFile.close();
    */

    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    digitalWrite(RFM95_CS, LOW);
    digitalWrite(4,HIGH);

    while (!manager.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    // digitalWrite(RFM95_CS, LOW);

}

void loop()
{
    uint8_t data[] = " Hi.. This is node 1.";
    //   unsigned char sd_data[50];
// Dont put this on the stack:
    // uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t buf[50];
    // nothing happens after setup

    //LoRa.setPins( LoRa_CS, LoRa_RST, DI0 );
    digitalWrite(4,HIGH);    //  deselet SD Card SPI
    digitalWrite(RFM95_CS, LOW);    //  SELECT (low) LoRa SPI
    SPI.begin();

    // Send a message to manager_client
    if (manager.sendtoWait((uint8_t*)data,sizeof(data),1))
    {
        Serial.print("successfully sent to: ");
        Serial.println(1);
    }
    else
    {
        Serial.println("send failed");
    }

    // receiver ....

    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;

    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
        Serial.print("Got Response From : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
    }
    else
    {

    }
    //   digitalWrite(4, LOW);
    SPI.end();

    digitalWrite(4, LOW);    //  deselet SD Card SPI
    digitalWrite(RFM95_CS,HIGH);    //  SELECT (low) LoRa SPI
    SPI.begin();


    if (!myFile.open("mrinmoy.txt", O_READ))
    {
        sd.errorHalt("opening test.txt for read failed");
    }

    Serial.println("mrinmoy.txt:");
    char ch;
    String str;
    int i=0;
    uint8_t sd_data[50];

    while (myFile.available()>0)
    {
        ch= myFile.read();
        str+=ch;
    }

    String str2=str;
    Serial.println(str2);
    Serial.println(str2.length());


    myFile.close();
    SPI.end();


    delay(500);
}