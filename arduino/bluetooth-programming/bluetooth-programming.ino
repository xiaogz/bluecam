// Basic Bluetooth sketch HC-05_01
// Sends "Bluetooth Test" to the serial monitor and the software serial once every second.
//
// Connect the HC-05 module and data over Bluetooth
//
// The HC-05 defaults to commincation mode when first powered on.
// The default baud rate for communication is 9600
// Taken from http://www.martyncurrey.com/arduino-with-hc-05-bluetooth-module-in-slave-mode/

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(2, 3); // RX | TX
// Connect the HC-05 TX to Arduino pin 2 RX.
// Connect the HC-05 RX to Arduino pin 3 TX through a voltage divider.

char c = ' ';

void setup()
{
    Serial.begin(9600);
    Serial.println("Enter AT commands:");

    // HC-05 default serial speed for communcation mode is 9600
    BTSerial.begin(38400);
    // TODO: verify that the AT mode baudrate either stays at 38400 or becomes whatever baudrate is configured
}

void loop()
{
    // Keep reading from HC-05 and send to Arduino Serial Monitor
    if (BTSerial.available())
        Serial.write(BTSerial.read());

    // Keep reading from Arduino Serial Monitor and send to HC-05
    if (Serial.available())
        BTSerial.write(Serial.read());
}
