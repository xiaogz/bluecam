// ArduCAM Mini demo (C)2018 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Mini camera, and can run on any Arduino platform.
// This demo was made for ArduCAM_Mini_2MP_Plus.
// It needs to be used in combination with PC software.
// It can take photo continuously as video streaming.
//
// The demo sketch will do the following tasks:
// 1. Set the camera to JPEG output g_mode.
// 2. Read data from Serial port and deal with it
// 3. If receive 0x00-0x08,the resolution will be changed.
// 4. If receive 0x10,camera will capture a JPEG photo and buffer the image to FIFO.Then write datas
// to Serial port.
// 5. If receive 0x20,camera will capture JPEG photo and write datas continuously.Stop when receive
// 0x21.
// 6. If receive 0x30,camera will capture a BMP  photo and buffer the image to FIFO.Then write datas
// to Serial port.
// 7. If receive 0x11 ,set camera to JPEG output g_mode.
// 8. If receive 0x31 ,set camera to BMP  output g_mode.
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_2MP_Plus
// and use Arduino IDE 1.6.8 compiler or above
#include "memorysaver.h"
#include <ArduCAM.h>
#include <SPI.h>
#include <Wire.h>
// This demo can only work on OV2640_MINI_2MP_PLUS platform.

#if !(defined OV2640_MINI_2MP_PLUS)
 #error Please select the correct hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif

#define BMPIMAGEOFFSET 66
const unsigned char bmp_header[BMPIMAGEOFFSET] PROGMEM = {
    0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00,
    0x28, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x10, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00,
    0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8,
    0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00
};
// set pin 7 as the slave select for the digital port:
const int g_CS = 7;
static int g_mode = 0;
uint8_t start_capture = 0;
#if defined(OV2640_MINI_2MP_PLUS)
ArduCAM myCAM(OV2640, g_CS);
#endif
uint8_t read_fifo_burst(ArduCAM myCAM);

void setup()
{
    // debug
    pinMode(LED_BUILTIN, OUTPUT);

    // put your setup code here, to run once:
    uint8_t vid;
    uint8_t pid;
    uint8_t temp;

    Wire.begin();
    // TODO: increase this rate and see if bluetooth module works
    //Serial.begin(460800);
    Serial.begin(230400);
    //Serial.begin(115200);
    // it's confirmed that bluetooth works on 230400 but not on 460800
    // TODO: see if changing the resistors on the voltage divider would allow
    // higher baud rate to work

    //Serial.println(F("ACK CMD ArduCAM Start! END"));
    // set the g_CS as an output:
    pinMode(g_CS, OUTPUT);
    digitalWrite(g_CS, HIGH);
    // initialize SPI:
    SPI.begin();
    // Reset the CPLD
    myCAM.write_reg(0x07, 0x80);
    delay(100);
    myCAM.write_reg(0x07, 0x00);
    delay(100);

    while (1) {
        // Check if the ArduCAM SPI bus is OK
        myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
        temp = myCAM.read_reg(ARDUCHIP_TEST1);
        if (temp != 0x55) {
            //Serial.println(F("ACK CMD SPI interface Error! END"));
            delay(1000);
            continue;
        }
        else {
            //Serial.println(F("ACK CMD SPI interface OK. END"));
            break;
        }
    }

#if defined(OV2640_MINI_2MP_PLUS)
    while (1) {
        // Check if the camera module type is OV2640
        myCAM.wrSensorReg8_8(0xff, 0x01);
        myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
        myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
        if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) {
            Serial.println(F("ACK CMD Can't find OV2640 module!"));
            delay(1000);
            continue;
        }
        else {
            //Serial.println(F("ACK CMD OV2640 detected. END"));
            break;
        }
    }
#endif
    // Change to JPEG capture g_mode and initialize the OV5642 module
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
#if defined(OV2640_MINI_2MP_PLUS)
    myCAM.OV2640_set_JPEG_size(OV2640_320x240);
#endif
    delay(1000);
    myCAM.clear_fifo_flag();
}

void loop()
{
    //delay(1000);
    //Serial.println(F("ABCD"));
    // these are used for video streaming mode only and duplicates code of
    // read_fifo_burst()
    uint8_t temp = 0xff;
    uint8_t temp_last = 0;
    bool hasEncounteredJPEGHeader = false;

    if (Serial.available()) {
        temp = Serial.read();
        switch (temp) {
        case 0x61: // 'a'
            g_mode = 1;
            temp = 0xff;
            start_capture = 1;
            //Serial.println(F("JPG snap"));
            break;
        case 0x72: // 'r'
            g_mode = 2;
            temp = 0xff;
            start_capture = 2;
            //Serial.println(F("ACK CMD CAM start video streaming. END"));
            break;
        default:
            break;
        }
    }

    if (g_mode == 1) {
        if (start_capture == 1) {
            myCAM.flush_fifo();
            myCAM.clear_fifo_flag();
            // Start capture
            myCAM.start_capture();
            start_capture = 0;
        }
        if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
            //Serial.println(F("ACK CMD CAM Capture Done. END"));
            delay(50);
            read_fifo_burst(myCAM);
            // Clear the capture done flag
            myCAM.clear_fifo_flag();
            //g_mode = 0;
        }
    }
    else if (g_mode == 2) { // video streaming loop
        while (1) {
            // check for user-issued stop command
            temp = Serial.read();
            if (temp == 0x78) { // 'x'
                start_capture = 0;
                g_mode = 0;
                //Serial.println(F("ACK CMD CAM stop video streaming. END"));
                break;
            }

            // Start capture
            if (start_capture == 2) {
                myCAM.flush_fifo();
                myCAM.clear_fifo_flag();
                myCAM.start_capture();
                start_capture = 0;
            }
            if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
                uint32_t length = 0;
                length = myCAM.read_fifo_length();
                if ((length >= MAX_FIFO_SIZE) | (length == 0)) {
                    myCAM.clear_fifo_flag();
                    start_capture = 2;
                    continue;
                }
                myCAM.CS_LOW();
                myCAM.set_fifo_burst(); // Set fifo burst g_mode
                temp = SPI.transfer(0x00);
                --length;
                while (length--) {
                    temp_last = temp;
                    temp = SPI.transfer(0x00);
                    if (hasEncounteredJPEGHeader == true) {
                        Serial.write(temp);
                    }
                    else if ((temp == 0xD8) & (temp_last == 0xFF)) {
                        hasEncounteredJPEGHeader = true;
                        //Serial.println(F("ACK CMD IMG END"));
                        Serial.write(temp_last);
                        Serial.write(temp);
                    }
                    if ((temp == 0xD9) && (temp_last == 0xFF)) {
                        break;
                    }
                    //delayMicroseconds(15);
                    delayMicroseconds(3);
                }
                myCAM.CS_HIGH();
                myCAM.clear_fifo_flag();
                start_capture = 2;
                hasEncounteredJPEGHeader = false;
            }
        }
    }

    /*
    else if (g_mode == 0) {
        Serial.end();
    }
    */
/*
    uint8_t temp = 0xff;
    uint8_t temp_last = 0;
    bool hasEncounteredJPEGHeader = false;

    if (Serial.available()) {
        temp = Serial.read();
        switch (temp) {
        case 0:
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_160x120);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_160x120 END"));
#endif
            temp = 0xff;
            break;
        case 1:
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_176x144);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_176x144 END"));
#endif
            temp = 0xff;
            break;
        case 2:
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_320x240);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_320x240 END"));
#endif
            temp = 0xff;
            break;
        case 3:
            temp = 0xff;
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_352x288);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_352x288 END"));
#endif
            break;
        case 4:
            temp = 0xff;
#if defined(OV2640_MINI_2MP)
            myCAM.OV2640_set_JPEG_size(OV2640_640x480);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_640x480 END"));
#endif
            break;
        case 5:
            temp = 0xff;
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_800x600);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_800x600 END"));
#endif
            break;
        case 6:
            temp = 0xff;
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_1024x768);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_1024x768 END"));
#endif
            break;
        case 7:
            temp = 0xff;
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_1280x1024);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_1280x1024 END"));
#endif
            break;
        case 8:
            temp = 0xff;
#if defined(OV2640_MINI_2MP_PLUS)
            myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
            delay(1000);
            Serial.println(F("ACK CMD switch to OV2640_1600x1200 END"));
#endif
            break;
        case 0x10:
            g_mode = 1;
            temp = 0xff;
            start_capture = 1;
            Serial.println(F("ACK CMD CAM start single shoot. END"));
            break;
        case 0x11:
            temp = 0xff;
            Serial.println(F("ACK CMD Change OK. END"));
            myCAM.set_format(JPEG);
            myCAM.InitCAM();
            myCAM.OV2640_set_JPEG_size(OV2640_320x240);
            break;
        case 0x20:
            g_mode = 2;
            temp = 0xff;
            start_capture = 2;
            Serial.println(F("ACK CMD CAM start video streaming. END"));
            break;
        case 0x30:
            g_mode = 3;
            temp = 0xff;
            start_capture = 3;
            Serial.println(F("ACK CMD CAM start single shoot. END"));
            break;
        case 0x31:
            temp = 0xff;
            myCAM.set_format(BMP);
            myCAM.InitCAM();
            break;
        case 0x30:
            g_mode = 3;
            temp = 0xff;
            start_capture = 3;
            Serial.println(F("ACK CMD CAM start single shoot. END"));
            break;
        default:
            break;
        }
    }

    if (g_mode == 1) {
        if (start_capture == 1) {
            myCAM.flush_fifo();
            myCAM.clear_fifo_flag();
            // Start capture
            myCAM.start_capture();
            start_capture = 0;
        }
        if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
            Serial.println(F("ACK CMD CAM Capture Done. END"));
            delay(50);
            read_fifo_burst(myCAM);
            // Clear the capture done flag
            myCAM.clear_fifo_flag();
        }
    }
    else if (g_mode == 2) {
        while (1) {
            temp = Serial.read();
            if (temp == 0x21) {
                start_capture = 0;
                g_mode = 0;
                Serial.println(F("ACK CMD CAM stop video streaming. END"));
                break;
            }
            if (start_capture == 2) {
                myCAM.flush_fifo();
                myCAM.clear_fifo_flag();
                // Start capture
                myCAM.start_capture();
                start_capture = 0;
            }
            if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
                uint32_t length = 0;
                length = myCAM.read_fifo_length();
                if ((length >= MAX_FIFO_SIZE) | (length == 0)) {
                    myCAM.clear_fifo_flag();
                    start_capture = 2;
                    continue;
                }
                myCAM.CS_LOW();
                myCAM.set_fifo_burst(); // Set fifo burst g_mode
                temp = SPI.transfer(0x00);
                length--;
                while (length--) {
                    temp_last = temp;
                    temp = SPI.transfer(0x00);
                    if (hasEncounteredJPEGHeader == true) {
                        Serial.write(temp);
                    }
                    else if ((temp == 0xD8) & (temp_last == 0xFF)) {
                        hasEncounteredJPEGHeader = true;
                        Serial.println(F("ACK CMD IMG END"));
                        Serial.write(temp_last);
                        Serial.write(temp);
                    }
                    if ((temp == 0xD9) && (temp_last == 0xFF)) // If find the end ,break while,
                        break;
                    delayMicroseconds(15);
                }
                myCAM.CS_HIGH();
                myCAM.clear_fifo_flag();
                start_capture = 2;
                hasEncounteredJPEGHeader = false;
            }
        }
    }
    else if (g_mode == 3) {
        if (start_capture == 3) {
            // Flush the FIFO
            myCAM.flush_fifo();
            myCAM.clear_fifo_flag();
            // Start capture
            myCAM.start_capture();
            start_capture = 0;
        }
        if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
            Serial.println(F("ACK CMD CAM Capture Done. END"));
            delay(50);
            uint8_t temp;
            uint32_t length = 0;
            length = myCAM.read_fifo_length();
            if (length >= MAX_FIFO_SIZE) {
                Serial.println(F("ACK CMD Over size. END"));
                myCAM.clear_fifo_flag();
                return;
            }
            if (length == 0) // 0 kb
            {
                Serial.println(F("ACK CMD Size is 0. END"));
                myCAM.clear_fifo_flag();
                return;
            }
            myCAM.CS_LOW();
            myCAM.set_fifo_burst(); // Set fifo burst g_mode

            Serial.write(0xFF);
            Serial.write(0xAA);
            for (temp = 0; temp < BMPIMAGEOFFSET; temp++) {
                Serial.write(pgm_read_byte(&bmp_header[temp]));
            }
            char VH, VL;
            int i = 0, j = 0;
            for (i = 0; i < 240; i++) {
                for (j = 0; j < 320; j++) {
                    VH = SPI.transfer(0x00);
                    ;
                    VL = SPI.transfer(0x00);
                    ;
                    Serial.write(VL);
                    delayMicroseconds(12);
                    Serial.write(VH);
                    delayMicroseconds(12);
                }
            }
            Serial.write(0xBB);
            Serial.write(0xCC);
            myCAM.CS_HIGH();
            // Clear the capture done flag
            myCAM.clear_fifo_flag();
            g_mode = 0; // clears mode flag
        }
    }
*/
}

// this function is only called for single-capture mode
uint8_t read_fifo_burst(ArduCAM myCAM)
{
    bool hasEncounteredJPEGHeader = false;
    uint8_t temp = 0;
    uint8_t temp_last = 0;
    uint32_t length = myCAM.read_fifo_length();

    //Serial.println(length, DEC);
    if (length >= MAX_FIFO_SIZE) // 512 kb
    {
        //Serial.println(F("ACK CMD Over size. END"));
        return 0;
    }
    if (length == 0) // 0 kb
    {
        //Serial.println(F("ACK CMD Size is 0. END"));
        return 0;
    }

    myCAM.CS_LOW();
    myCAM.set_fifo_burst(); // Set fifo burst g_mode
    temp = SPI.transfer(0x00);
    length--;
    while (length--) {
        //digitalWrite(LED_BUILTIN, HIGH);

        temp_last = temp;
        temp = SPI.transfer(0x00);
        if (hasEncounteredJPEGHeader == true) {
            Serial.write(temp);
        }
        else if ((temp == 0xD8) & (temp_last == 0xFF)) {
            hasEncounteredJPEGHeader = true;
            //Serial.println(F("ACK CMD IMG END"));
            Serial.write(temp_last);
            Serial.write(temp);
        }

        if ((temp == 0xD9) && (temp_last == 0xFF)) // If find the end ,break while,
            break;
        delayMicroseconds(15);

        //digitalWrite(LED_BUILTIN, LOW);
    }

    myCAM.CS_HIGH();
    hasEncounteredJPEGHeader = false;
    return 1;
}
