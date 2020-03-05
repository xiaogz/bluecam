# BlueCam

## Hardware Requirements
1. [Arduino Uno R3 microcontroller.](https://www.amazon.ca/Elegoo-Board-ATmega328P-ATMEGA16U2-Arduino/dp/B01EWOE0UU/)
2. [ArduCAM OV2640 Mini 2MP Plus camera module.](https://www.amazon.ca/Arducam-Module-Megapixels-Arduino-Mega2560/dp/B012UXNDOY/)
3. [HC-05 Bluetooth module.](https://www.amazon.ca/DSD-TECH-HC-05-Pass-Through-Communication/dp/B01G9KSAF6/)
4. An Ubuntu (or Debian-based?) laptop with Bluetooth capabilities.

## Software Requirements
- `sudo apt install libjpeg-turbo8-dev libsdl2-dev libpthread-stubs0-dev`
    - somehow, `sudo apt install libjpeg-dev` only installs documentation and
      a copyright text...?
## Hardware Setup

## Software Setup

Build
-----
1. `sudo apt install libjpeg-turbo8-dev libsdl2-dev libpthread-stubs0-dev`;
    - somehow, `sudo apt install libjpeg-dev` only installs documentation and
      a copyright text...?

Goal
----
- challenge myself technically
- make a dashboard camera that can record video and read license plates within
  2-3 weeks during evening and weekend hours
- dashboard camera consists of:
    - Arduino Uno R3 (micro controller)
    - OV2640 camera (2 Megapixels)
    - HC-05 Bluetooth
    * I had time constraints and the Uno R3 + HC-05 was available from my
      brother-in-law at the time. I was looking into image processing at the
      time and wrote a preliminary Canny edge detection algorithm. It seemed
      natural to simply buy an Arduino camera to try and attempt image
      processing in real-time. The OV2640 was the only one that can interface
      with the Uno R3 using the CSI interface. Also, the Amazon reviews of the
      camera indicated that other people have had success using the camera on
      the Uno R3 despite its limited processing speed.
- image processing server consists of:
    - Samsung Galaxy A5 2017 phone (interfacing via Bluetooth)
        - CPU: Octa-core 1.9 GHz Cortex-A53 (ARM)
        - GPU: Mali-T830MP3 (OpenCL 1.2 support)
    - Android app with Bluetooth to communicate with Arduino camera
    - image processing library written in C++ and packaged using NDK
    * I implemented the Canny edge detection algorithm in C++17 on a gRPC
      server and wanted to re-use that. Also, I'm not as familiar with Java
      (nor Kotlin). The other reason for C++ and NDK usage is that I plan on
      re-writing the image processing algorithms leveraging GPGPU and from my
      limited research, my phone's GPU supports OpenCL 1.2 via C++ and NDK
      only.
- Bluetooth needs 3.5V input but Arduino outputs 5V so I needed to order
  resistors along with my camera

Outcome
-------
- did not make the Android image processing server component; opted for a C++ binary run from a Bluetooth compatible laptop
- image processing was very basic: no license plate recognition at all but I do have RGB to grayscale processing, Gaussian blurring, and Canny edge detection
- got more familiar with Arduino hardware setup and programming

