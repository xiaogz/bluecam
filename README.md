Build
-----
1. `sudo apt install libjpeg-turbo8-dev`; somehow, `sudo apt install
   libjpeg-dev` only installs documentation and a copyright text...?

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
      with the Uno R3 using the CSI interface. Also, the Amazon revies of the
      camera indicated that other people have had success making the camera on
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

References
----------
- my phone specs https://www.gsmarena.com/samsung_galaxy_a5_(2017)-8494.php
    - GPU supports OpenCL https://www.notebookcheck.net/ARM-Mali-T830-MP3.196668.0.html

