// taken from https://stackoverflow.com/a/38318768

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <jpeglib.h>
#include <memory>
#include <termios.h>
#include <unistd.h>

#include "utility.hpp"

// we are working with JPEG 320×240
static constexpr const size_t k1KB = 1024;
static constexpr size_t kJpgBufferSize = 512 * k1KB;
static constexpr size_t kRawImgBufferSize = 2 * 1024 * k1KB;
static constexpr size_t kCommandLength = 3; // letter + \r\n
static constexpr size_t kMaxTimeoutCount = 7;
static constexpr size_t kSerialBufferSize = 100;

int set_interface_attribs(int serialPort, int speed)
{
    struct termios tty;

    if (::tcgetattr(serialPort, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    ::cfsetospeed(&tty, (speed_t)speed);
    ::cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    // this needs -std=gnu11
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    // see https://stackoverflow.com/a/38714644 and
    // https://stackoverflow.com/a/11513102 for why program will hang if VMIN is set to 1
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (::tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", ::strerror(errno));
        return -1;
    }
    return 0;
}

bool DecompressJpegImage(const uint8_t * imageData, const uint32_t dataSize)
{
    bool retval = true;

    struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, imageData, dataSize);

	if (jpeg_read_header(&cinfo, TRUE) != 1) {
        retval = false;
    }

    if (retval) {
        jpeg_start_decompress(&cinfo);

        const uint32_t width = cinfo.output_width;
        const uint32_t height = cinfo.output_height;
        const uint32_t colorBitFactor = cinfo.output_components;
        const uint32_t rowStride = width * colorBitFactor;

        printf("Image is %u by %u with %u components\n", width, height, colorBitFactor);

        uint64_t rawImageSize = width * height * colorBitFactor;

        auto rawImage = std::make_unique<uint8_t[]>(rawImageSize);

        uint8_t * bufferArray[1];
        while (cinfo.output_scanline < cinfo.output_height) {
            bufferArray[0] = rawImage.get() + cinfo.output_scanline * rowStride;
            jpeg_read_scanlines(&cinfo, bufferArray, 1);
        }

        jpeg_finish_decompress(&cinfo);

        printf("jpeg decompression finished\n");

        if (!Util::WritePPMImageToFile(
                "output.ppm", rawImage.get(), width, height, colorBitFactor)) {
            retval = false;
        }
    }

    jpeg_destroy_decompress(&cinfo);

    return retval;
}

//void set_mincount(int serialPort, int mcount)
//{
//    struct termios tty;
//
//    if (tcgetattr(serialPort, &tty) < 0) {
//        printf("Error tcgetattr: %s\n", strerror(errno));
//        return;
//    }
//
//    tty.c_cc[VMIN] = mcount ? 1 : 0;
//    tty.c_cc[VTIME] = 5; /* half second timer */
//
//    if (tcsetattr(serialPort, TCSANOW, &tty) < 0)
//        printf("Error tcsetattr: %s\n", strerror(errno));
//}

int main(int argc, char ** argv)
{
    const char * portname = "/dev/ttyACM0";

    int wlen;

    const int serialPort = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (serialPort < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 230400, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(serialPort, B230400);
    // set_mincount(serialPort, 0);                /* set to pure timed read */

    /* simple output */
    wlen = write(serialPort, "a\r\n", kCommandLength);
    if (wlen != kCommandLength) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    ::tcdrain(serialPort); /* delay for output */

    auto jpgBuffer = std::make_unique<uint8_t[]>(kJpgBufferSize);

    /*
    auto jpgBuffer = std::make_unique<uint8_t[]>(kJpgBufferSize);

    uint8_t * temp = jpgBuffer.get();

    */

    /* simple noncanonical input */
    uint64_t totalBytesReceived = 0;
    uint32_t timeoutCounter = 0;
    int32_t bytesRead;

    while (1) {
        uint8_t serialBuffer[kSerialBufferSize];

        bytesRead = read(serialPort, serialBuffer, sizeof(serialBuffer) - 1);
        if (bytesRead > 0) {
            // dst src size
            ::memcpy(&jpgBuffer[totalBytesReceived], serialBuffer, bytesRead);

            totalBytesReceived += bytesRead;
        }
        else if (bytesRead < 0) {
            printf("Error from read: %d: %s\n", bytesRead, strerror(errno));
            break;
        }
        else { /* bytesRead == 0 */
            printf("Timeout from read\n");
            if (timeoutCounter > kMaxTimeoutCount) {
                break;
            }
            ++timeoutCounter;
        }
        /* we repeat read to get full message */
    };

    printf("totalBytesReceived = %lu\n", totalBytesReceived);

    if (!DecompressJpegImage(jpgBuffer.get(), totalBytesReceived)) {
        printf("failed to extract image from jpeg data\n");
    }

    if (close(serialPort) != 0) {
        printf("close() failed with %d: %s\n", errno, strerror(errno));
    }
}

