// taken from https://stackoverflow.com/a/38318768

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <jpeglib.h>
#include <memory>
#include <SDL2/SDL.h>
#include <termios.h>
#include <unistd.h>

#include "serialcomm.hpp"
#include "utility.hpp"

using UPtrBytes = std::unique_ptr<uint8_t[]>;

// we are receiving JPEG image
static constexpr const size_t k1KB = 1024;
static constexpr size_t kJpgBufferSize = 128 * k1KB;
static constexpr size_t kMaxTimeoutCount = 20;
static constexpr size_t kSerialBufferSize = 100;
static constexpr size_t kVideoWidth = 320;
static constexpr size_t kVideoHeight = 240;

//static int g_serialPort = -1;

enum class ParserState : uint32_t
{
    SeekStart,
    SeekEnd,
};

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
    tty.c_cc[VTIME] = 2;

    if (::tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", ::strerror(errno));
        return -1;
    }
    return 0;
}

// TODO: proper error handling
UPtrBytes DecompressJpegImage(
    const uint8_t * inputImage,
    const uint32_t dataSize)
{
    struct jpeg_decompress_struct dinfo;
	struct jpeg_error_mgr jerr;

    dinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&dinfo);
    jpeg_mem_src(&dinfo, inputImage, dataSize);

    if (jpeg_read_header(&dinfo, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&dinfo);
        return UPtrBytes{nullptr};
    }

    jpeg_start_decompress(&dinfo);

    const uint32_t width = dinfo.output_width;
    const uint32_t height = dinfo.output_height;
    const uint32_t colorBitFactor = dinfo.output_components;
    const uint32_t rowStride = width * colorBitFactor;

    printf("Image is %u by %u with %u components\n", width, height, colorBitFactor);

    uint64_t rawImageSize = width * height * colorBitFactor;

    UPtrBytes rawImage = std::make_unique<uint8_t[]>(rawImageSize);

    uint8_t * bufferArray[1];
    while (dinfo.output_scanline < dinfo.output_height) {
        bufferArray[0] = rawImage.get() + dinfo.output_scanline * rowStride;
        jpeg_read_scanlines(&dinfo, bufferArray, 1);
    }

    jpeg_finish_decompress(&dinfo);

    printf("jpeg decompression finished\n");

    jpeg_destroy_decompress(&dinfo);

    return std::move(rawImage);
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

/*
 * TODO
bool initialize(const char * portname)
{
    g_serialPort = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (g_serialPort < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return false;
    }

    return true;
}

void cleanup()
{
    if (g_serialPort != -1 && close(g_serialPort) != 0) {
        printf("close() failed with %d: %s\n", errno, strerror(errno));
    }
}
*/

int main(int argc, char ** argv)
{
    const char * portname = "/dev/ttyACM0";
    //const char * portname = "/dev/rfcomm0";

    printf("before opening\n");
    int serialPort = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (serialPort < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 230400, 8 bits, no parity, 1 stop bit */
    //set_interface_attribs(serialPort, B460800);
    // HC05 says 460800 baud rate is possible but I couldn't get it to work
    int retval = set_interface_attribs(serialPort, B230400);
    //int retval = set_interface_attribs(serialPort, B921600);
    //set_interface_attribs(serialPort, B115200);
    // set_mincount(serialPort, 0);                /* set to pure timed read */
    if (retval != 0) {
        printf("failed to set baud rate\n");
    }

    // we alternate containers so that data stream is buffered in 1 container
    // while jpeg data is processed in another one
    auto jpgBuffer = std::make_unique<uint8_t[]>(kJpgBufferSize);
    auto jpgBuffer2 = std::make_unique<uint8_t[]>(kJpgBufferSize);

    uint64_t totalBytesReceived = 0;
    uint64_t jpegFrameSize;
    uint32_t timeoutCounter = 0;
    int32_t bytesRead;

    uint8_t serialBuffer[kSerialBufferSize];
    uint32_t serialBufferDelimiter = 0; // TODO: use this
    uint32_t capturedFrames = 0;

    printf("before first write\n");
    const char * cmd = "r";
    int wlen = write(serialPort, cmd, ::strlen(cmd));
    if (wlen != ::strlen(cmd)) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    //::tcdrain(serialPort); /* delay for output */

    /* simple noncanonical input */
    while (1) {
        bytesRead = read(
            serialPort,
            //&serialBuffer[serialBufferDelimiter],
            serialBuffer,
            sizeof(serialBuffer) - 1);

        //printf("read %d bytes\n", bytesRead);

        if (bytesRead > 0) {
            ::memcpy(&jpgBuffer[totalBytesReceived], serialBuffer, bytesRead);

            totalBytesReceived += bytesRead;

            //printf("got %lu bytes in total\n", totalBytesReceived);

            // TODO: confirm that the marker is always there
            if (totalBytesReceived > 4) {
                // there should be at least a SOI and a EOI marker
                if (serialBuffer[bytesRead - 2] == 0xff &&
                    serialBuffer[bytesRead - 1] == 0xd9)
                {
                    jpegFrameSize = totalBytesReceived;
                    printf("jpeg frame has %lu bytes.\n", jpegFrameSize);
                    totalBytesReceived = 0;
                    capturedFrames += 1;
                    if (capturedFrames > 10) {
                        break;
                    }
                    continue;
                }
            }

            //timeoutCounter = 0;
        }
        else if (bytesRead < 0) {
            printf("Error from read: %d: %s\n", bytesRead, strerror(errno));
            break;
        }
        else { /* bytesRead == 0 */
            if (timeoutCounter > kMaxTimeoutCount) {
                printf("Timeout from read\n");
                break;
            }
            ++timeoutCounter;
        }
        /* we repeat read to get full message */
    };

    printf("captured %u frames\n", capturedFrames);

    const char * cmd2 = "x";
    wlen = write(serialPort, cmd2, ::strlen(cmd2));
    if (wlen != ::strlen(cmd2)) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    ::tcdrain(serialPort); /* delay for output */

    //printf("totalBytesReceived = %lu\n", totalBytesReceived);

    // TODO: have image processing in another thread
    /*
    UPtrBytes rawImage = DecompressJpegImage(jpgBuffer.get(), jpegFrameSize);

    if (rawImage.get() != nullptr) {
        if (!Util::WritePPMImageToFile(
                "output.ppm", rawImage.get(), kVideoWidth, kVideoHeight, 3)) {
        }
    }
    else {
        printf("failed to extract image from jpeg data\n");
    }
    */

    if (close(serialPort) != 0) {
        printf("close() failed with %d: %s\n", errno, strerror(errno));
    }
}

