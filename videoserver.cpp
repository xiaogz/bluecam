// taken from https://stackoverflow.com/a/38318768

#include <SDL2/SDL.h>

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
#include <vector>

#include "serialcomm.hpp"
#include "utility.hpp"

using UPtrBytes = std::unique_ptr<uint8_t[]>;

// we are receiving JPEG image
static constexpr const size_t k1KB = 1024;
static constexpr size_t kJpgBufferSize = 32 * k1KB;
static constexpr size_t kMaxTimeoutCount = 20;
static constexpr size_t kSerialBufferSize = 100;
static constexpr size_t kTotalFrames = 5;
static constexpr const char* kAppName = "BlueCam";
static constexpr size_t kVideoWidth = 320;
static constexpr size_t kVideoHeight = 240;

// TODO: we can make all this into a class
// SDL-specific stuff
////////////////////
// begin of namespace GUI
namespace GUI
{
////////////////////

SDL_Window* g_window = nullptr;
SDL_Renderer* g_renderer = nullptr;
SDL_Texture* g_texture = nullptr; // for gray image frames only
SDL_bool g_isDone = SDL_FALSE;

bool InitGUI(const char* appName, const size_t width, const size_t height)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("failed to initialize SDL\n");
        return false;
    }

    g_window = SDL_CreateWindow(appName,
                                SDL_WINDOWPOS_UNDEFINED,
                                SDL_WINDOWPOS_UNDEFINED,
                                width,
                                height,
                                SDL_WINDOW_RESIZABLE);
    if (!g_window) {
        printf("failed to create SDL window\n");
        return false;
    }

    const int firstAvailableRenderer = -1;
    const int flags = 0;
    g_renderer = SDL_CreateRenderer(g_window, firstAvailableRenderer, flags);
    if (!g_renderer) {
        printf("failed to create renderer\n");
        return false;
    }

    if (SDL_RenderSetScale(g_renderer, 2.0, 2.0) != 0) {
        printf("failed to set scaling factor\n");
        return false;
    }

    g_texture = SDL_CreateTexture(
      g_renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!g_texture) {
        printf("failed to create streaming texture for grayscale image\n");
        return false;
    }

    return true;
}

// this function relies on some global variables
// this function only works for coloured images for now
bool UpdateRGBFrame(const uint8_t* pixelData)
{
    const int colorBitFactor = 3;

    const int depth = colorBitFactor * 8;
    const int pitch = colorBitFactor * kVideoWidth;

    const uint32_t rmask = 0x000000ff;
    const uint32_t gmask = 0x0000ff00;
    const uint32_t bmask = 0x00ff0000;
    const uint32_t amask = 0;

    SDL_Surface* surface = SDL_CreateRGBSurfaceFrom(
      (void*)pixelData, kVideoWidth, kVideoHeight, depth, pitch, rmask, gmask, bmask, amask);

    if (!surface) {
        printf("failed to create surface!\n");
        return false;
    }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(g_renderer, surface);
    if (!texture) {
        printf("failed to create texture from surface!\n");
        return false;
    }

    SDL_RenderClear(g_renderer);
    SDL_RenderCopy(g_renderer, texture, NULL, NULL);
    SDL_RenderPresent(g_renderer);

    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);

    return true;
}

bool UpdateGrayFrame(const uint8_t* grayPixelData)
{
    uint8_t* pixelAccess = nullptr;
    int pitch = 0; // not used

    SDL_LockTexture(g_texture, nullptr, reinterpret_cast<void**>(&pixelAccess), &pitch);

    for (size_t y = 0; y < kVideoHeight; ++y) {
        for (size_t x = 0; x < kVideoWidth; ++x) {
            const size_t inputIndex = (y * kVideoWidth + x);
            const size_t newIndex = inputIndex * 4;
            pixelAccess[newIndex] = 0xff; // alpha
            pixelAccess[newIndex + 1] = grayPixelData[inputIndex];
            pixelAccess[newIndex + 2] = grayPixelData[inputIndex];
            pixelAccess[newIndex + 3] = grayPixelData[inputIndex];
        }
    }

    SDL_UnlockTexture(g_texture);

    SDL_RenderClear(g_renderer);
    SDL_RenderCopy(g_renderer, g_texture, NULL, NULL);
    SDL_RenderPresent(g_renderer);

    return true;
}

void Cleanup()
{
    if (g_window) {
        SDL_DestroyWindow(g_window);
        g_window = nullptr;
    }

    if (g_renderer) {
        SDL_DestroyRenderer(g_renderer);
        g_renderer = nullptr;
    }

    if (g_texture) {
        SDL_DestroyTexture(g_texture);
        g_texture = nullptr;
    }

    SDL_Quit();
}

////////////////////
// end of namespace GUI
};
////////////////////

enum class ParserState : uint32_t
{
    SeekStart,
    SeekEnd,
};

uint64_t GetTime()
{
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return time.tv_sec * 10e9 + time.tv_nsec;
}

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
    tty.c_cflag |= CS8;     /* 8-bit characters */
    tty.c_cflag &= ~PARENB; /* no parity bit */
    tty.c_cflag &= ~CSTOPB; /* only need 1 stop bit */
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
UPtrBytes DecompressJpegImage(const uint8_t* inputImage, const uint32_t dataSize)
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

    uint8_t* bufferArray[1];
    while (dinfo.output_scanline < dinfo.output_height) {
        bufferArray[0] = rawImage.get() + dinfo.output_scanline * rowStride;
        jpeg_read_scanlines(&dinfo, bufferArray, 1);
    }

    jpeg_finish_decompress(&dinfo);

    printf("jpeg decompression finished\n");

    jpeg_destroy_decompress(&dinfo);

    return std::move(rawImage);
}

// void set_mincount(int serialPort, int mcount)
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

int main(int argc, char** argv)
{
    if (!GUI::InitGUI(kAppName, kVideoWidth, kVideoHeight)) {
        printf("Error initializing SDL GUI\n");
        return -1;
    }

    const char* portname = "/dev/ttyACM0";
    // const char * portname = "/dev/rfcomm0";

    printf("before opening\n");
    int serialPort = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (serialPort < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 230400, 8 bits, no parity, 1 stop bit */
    // set_interface_attribs(serialPort, B460800);
    // HC05 says 460800 baud rate is possible but I couldn't get it to work
    int retval = set_interface_attribs(serialPort, B230400);
    // int retval = set_interface_attribs(serialPort, B460800);
    // int retval = set_interface_attribs(serialPort, B921600);
    // set_interface_attribs(serialPort, B115200);
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
    const uint64_t kPixelCount = kVideoWidth * kVideoHeight;

    uint8_t serialBuffer[kSerialBufferSize];
    uint32_t serialBufferDelimiter = 0; // TODO: use this
    uint32_t capturedFrames = 0;

    printf("before first write\n");
    const char* cmd = "r";
    int wlen = write(serialPort, cmd, ::strlen(cmd));
    if (wlen != ::strlen(cmd)) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    // this delay causes first frame's read to be incomplete
    //::tcdrain(serialPort); /* delay for output */

    int streamState = 0; // 0 for seeking start, 1 for seeking end

    /* simple noncanonical input */
    while (1) {
        bytesRead = read(serialPort,
                         //&serialBuffer[serialBufferDelimiter],
                         serialBuffer,
                         sizeof(serialBuffer) - 1);

        // printf("read %d bytes\n", bytesRead);

        if (bytesRead > 0) {

            ::memcpy(&jpgBuffer[totalBytesReceived], serialBuffer, bytesRead);

            totalBytesReceived += bytesRead;

            // printf("got %lu bytes in total\n", totalBytesReceived);

            // TODO: confirm that the marker is always there
            if (totalBytesReceived > 4) {
                // there should be at least a SOI and a EOI marker
                if (serialBuffer[bytesRead - 2] == 0xff && serialBuffer[bytesRead - 1] == 0xd9) {
                    jpegFrameSize = totalBytesReceived;
                    printf("jpeg frame has %lu bytes.\n", jpegFrameSize);
                    totalBytesReceived = 0;

                    UPtrBytes rawImage = DecompressJpegImage(jpgBuffer.get(), jpegFrameSize);

                    if (rawImage.get() == nullptr) {
                        printf("jpeg decompression failed\n");
                        continue;
                    }

                    UPtrBytes grayImage = Util::ConvertColorImageToGrayscale(
                        rawImage.get(), kVideoWidth, kVideoHeight);

                    UPtrBytes blurredImage = Util::ComputeGaussianBlur(
                        grayImage.get(), kVideoWidth, kVideoHeight);

                    std::vector<double> sobelAngles(kPixelCount);

                    UPtrBytes edgeGradients = Util::ComputeSobelEdges(
                        blurredImage.get(), kVideoWidth, kVideoHeight, sobelAngles);

                    Util::NonMaximumEdgeSuppression(
                        sobelAngles, kVideoWidth, kVideoHeight, edgeGradients.get());

                    const uint8_t otsuThreshold = Util::ComputeOtsuThreshold(
                        edgeGradients.get(), kPixelCount);

                    Util::HysteresisEdgeFiltering(
                        kVideoWidth,
                        kVideoHeight,
                        otsuThreshold / 2,
                        otsuThreshold,
                        edgeGradients.get());

                    if (!GUI::UpdateGrayFrame(edgeGradients.get())) {
                        GUI::Cleanup();
                        return 1;
                    }

                    // TODO: overlay image processing results onto rawImage

                    capturedFrames += 1;
                    if (capturedFrames >= kTotalFrames) {
                        // capture last frame for eye validation
                        if (!Util::WritePPMImageToFile(
                              "output-color.pgm", rawImage.get(), kVideoWidth, kVideoHeight, 3)) {
                            printf("failed to write image to file\n");
                        }

                        if (!Util::WritePPMImageToFile(
                              "output-blurred.pgm", blurredImage.get(), kVideoWidth, kVideoHeight, 1)) {
                            printf("failed to write image to file\n");
                        }

                        if (!Util::WritePPMImageToFile(
                              "output-edges.pgm", edgeGradients.get(), kVideoWidth, kVideoHeight, 1)) {
                            printf("failed to write image to file\n");
                        }

                        break;
                    }
                    continue;
                }
            }
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

    const char* cmd2 = "x";
    wlen = write(serialPort, cmd2, ::strlen(cmd2));
    if (wlen != ::strlen(cmd2)) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    ::tcdrain(serialPort); /* delay for output */

    // TODO: have image processing in another thread
    if (close(serialPort) != 0) {
        printf("close() failed with %d: %s\n", errno, strerror(errno));
    }

    GUI::Cleanup();
}
