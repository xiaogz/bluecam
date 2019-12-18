#include "utility.hpp"

#include <fstream>
#include <iostream>
#include <memory>

static constexpr const uint32_t kMaxPixelValue = 255;

using UPtrBytes = std::unique_ptr<uint8_t[]>;

bool Util::WritePPMImageToFile(
    const char * filePath,
    const uint8_t * imageData,
    const uint32_t width,
    const uint32_t height,
    const uint32_t colorBitFactor)
{
    if (filePath == nullptr || filePath[0] == '\0') {
        std::cerr << "filePath cannot be null or empty!\n";
        return false;
    }

    if (colorBitFactor != 1 && colorBitFactor != 3) {
        std::cerr << "colorBitFactor is %u but it should either be 1 (grayscale) or 3 (RGB)\n";
        return false;
    }

    std::ofstream writeStream{filePath, std::ios::binary};

    const char * magicNumber = (colorBitFactor == 1) ? "P5" : "P6";

    writeStream << magicNumber << '\n';
    writeStream << width << " " << height << '\n';
    writeStream << kMaxPixelValue << '\n';

    const size_t dataSize = width * height * colorBitFactor;
    if (!writeStream.write(reinterpret_cast<const char *>(imageData), dataSize)) {
        std::cout << "failed to write to " << filePath << "!\n";
        return false;
    }

    return true;
}

// this function transfers ownership!
UPtrBytes Util::ConvertColorImageToGrayscale(
    const uint8_t * coloredImageData,
    const uint32_t width,
    const uint32_t height)
{
    const size_t pixelCount = width * height;
    const size_t colorBitFactor = 3;

    UPtrBytes grayImage = std::make_unique<uint8_t[]>(pixelCount);

    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const uint32_t newIndex = (y * width + x);
            const uint32_t inputIndex = newIndex * colorBitFactor;

            double grayPixel =
                0.299 * coloredImageData[inputIndex] +
                0.587 * coloredImageData[inputIndex + 1] +
                0.114 * coloredImageData[inputIndex + 2];

            grayImage[newIndex] = static_cast<uint8_t>(grayPixel);
        }
    }

    return std::move(grayImage);
}

