#include "utility.hpp"

#include <fstream>
#include <iostream>

bool Util::WritePPMImageToFile(
    const char * filePath,
    const uint8_t * imageData,
    const uint32_t width,
    const uint32_t height,
    const uint32_t colorBitFactor)
{
    if (filePath == nullptr || filePath[0] == '\0') {
        std::cout << "filePath cannot be null or empty!\n";
        return false;
    }

    std::ofstream writeStream{filePath, std::ios::binary};

    const char * magicNumber = (colorBitFactor == 1) ? "P5" : "P6";

    writeStream << magicNumber << '\n';
    writeStream << width << " " << height << '\n';
    writeStream << 255 << '\n';

    // just grayscale image for now
    const size_t dataSize = width * height * colorBitFactor;
    if (!writeStream.write(reinterpret_cast<const char *>(imageData), dataSize)) {
        std::cout << "failed to write to " << filePath << "!\n";
        return false;
    }

    return true;
}

