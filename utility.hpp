#ifndef UTILITY_H
#define UTILITY_H

#include <cstdint>
#include <memory>

/*
 * TODO: use this instead of raw data
struct ImageMetaData
{
    uint32_t width;
    uint32_t height;
    bool isColored;
    std::unique_ptr<uint8_t[]> data;
};
*/

namespace Util
{

bool WritePPMImageToFile(
    const char * filePath,
    const uint8_t * imageData,
    const uint32_t width,
    const uint32_t height,
    const uint32_t colorBitFactor);

std::unique_ptr<uint8_t[]> ConvertColorImageToGrayscale(
    const uint8_t * imageData,
    const uint32_t width,
    const uint32_t height);
};

#endif //#ifndef UTILITY_H

