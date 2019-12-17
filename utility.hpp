#ifndef UTILITY_H
#define UTILITY_H

#include <cstdint>

namespace Util
{

bool WritePPMImageToFile(
    const char * filePath,
    const uint8_t * imageData,
    const uint32_t width,
    const uint32_t height,
    const uint32_t colorBitFactor);
};

#endif //#ifndef UTILITY_H

