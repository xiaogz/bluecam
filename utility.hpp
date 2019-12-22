#ifndef UTILITY_H
#define UTILITY_H

#include <cstdint>
#include <memory>
#include <vector>

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

    // TODO: add a overall function called GetEdges()

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

std::unique_ptr<uint8_t[]> ComputeGaussianBlur(
    const uint8_t * inputData,
    const uint32_t inputWidth,
    const uint32_t inputHeight);

std::unique_ptr<uint8_t[]> ComputeSobelEdges(
    const uint8_t * inputData,
    const uint32_t inputWidth,
    const uint32_t inputHeight,
    std::vector<double> & angles);

void NonMaximumEdgeSuppression(
    const std::vector<double> & sobelAngles,
    const long inputWidth,
    const long inputHeight,
    uint8_t * sobelGradient);

uint8_t ComputeOtsuThreshold(
    uint8_t * inputData,
    const size_t inputDataSize);

void HysteresisEdgeFiltering(
    const uint32_t inputWidth,
    const uint32_t inputHeight,
    const uint8_t lowerThreshold,
    const uint8_t higherThreshold,
    uint8_t * edges);
};

#endif //#ifndef UTILITY_H

