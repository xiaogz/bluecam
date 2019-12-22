#include "utility.hpp"

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>

static constexpr const uint32_t kMaxPixelValue = 255;

using UPtrBytes = std::unique_ptr<uint8_t[]>;

// smoothing is also known as averaging; like a mini-Gaussian blur
static constexpr const double g_sobelSmoothing[] = {1.0, 2.0, 1.0};
static constexpr const double g_sobelDifferentation[] = {-1.0, 0.0, 1.0};
static constexpr const long g_sobelKernelSize = 3;

constexpr const size_t g_kIntensityCount = 256;

constexpr const double g_kPI = M_PI;
constexpr const double g_kOneEighthPI = M_PI / 8;
constexpr const double g_kThreeEighthPI = 3 * g_kOneEighthPI;
constexpr const double g_kFiveEighthPI = 5 * g_kOneEighthPI;
constexpr const double g_kSevenEighthPI = 7 * g_kOneEighthPI;
constexpr const double g_kNPI = -1.0 * M_PI;
constexpr const double g_kNOneEighthPI = -1.0 * g_kOneEighthPI;
constexpr const double g_kNThreeEighthPI = -1.0 * g_kThreeEighthPI;
constexpr const double g_kNFiveEighthPI = -1.0 * g_kFiveEighthPI;
constexpr const double g_kNSevenEighthPI = -1.0 * g_kSevenEighthPI;

enum class EdgeDirection
{
    Horizontal,     // -
    Vertical,       // |
    ForwardDiag,    // /
    BackwardDiag,   // '\'
};

inline EdgeDirection GetEdgeDirection(const double angle)
{
    if ((angle <= g_kOneEighthPI && angle >= g_kNOneEighthPI) ||
        (angle >= g_kSevenEighthPI || angle <= g_kNSevenEighthPI)) {
        return EdgeDirection::Horizontal;
    }
    else if ((angle >= g_kOneEighthPI && angle <= g_kThreeEighthPI) ||
        (angle <= g_kNFiveEighthPI && angle >= g_kNSevenEighthPI)){
        return EdgeDirection::ForwardDiag;
    }
    else if ((angle >= g_kThreeEighthPI && angle <= g_kFiveEighthPI) ||
        (angle <= g_kNThreeEighthPI && angle >= g_kNFiveEighthPI)) {
        return EdgeDirection::Vertical;
    }

    // only backward diagonal left
    return EdgeDirection::BackwardDiag;
}

static std::vector<double> Compute1DGaussianKernel(const double stdDev);

static const std::vector<double> g_kGk1d = Compute1DGaussianKernel(1.7);

static std::vector<double> Compute1DGaussianKernel(const double stdDev)
{
    int gridSize = static_cast<int>(stdDev * 3);

    assert(gridSize > 1);

    // ensure kernel size is odd to avoid shifting image
    if (gridSize % 2 == 0) {
        gridSize += 1;
    }

    const double distFromOrigin = static_cast<double>(gridSize) / 2;
    const double sigmaSquared = stdDev * stdDev;
    const double denominator = 2 * M_PI * sigmaSquared;

    // use 1D kernels instead of 2D
    std::vector<double> gk(gridSize);
    double sum = 0.0;

    for (int x = 0; x < gridSize; ++x) {
        gk[x] = exp(-0.5 * ::pow((x - distFromOrigin), 2.0) / denominator) / ::sqrt(denominator);
        sum += gk[x];
    }

    // normalize kernel
    for (auto& element : gk) {
        element /= sum;
    }

    return gk;
}

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

UPtrBytes Util::ComputeGaussianBlur(
    const uint8_t * inputData,
    const uint32_t inputWidth,
    const uint32_t inputHeight)
{
    const size_t inputDataSize = static_cast<size_t>(inputWidth) * inputHeight;

    std::vector<double> intermediateContainer(inputDataSize, 0.0);

    const long horizontalBorderOffset1 = static_cast<long>(g_kGk1d.size() / 2);
    const long horizontalBorderOffset2 = inputWidth - horizontalBorderOffset1;

    const long verticalBorderOffset1 = horizontalBorderOffset1;
    const long verticalBorderOffset2 = inputHeight - verticalBorderOffset1;

    const long gkGridOffset = horizontalBorderOffset1;

    double tempValue;

    UPtrBytes blurredImage = std::make_unique<uint8_t[]>(inputDataSize);

    // horizontal gaussian blurring
    for (long y = 0; y < inputHeight; ++y) {
        // middle
        for (long x = horizontalBorderOffset1; x < horizontalBorderOffset2; ++x) {
            const long dataIndex = y * inputWidth + x;
            tempValue = 0.0;
            for (size_t i = 0; i < g_kGk1d.size(); ++i) {
                const uint8_t pixel = static_cast<uint8_t>(
                    inputData[dataIndex + i - gkGridOffset]);
                tempValue += (g_kGk1d[i] * pixel);
            }
            intermediateContainer[dataIndex] += tempValue;
        }
        // left
        for (long x = 0; x < horizontalBorderOffset1; ++x) {
            const long dataIndex = y * inputWidth + x;
            tempValue = 0.0;
            for (long i = gkGridOffset - x; i < g_kGk1d.size(); ++i) {
                const uint8_t pixel = static_cast<uint8_t>(
                    inputData[dataIndex + i - gkGridOffset]);
                tempValue += (g_kGk1d[i] * pixel);
            }
            intermediateContainer[dataIndex] += tempValue;
        }
        // right
        for (long x = horizontalBorderOffset2; x < inputWidth; ++x) {
            const long dataIndex = y * inputWidth + x;
            tempValue = 0.0;
            for (long i = 0; i < (g_kGk1d.size() - (inputWidth - x) - 1); ++i) {
                const uint8_t pixel = static_cast<uint8_t>(
                    inputData[dataIndex + i - gkGridOffset]);
                tempValue += (g_kGk1d[i] * pixel);
            }
            intermediateContainer[dataIndex] += tempValue;
        }
    }

    // second round of vertical Gaussian blurring should account for the
    // horizontal blurring that already occurred!

    // vertical gaussian blurring
    for (long x = 0; x < inputWidth; ++x) {
        // middle
        for (long y = verticalBorderOffset1; y < verticalBorderOffset2; ++y) {
            const long dataIndex = y * inputWidth + x;
            tempValue = 0.0;
            for (size_t i = 0; i < g_kGk1d.size(); ++i) {
                const double intermediateValue =
                    intermediateContainer[dataIndex + inputWidth * (i - gkGridOffset)];
                tempValue += (g_kGk1d[i] * intermediateValue);
            }
            blurredImage[dataIndex] = static_cast<uint8_t>(tempValue);
        }
        // top
        for (long y = 0; y < verticalBorderOffset1; ++y) {
            const long dataIndex = y * inputWidth + x;
            tempValue = 0.0;
            for (long i = gkGridOffset - y; i < g_kGk1d.size(); ++i) {
                const double intermediateValue =
                    intermediateContainer[dataIndex + inputWidth * (i - gkGridOffset)];
                tempValue += (g_kGk1d[i] * intermediateValue);
            }
            blurredImage[dataIndex] = static_cast<uint8_t>(tempValue);
        }
        // bottom
        for (long y = verticalBorderOffset2; y < inputHeight; ++y) {
            const long dataIndex = y * inputWidth + x;
            tempValue = 0.0;
            for (long i = 0; i < (g_kGk1d.size() - (inputHeight - y) - 1); ++i) {
                const double intermediateValue =
                    intermediateContainer[dataIndex + inputWidth * (i - gkGridOffset)];
                tempValue += (g_kGk1d[i] * intermediateValue);
            }
            blurredImage[dataIndex] = static_cast<uint8_t>(tempValue);
        }
    }

    return std::move(blurredImage);
}

UPtrBytes Util::ComputeSobelEdges(
    const uint8_t * inputData,
    const uint32_t inputWidth,
    const uint32_t inputHeight,
    std::vector<double> & angles)
{
    const size_t inputDataSize = static_cast<size_t>(inputWidth) * inputHeight;

    const long horizontalBorderOffset1 = g_sobelKernelSize / 2;
    const long horizontalBorderOffset2 = inputWidth - horizontalBorderOffset1;

    const long verticalBorderOffset1 = horizontalBorderOffset1;
    const long verticalBorderOffset2 = inputHeight - verticalBorderOffset1;

    const long sobelKernelOffset = horizontalBorderOffset1;

    std::vector<double> holder(inputDataSize);
    std::vector<double> horizontalGradient(inputDataSize);
    std::vector<double> verticalGradient(inputDataSize);

    // horizontal Sobel differentiation
    for (long y = verticalBorderOffset1; y < verticalBorderOffset2; ++y) {
        // middle
        for (long x = horizontalBorderOffset1; x < horizontalBorderOffset2; ++x) {
            const long dataIndex = y * inputWidth + x;
            double tempValue = 0.0;
            for (size_t i = 0; i < g_sobelKernelSize; ++i) {
                tempValue += (g_sobelDifferentation[i] *
                    inputData[dataIndex + i - sobelKernelOffset]);
            }
            holder[dataIndex] = tempValue;
        }
    }
    // horizontal Sobel smoothing; the horizontal gradient's smoothing matrix is vertical
    for (long x = horizontalBorderOffset1; x < horizontalBorderOffset2; ++x) {
        // middle
        for (long y = verticalBorderOffset1; y < verticalBorderOffset2; ++y) {
            const long dataIndex = y * inputWidth + x;
            double tempValue = 0.0;
            for (size_t i = 0; i < g_sobelKernelSize; ++i) {
                tempValue += (g_sobelSmoothing[i] *
                    holder[dataIndex + inputWidth * (i - sobelKernelOffset)]);
            }
            horizontalGradient[dataIndex] = tempValue;
        }
    }

    // vertical Sobel differentiation
    for (long x = horizontalBorderOffset1; x < horizontalBorderOffset2; ++x) {
        // middle
        for (long y = verticalBorderOffset1; y < verticalBorderOffset2; ++y) {
            const long dataIndex = y * inputWidth + x;
            double tempValue = 0.0;
            for (size_t i = 0; i < g_sobelKernelSize; ++i) {
                tempValue += (g_sobelDifferentation[i] *
                    inputData[dataIndex + inputWidth * (i - sobelKernelOffset)]);
            }
            holder[dataIndex] = tempValue;
        }
    }
    // vertical Sobel smoothing; the vertical gradient's smoothing matrix is horizontal
    for (long y = verticalBorderOffset1; y < verticalBorderOffset2; ++y) {
        // middle
        for (long x = horizontalBorderOffset1; x < horizontalBorderOffset2; ++x) {
            const long dataIndex = y * inputWidth + x;
            double tempValue = 0.0;
            for (size_t i = 0; i < g_sobelKernelSize; ++i) {
                tempValue += (g_sobelSmoothing[i] *
                    holder[dataIndex + i - sobelKernelOffset]);
            }
            verticalGradient[dataIndex] = tempValue;
        }
    }

    UPtrBytes retEdges = std::make_unique<uint8_t[]>(inputDataSize);

    for (size_t i = 0; i < holder.size(); ++i) {
        holder[i] = std::min(
            std::max(
                ::sqrt(::pow(horizontalGradient[i], 2.0) + ::pow(verticalGradient[i], 2.0)),
                0.0),
            255.0);

        retEdges[i] = static_cast<uint8_t>(holder[i]);
        // TODO: use GetEdgeDirection here
        angles[i] = ::atan2(verticalGradient[i], horizontalGradient[i]);
    }

    return std::move(retEdges);
}

void Util::NonMaximumEdgeSuppression(
    const std::vector<double> & sobelAngles,
    const long inputWidth,
    const long inputHeight,
    uint8_t * sobelGradient)
{
    const long verticalBorderOffset1 = 1;
    const long verticalBorderOffset2 = inputHeight - 1;
    const long horizontalBorderOffset1 = 1;
    const long horizontalBorderOffset2 = inputWidth - 1;

    // we are guaranteed to have surrounding pixels since we are staying within
    // the edge pixels
    for (long y = verticalBorderOffset1; y < verticalBorderOffset2; ++y) {
        for (long x = horizontalBorderOffset1; x < horizontalBorderOffset2; ++x) {
            const long dataIndex = y * inputWidth + x;
            uint8_t & gradient = sobelGradient[dataIndex];
            switch(GetEdgeDirection(sobelAngles[dataIndex])) {
            case EdgeDirection::Horizontal:
                if (gradient < sobelGradient[dataIndex - inputWidth] ||
                    gradient < sobelGradient[dataIndex + inputWidth]) {
                    gradient = 0;
                }
                break;
            case EdgeDirection::ForwardDiag:
                if (gradient < sobelGradient[dataIndex - inputWidth - 1] ||
                    gradient < sobelGradient[dataIndex + inputWidth + 1]) {
                    gradient = 0;
                }
                break;
            case EdgeDirection::Vertical:
                if (gradient < sobelGradient[dataIndex - 1] ||
                    gradient < sobelGradient[dataIndex + 1]) {
                    gradient = 0;
                }
                break;
            case EdgeDirection::BackwardDiag:
                if (gradient < sobelGradient[dataIndex - inputWidth + 1] ||
                    gradient < sobelGradient[dataIndex + inputWidth - 1]) {
                    gradient = 0;
                }
                break;
            }
        }
    }
}

uint8_t Util::ComputeOtsuThreshold(uint8_t * inputData, const size_t inputDataSize)
{
    // TODO: move this out of function
    static std::array<uint64_t, g_kIntensityCount> pixelCounts{};
    static std::array<uint64_t, g_kIntensityCount> weightedCounts{};

    const uint64_t & totalPixelCount = inputDataSize;
    uint64_t totalWeightedCount = 0;

    std::fill(pixelCounts.begin(), pixelCounts.end(), 0);

    for (size_t i = 0; i < inputDataSize; ++i) {
        pixelCounts[inputData[i]] += 1;
    }

    for (size_t i = 0; i < g_kIntensityCount; ++i) {
        weightedCounts[i] = pixelCounts[i] * i;
        totalWeightedCount += weightedCounts[i];
        // debug
        //std::cout << "i = " << i << " pixelCounts[i] = " << pixelCounts[i] << '\n';
    }

    uint8_t threshold = 0;

    uint64_t backPixelCount = 0;
    uint64_t forePixelCount;

    double backWeightedCount = 0;
    double foreWeightedCount;

    double maxICVar = 0.0;

    for (size_t i = 0; i < g_kIntensityCount; ++i) {
        const uint64_t & currentPixelCount = pixelCounts[i];
        backPixelCount += currentPixelCount;
        if (backPixelCount == 0) {
            continue;
        }

        forePixelCount = totalPixelCount - backPixelCount;
        if (forePixelCount == 0) {
            break;
        }

        const uint64_t & currentWeightedCount = weightedCounts[i];

        backWeightedCount += currentWeightedCount;

        const double backMean = static_cast<double>(backWeightedCount) / backPixelCount;
        const double foreMean =
            static_cast<double>(totalWeightedCount - backWeightedCount) / forePixelCount;

        const double ICVar = ::pow(backMean - foreMean, 2.0) * backPixelCount * forePixelCount;
        if (ICVar > maxICVar) {
            maxICVar = ICVar;
            threshold = i;
        }
    }

    return threshold;
}

void Util::HysteresisEdgeFiltering(
    const uint32_t inputWidth,
    const uint32_t inputHeight,
    const uint8_t lowerThreshold,
    const uint8_t higherThreshold,
    uint8_t * edges)
{
    const long verticalBorderOffset1 = 1;
    const long verticalBorderOffset2 = inputHeight - 1;
    const long horizontalBorderOffset1 = 1;
    const long horizontalBorderOffset2 = inputWidth - 1;

    // we are guaranteed to have surrounding pixels since we are staying within
    // the edge pixels
    for (long y = verticalBorderOffset1; y < verticalBorderOffset2; ++y) {
        for (long x = horizontalBorderOffset1; x < horizontalBorderOffset2; ++x) {
            const long dataIndex = y * inputWidth + x;
            uint8_t & edgeStrength = edges[dataIndex];
            /*
            if (edgeStrength >= higherThreshold) {
                continue;
            }
            */
            if (edgeStrength < lowerThreshold) {
                edgeStrength = 0;
            }
            else if (edgeStrength < higherThreshold) {
                // medium-strength edges are discarded if they are not near any strong edge
                if (edges[dataIndex - 1] < higherThreshold &&
                    edges[dataIndex + 1] < higherThreshold &&
                    edges[dataIndex - inputWidth - 1] < higherThreshold &&
                    edges[dataIndex - inputWidth] < higherThreshold &&
                    edges[dataIndex - inputWidth + 1] < higherThreshold &&
                    edges[dataIndex + inputWidth - 1] < higherThreshold &&
                    edges[dataIndex + inputWidth] < higherThreshold &&
                    edges[dataIndex + inputWidth + 1] < higherThreshold)
                {
                    edgeStrength = 0;
                }
            }
        }
    }
}

