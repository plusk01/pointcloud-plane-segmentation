#ifndef STATISTICSUTILS_H
#define STATISTICSUTILS_H

#include <algorithm>
#include <iostream>
#include <vector>

#include <Eigen/Core>

#include "angleutils.h"

class StatisticsUtils
{
public:
    StatisticsUtils(size_t bufferSize)
    {
        mDataBuffer.reserve(bufferSize);
        mTempBuffer.reserve(bufferSize);
    }

    std::vector<double>& dataBuffer()
    {
        return mDataBuffer;
    }

    size_t size() const
    {
        return mSize;
    }

    void size(size_t size)
    {
        mSize = size;
        if (mDataBuffer.size() < size)
        {
            mDataBuffer.resize(size);
            mTempBuffer.resize(size);
        }
    }

    double getMedian()
    {
        std::memcpy(mTempBuffer.data(), mDataBuffer.data(), sizeof(double) * mSize);
        std::nth_element(mTempBuffer.begin(), mTempBuffer.begin() + mSize / 2, mTempBuffer.begin() + mSize);
        return mTempBuffer[mSize / 2];
    }

    double getMAD(double median)
    {
        for (size_t i = 0; i < mSize; i++)
        {
            mTempBuffer[i] = std::abs(mDataBuffer[i] - median);
        }
        std::nth_element(mTempBuffer.begin(), mTempBuffer.begin() + mSize / 2, mTempBuffer.begin() + mSize);
        return 1.4826f * mTempBuffer[mSize / 2];
    }

    inline double getRScore(double value, double median, double mad)
    {
        return std::abs(value - median) / mad;
    }

    void getMinMaxRScore(double &min, double &max, double range)
    {
        double median = getMedian();
        double mad = getMAD(median);
        min = median - range * mad;
        max = median + range * mad;
    }

    double getMean()
    {
        double sum = 0;
        for (size_t i = 0; i < mSize; i++)
        {
            sum += mDataBuffer[i];
        }
        return sum / mSize;
    }

    double getSTD(double mean)
    {
        double sum = 0;
        for (size_t i = 0; i < mSize; i++)
        {
            sum += std::pow(mDataBuffer[i] - mean, 2);
        }
        return std::sqrt(sum / (mSize - 1));
    }

    void getMinMaxZScore(double &min, double &max, double range)
    {
        double mean = getMean();
        double std = getSTD(mean);
        min = mean - range * std;
        max = mean + range * std;
    }

private:
    std::vector<double> mDataBuffer;
    std::vector<double> mTempBuffer;
    size_t mSize;

};

#endif // STATISTICSUTILS_H
