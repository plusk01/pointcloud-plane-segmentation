#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <mutex>
#include <vector>
#include <set>

#include "point.h"

template <size_t DIMENSION>
class PointCloud
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;

    PointCloud(const std::vector<Point<DIMENSION> > &points)
        : mPoints(points)
    {
        static_assert(DIMENSION > 0, "Dimension must be greater than zero.");
        update();
    }

    virtual ~PointCloud() {}

    Point<DIMENSION>& operator[](size_t index)
    {
        return mPoints[index];
    }

    const Point<DIMENSION>& at(size_t index) const
    {
        return mPoints[index];
    }

    size_t size() const
    {
        return mPoints.size();
    }

    Vector center() const
    {
        return mCenter;
    }

    float maxSize() const { return mMaxSize; }
    const Vector& bottomLeft() const { return mBottomLeft; }
    const Vector& topRight() const { return mTopRight; }
    Vector extensionCenter() const { return (mBottomLeft + mTopRight) / 2; }

    void update()
    {
        calculateCenter();
        calculateExtension();
    }

    void calculateCenter()
    {
        if (mPoints.empty()) mCenter = Point<DIMENSION>::Vector::Zero();
        else
        {
            mCenter = Vector::Zero();
            for (const Point<DIMENSION> &point : mPoints)
            {
                mCenter += point.position();
            }
            mCenter /= mPoints.size();
        }
    }

    void calculateExtension()
    {
        Vector min;
        Vector max;
        float maxValue = std::numeric_limits<float>::max();
        min = Vector::Constant(maxValue);
        max = Vector::Constant(-maxValue);
        for (const Point<DIMENSION> &point : mPoints)
        {
            for (unsigned int i = 0; i < DIMENSION; i++)
            {
                min(i) = std::min(min(i), point.position()(i));
                max(i) = std::max(max(i), point.position()(i));
            }
        }

        mBottomLeft = min;
        mTopRight = max;
        float maxSize = 0;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            maxSize = std::max(maxSize, max(dim) - min(dim));
        }
        mMaxSize = maxSize;
    }

protected:
    std::vector<Point<DIMENSION> > mPoints;
    Vector mCenter;
    Vector mBottomLeft;
    Vector mTopRight;
    float mMaxSize;

};

using PointCloud3d = PointCloud<3>;

#endif // POINTCLOUD_H
