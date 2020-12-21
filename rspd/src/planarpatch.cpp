#include "rspd/planarpatch.h"

#include <iostream>

PlanarPatch::PlanarPatch(const PointCloudConstPtr& pointCloud, StatisticsUtils *statistics,
                         const std::vector<size_t> &points, double minAllowedNormal,
                         double maxAllowedDist, double outlierRatio)
    : mPointCloud(pointCloud)
    , mStatistics(statistics)
    , mPoints(points)
    , mOriginalSize(getSize())
    , mNumNewPoints(0)
    , mNumUpdates(0)
    , mStable(false)
    , mMinAllowedNormal(minAllowedNormal)
    , mMaxAllowedDist(maxAllowedDist)
    , mOutlierRatio(outlierRatio)
    , mUsedVisited2(false)
{

}

double PlanarPatch::getSize() const
{
    Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d max = -min;
    for (const size_t &point : mPoints)
    {
        Eigen::Vector3d position = mPointCloud->points_[point];
        for (size_t i = 0; i < 3; i++)
        {
            min(i) = std::min(min(i), position(i));
            max(i) = std::max(max(i), position(i));
        }
    }

    double maxSize = 0;
    for (size_t dim = 0; dim < 3; dim++) {
        maxSize = std::max(maxSize, max(dim) - min(dim));
    }
    return maxSize;
}

Plane PlanarPatch::getPlane()
{
    mStatistics->size(mPoints.size());
    Eigen::Vector3d center;
    for (size_t dim = 0; dim < 3; dim++)
    {
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            mStatistics->dataBuffer()[i] = mPointCloud->points_[mPoints[i]](dim);
        }
        center(dim) = mStatistics->getMedian();
    }
    Eigen::Vector3d normal;
    for (size_t dim = 0; dim < 3; dim++)
    {
        for (size_t i = 0; i < mPoints.size(); i++)
        {
            mStatistics->dataBuffer()[i] = mPointCloud->normals_[mPoints[i]](dim);
        }
        normal(dim) = mStatistics->getMedian();
    }
    normal = normal.normalized();
    return Plane(center, normal);
}

double PlanarPatch::getMaxPlaneDist()
{
    mStatistics->size(mPoints.size());
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        const Eigen::Vector3d &position = mPointCloud->points_[mPoints[i]];
        mStatistics->dataBuffer()[i] = std::abs(mPlane.getSignedDistanceFromSurface(position));
    }
    double minDist, maxDist;
    mStatistics->getMinMaxRScore(minDist, maxDist, 3);
    return maxDist;
}

double PlanarPatch::getMinNormalDiff()
{
    mStatistics->size(mPoints.size());
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        const Eigen::Vector3d &normal = mPointCloud->normals_[mPoints[i]];
        mStatistics->dataBuffer()[i] = std::abs(normal.dot(mPlane.normal()));
    }
    double minDiff, maxDiff;
    mStatistics->getMinMaxRScore(minDiff, maxDiff, 3);
    return minDiff;
}

bool PlanarPatch::isNormalValid() const
{
    return mMinNormalDiff > mMinAllowedNormal;
}

bool PlanarPatch::isDistValid() const
{
    Eigen::Vector3d basisU, basisV;
    GeometryUtils::orthogonalBasis(mPlane.normal(), basisU, basisV);
    Eigen::Vector3d extreme = basisU * mOriginalSize + mPlane.normal() * mMaxDistPlane;
    return std::abs(extreme.normalized().dot(mPlane.normal())) < mMaxAllowedDist;
}

bool PlanarPatch::isPlanar()
{
    mPlane = getPlane();

    mMinNormalDiff = getMinNormalDiff();
    if (!isNormalValid()) return false;
    size_t countOutliers = 0;
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        bool outlier = mStatistics->dataBuffer()[i] < mMinNormalDiff;
        mOutliers[mPoints[i]] = outlier;
        countOutliers += outlier;
    }
    if (countOutliers > mPoints.size() * mOutlierRatio) return false;

    mMaxDistPlane = getMaxPlaneDist();
    if (!isDistValid()) return false;
    countOutliers = 0;
    for (size_t i = 0; i < mPoints.size(); i++)
    {
        bool outlier = mOutliers[mPoints[i]] || mStatistics->dataBuffer()[i] > mMaxDistPlane;
        mOutliers[mPoints[i]] = outlier;
        countOutliers += outlier;
    }
    if (countOutliers < mPoints.size() * mOutlierRatio)
    {
        removeOutliers();
        return true;
    }
    return false;
}

void PlanarPatch::updatePlane()
{
    mPlane = getPlane();
    mVisited.clear();
    if (mNumUpdates > 1)
    {
        mUsedVisited2 = true;
    }
    if (mUsedVisited2) {
        if (mVisited2.empty()) {
            mVisited2 = std::vector<bool>(mPointCloud->points_.size(), false);
        } else {
            std::fill(mVisited2.begin(), mVisited2.end(), false);
        }
    }
    mNumNewPoints = 0;
    ++mNumUpdates;
}

void PlanarPatch::removeOutliers()
{
    mPoints.erase(std::remove_if(mPoints.begin(), mPoints.end(), [this](const size_t &point) {
        return mOutliers[point];
    }), mPoints.end());
    mOutliers.clear();
}
