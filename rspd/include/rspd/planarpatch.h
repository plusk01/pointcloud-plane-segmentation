#ifndef PLANEDETECTORPATCH_H
#define PLANEDETECTORPATCH_H

#include <memory>
#include <unordered_set>
#include <unordered_map>

#include <Eigen/Core>
#include "plane.h"
// #include "pointcloud.h"
#include "statisticsutils.h"

#include <open3d/Open3D.h>

struct RotatedRect
{
    Eigen::Matrix3Xd matrix;
    Eigen::Matrix3d basis;
    double area;
    Eigen::Vector3d bottomLeft;
    Eigen::Vector3d topRight;
    Eigen::Matrix3d R_12;

    RotatedRect() : area(std::numeric_limits<double>::max()) {}

    RotatedRect(const Eigen::Matrix3Xd &matrix, const Eigen::Matrix3d &basis, double degrees)
    {
        Eigen::Matrix3d R;
        R_12 = Eigen::AngleAxisd(degrees * 3.14159 / 180., Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d newBasis = basis * R_12;
        Eigen::Matrix3Xd newMatrix = newBasis.transpose() * matrix;
        Eigen::Vector3d max = newMatrix.rowwise().maxCoeff();
        Eigen::Vector3d min = newMatrix.rowwise().minCoeff();
        double w = max(0) - min(0);
        double h = max(1) - min(1);
        double area = w * h;
        this->matrix = newMatrix;
        this->area = area;
        this->basis = newBasis;
        this->bottomLeft = min;
        this->topRight = max;
        this->R_12 = R_12;
    }

};

class PlanarPatch
{
public:
    using PointCloudConstPtr = std::shared_ptr<const open3d::geometry::PointCloud>;
    PlanarPatch(const PointCloudConstPtr& mPointCloud, StatisticsUtils *statistics,
                const std::vector<size_t> &mPoints, double minAllowedNormal,
                double maxAllowedDist, double outlierRatio);

    size_t index() const
    {
        return mIndex;
    }

    void index(size_t index)
    {
        mIndex = index;
    }

    inline bool isInlier(size_t point) const
    {
        return std::abs(mPlane.normal().dot(mPointCloud->normals_[point])) > mMinNormalDiff &&
                std::abs(mPlane.getSignedDistanceFromSurface(mPointCloud->points_[point])) < mMaxDistPlane;
    }

    inline bool isVisited(size_t point) const
    {
        if (!mVisited2.empty()) {
            return mVisited2[point];
        }
        return mVisited.find(point) != mVisited.end();
    }

    inline void visit(size_t point)
    {
        if (!mVisited2.empty()) {
            mVisited2[point] = true;
        } else {
            mVisited.insert(point);
        }
    }

    inline void addPoint(size_t point)
    {
        mPoints.push_back(point);
        ++mNumNewPoints;
        mOutliers[point] = false;
    }

    bool isPlanar();

    void updatePlane();

    const std::vector<size_t>& points() const
    {
        return mPoints;
    }

    void points(const std::vector<size_t> &points)
    {
        mPoints = points;
    }

    const Plane& plane() const
    {
        return mPlane;
    }

    void plane(const Plane &plane)
    {
        mPlane = plane;
    }

    const Eigen::Vector3d center() const
    {
        return mPlane.center();
    }

    void center(const Eigen::Vector3d &center)
    {
        mPlane.center(center);
    }

    const Eigen::Vector3d normal() const
    {
        return mPlane.normal();
    }

    void normal(const Eigen::Vector3d &normal)
    {
        mPlane.normal(normal);
    }

    double maxDistPlane() const
    {
        return mMaxDistPlane;
    }

    void maxDistPlane(double maxDistPlane)
    {
        mMaxDistPlane = maxDistPlane;
    }

    double minNormalDiff() const
    {
        return mMinNormalDiff;
    }

    void minNormalDiff(double minNormalDiff)
    {
        mMinNormalDiff = minNormalDiff;
    }

    RotatedRect& rect()
    {
        return mRect;
    }

    size_t& numNewPoints()
    {
        return mNumNewPoints;
    }

    bool& stable()
    {
        return mStable;
    }

    size_t numUpdates() const
    {
        return mNumUpdates;
    }

    size_t originalSize() const
    {
        return mOriginalSize;
    }

    void removeOutliers();

    double getSize() const;

private:
    PointCloudConstPtr mPointCloud;
    StatisticsUtils *mStatistics;
    std::vector<size_t> mPoints;
    double mOriginalSize;
    size_t mIndex;
    Plane mPlane;
    double mMaxDistPlane;
    double mMinNormalDiff;
    RotatedRect mRect;
    size_t mNumNewPoints;
    size_t mNumUpdates;
    std::unordered_set<size_t> mVisited;
    std::vector<bool> mVisited2;
    std::unordered_map<size_t, bool> mOutliers;
    bool mStable;
    double mMinAllowedNormal;
    double mMaxAllowedDist;
    double mOutlierRatio;
    bool mUsedVisited2;

    bool isNormalValid() const;

    bool isDistValid() const;

    double getMaxPlaneDist();

    double getMinNormalDiff();

    Plane getPlane();

};

#endif // PLANEDETECTORPATCH_H
