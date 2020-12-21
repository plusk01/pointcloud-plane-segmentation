#ifndef PLANEDETECTOR_H
#define PLANEDETECTOR_H

#include "planarpatch.h"
#include "boundaryvolumehierarchy.h"

#include <open3d/Open3D.h>

class PlaneDetector
{
public:
    using PointCloudConstPtr = std::shared_ptr<const open3d::geometry::PointCloud>;
    PlaneDetector(const PointCloudConstPtr& pointCloud, std::vector<std::vector<int>>& neighbors);

    void delimitPlane(PlanarPatch *patch);

    void delimitPlane(Plane *plane);

    double minNormalDiff() const
    {
        return mMinNormalDiff;
    }

    void minNormalDiff(double minNormalDiff)
    {
        mMinNormalDiff = minNormalDiff;
    }

    double maxDist() const
    {
        return mMaxDist;
    }

    void maxDist(double maxDist)
    {
        mMaxDist = maxDist;
    }

    double outlierRatio() const
    {
        return mOutlierRatio;
    }

    void outlierRatio(double outlierRatio)
    {
        mOutlierRatio = outlierRatio;
    }

    std::set<Plane*> detect();

    const PointCloudConstPtr& pointCloud() const
    {
        return mPointCloud;
    }

private:
    PointCloudConstPtr mPointCloud;
    std::vector<std::vector<int>> mNeighbors;

    Eigen::Vector3d mBottomLeft;
    Eigen::Vector3d mTopRight;
    Eigen::Vector3d mExtCenter;
    double mMaxSize; // pointcloud rect extension max size

    std::vector<PlanarPatch*> mPatchPoints;
    double mMinNormalDiff;
    double mMaxDist;
    double mOutlierRatio;

    bool detectPlanarPatches(BVH3d *node, StatisticsUtils *statistics, size_t minNumPoints, std::vector<PlanarPatch*> &patches);

    void growPatches(std::vector<PlanarPatch*> &patches, bool relaxed = false);

    void mergePatches(std::vector<PlanarPatch*> &patches);

    bool updatePatches(std::vector<PlanarPatch*> &patches);

    void getPlaneOutlier(const PlanarPatch *patch, std::vector<size_t> &outlier);

    bool isFalsePositive(PlanarPatch *patch);

};

#endif // PLANEDETECTOR_H
