#ifndef PLANEDETECTOR_H
#define PLANEDETECTOR_H

#include "pointcloud.h"
#include "planarpatch.h"
#include "boundaryvolumehierarchy.h"

class PlaneDetector
{
public:
    PlaneDetector(const PointCloud3d *pointCloud);

    void delimitPlane(PlanarPatch *patch);

    void delimitPlane(Plane *plane);

    float minNormalDiff() const
    {
        return mMinNormalDiff;
    }

    void minNormalDiff(float minNormalDiff)
    {
        mMinNormalDiff = minNormalDiff;
    }

    float maxDist() const
    {
        return mMaxDist;
    }

    void maxDist(float maxDist)
    {
        mMaxDist = maxDist;
    }

    float outlierRatio() const
    {
        return mOutlierRatio;
    }

    void outlierRatio(float outlierRatio)
    {
        mOutlierRatio = outlierRatio;
    }

    std::set<Plane*> detect();

    const PointCloud3d* pointCloud() const
    {
        return mPointCloud;
    }

private:
    const PointCloud3d *mPointCloud;

    std::vector<PlanarPatch*> mPatchPoints;
    float mMinNormalDiff;
    float mMaxDist;
    float mOutlierRatio;

    bool detectPlanarPatches(Octree *node, StatisticsUtils *statistics, size_t minNumPoints, std::vector<PlanarPatch*> &patches);

    void growPatches(std::vector<PlanarPatch*> &patches, bool relaxed = false);

    void mergePatches(std::vector<PlanarPatch*> &patches);

    bool updatePatches(std::vector<PlanarPatch*> &patches);

    void getPlaneOutlier(const PlanarPatch *patch, std::vector<size_t> &outlier);

    bool isFalsePositive(PlanarPatch *patch);

};

#endif // PLANEDETECTOR_H
