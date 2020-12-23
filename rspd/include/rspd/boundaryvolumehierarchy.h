#ifndef BOUNDARYVOLUMEHIERARCHY_H
#define BOUNDARYVOLUMEHIERARCHY_H

#include <algorithm>
#include <numeric>
#include <set>
#include <queue>

#include <open3d/Open3D.h>

template <size_t DIMENSION>
class BoundaryVolumeHierarchy
{
public:
    using PointCloudConstPtr = std::shared_ptr<const open3d::geometry::PointCloud>;
    using Vector = Eigen::Vector3d;
    static constexpr size_t NUM_CHILDREN = 1 << DIMENSION;

    BoundaryVolumeHierarchy(const PointCloudConstPtr& pointCloud)
        : mPointCloud(pointCloud)
        , mRoot(this)
        , mParent(this)
        , mLeaf(true)
        , mLevel(0)
    {

        const Vector min = pointCloud->GetMinBound();
        const Vector max = pointCloud->GetMaxBound();
        mCenter = (min + max) / 2;

        double maxSize = 0;
        for (size_t dim = 0; dim<3; dim++) {
            maxSize = std::max(maxSize, max(dim) - min(dim));
        }

        mSize = maxSize / 2;
        mIndices = std::vector<size_t>(pointCloud->points_.size());
        std::iota(mIndices.begin(), mIndices.end(), 0);
        // mLeafTable = std::vector<BoundaryVolumeHierarchy<DIMENSION>*>(pointCloud->points_.size(), this);
    }

    BoundaryVolumeHierarchy(const BoundaryVolumeHierarchy &bvh) = delete;

    ~BoundaryVolumeHierarchy()
    {
        if (!isLeaf())
        {
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                {
                    delete mChildren[i];
                }
            }
        }
    }

    const PointCloudConstPtr& pointCloud() const
    {
        return mPointCloud;
    }

    void partition(size_t levels = 1, size_t minNumPoints = 1, double minSize = 0.0f)
    {
        if (!isLeaf())
        {
            open3d::utility::LogWarning("not isLeaf on level {}", mLevel);
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                    mChildren[i]->partition(levels - 1, minNumPoints, minSize);
            }
        }
        else
        {
            if (levels <= 0 || mIndices.size() <= minNumPoints || mIndices.size() <= 1 || mSize < minSize) return;
            // create new centers
            Vector newCenters[NUM_CHILDREN];
            calculateNewCenters(newCenters);

            // std::cout << "---" << std::endl;
            // std::cout << "Current center: " << mCenter.transpose() << std::endl;
            // std::cout << "New centers: " << std::endl;
            // for (size_t i=0; i<NUM_CHILDREN; i++) {
            //     std::cout << "\t-" << i << ": " << newCenters[i].transpose() << std::endl;
            // }
            // std::cout << "---" << std::endl;

            mLeaf = false;

            // create children
            double newSize = mSize / 2;
            // open3d::utility::LogWarning("new centers (with newSize {}) on level {}", newSize, mLevel);
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                mChildren[i] = NULL;
            }

            // split points
            for (const size_t &index : mIndices)
            {
                // calculate child index comparing position to child center
                size_t childIndex = calculateChildIndex(this->pointCloud()->points_[index]);
                // open3d::utility::LogWarning("Level {}, index {}, childIndex {}", mLevel, index, childIndex);
                if (mChildren[childIndex] == NULL)
                {
                    if (this->pointCloud()->points_[index].array().sum() == 0) {
                        std::cout << "zero!" << std::endl;
                    }
                    mChildren[childIndex] = new BoundaryVolumeHierarchy<DIMENSION>(this, newCenters[childIndex], newSize);
                    mChildren[childIndex]->mIndices.reserve(mIndices.size());
                }
                mChildren[childIndex]->mIndices.push_back(index);
                // update current leaf where point is stored
                // mRoot->mLeafTable[index] = mChildren[childIndex];
            }

            mIndices.clear();

            // partition recursively
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL) {
                    mChildren[i]->partition(levels - 1, minNumPoints, minSize);
                }
            }
        }
    }

    BoundaryVolumeHierarchy<DIMENSION>* child(size_t index)
    {
        return mChildren[index];
    }

    std::vector<const BoundaryVolumeHierarchy<DIMENSION>*> children() const
    {
        if (isLeaf()) return std::vector<const BoundaryVolumeHierarchy<DIMENSION>*>();
        std::vector<const BoundaryVolumeHierarchy<DIMENSION>*> children;
        for (size_t i = 0; i < NUM_CHILDREN; i++)
        {
            if (mChildren[i] != NULL)
                children.push_back(mChildren[i]);
        }
        return children;
    }

    const BoundaryVolumeHierarchy<DIMENSION>* parent() const
    {
        return mParent;
    }

    bool isRoot() const
    {
        return this == mRoot;
    }

    bool isLeaf() const
    {
        return mLeaf;
    }

    size_t level() const
    {
        return mLevel;
    }

    size_t numPoints() const
    {
        if (isLeaf())
        {
            return mIndices.size();
        }
        else
        {
            size_t numPoints = 0;
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                    numPoints += mChildren[i]->numPoints();
            }
            return numPoints;
        }
    }

    std::vector<size_t> points() const
    {
        if (isLeaf())
        {
            return mIndices;
        }
        else
        {
            std::vector<size_t> points;
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                if (mChildren[i] != NULL)
                {
                    std::vector<size_t> childPoints = mChildren[i]->points();
                    points.insert(points.end(), childPoints.begin(), childPoints.end());
                }
            }
            return points;
        }
    }

private:
    PointCloudConstPtr mPointCloud;
    BoundaryVolumeHierarchy *mRoot;
    BoundaryVolumeHierarchy *mParent;
    BoundaryVolumeHierarchy *mChildren[NUM_CHILDREN];
    Vector mCenter;
    double mSize;
    bool mLeaf;
    size_t mLevel;
    std::vector<size_t> mIndices;
    // std::vector<BoundaryVolumeHierarchy<DIMENSION>*> mLeafTable;

    BoundaryVolumeHierarchy(BoundaryVolumeHierarchy *parent, const Vector &center, double size)
        : mPointCloud(parent->pointCloud())
        , mRoot(parent->mRoot)
        , mParent(parent)
        , mCenter(center)
        , mSize(size)
        , mLeaf(true)
        , mLevel(parent->mLevel + 1)
    {
        for (size_t i = 0; i < NUM_CHILDREN; i++)
        {
            mChildren[i] = NULL;
        }
    }

    void calculateNewCenters(Vector centers[NUM_CHILDREN])
    {
        double newSize = mSize / 2;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                int signal = (((i & (1 << (DIMENSION - dim - 1))) >> (DIMENSION - dim - 1)) << 1) - 1;
                centers[i](dim) = mCenter(dim) + newSize * signal;
            }
        }
    }

    size_t calculateChildIndex(const Vector &position)
    {
        size_t childIndex = 0;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            childIndex |= (position(dim) > mCenter(dim)) << (DIMENSION - dim - 1);
        }
        return childIndex;
    }

};

using BVH3d = BoundaryVolumeHierarchy<3>;

#endif // BOUNDARYVOLUMEHIERARCHY_H
