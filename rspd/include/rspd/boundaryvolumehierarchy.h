#ifndef BVH3d_H
#define BVH3d_H

#include <algorithm>
#include <numeric>
#include <set>
#include <queue>

#include <open3d/Open3D.h>

class BVH3d
{
public:
    using PointCloudConstPtr = std::shared_ptr<const open3d::geometry::PointCloud>;
    using Vector = Eigen::Vector3d;
    static constexpr size_t NUM_CHILDREN = 1 << 3;

    BVH3d(const PointCloudConstPtr& pointCloud)
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
    }

    BVH3d(const BVH3d &bvh) = delete;

    ~BVH3d()
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

            mLeaf = false;

            // create children
            double newSize = mSize / 2;
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                mChildren[i] = NULL;
            }

            // split points
            for (const size_t &index : mIndices)
            {
                // calculate child index comparing position to child center
                size_t childIndex = calculateChildIndex(this->pointCloud()->points_[index]);
                if (mChildren[childIndex] == NULL)
                {
                    if (this->pointCloud()->points_[index].array().sum() == 0) {
                        std::cout << "zero!" << std::endl;
                    }
                    mChildren[childIndex] = new BVH3d(this, newCenters[childIndex], newSize);
                    mChildren[childIndex]->mIndices.reserve(mIndices.size());
                }
                mChildren[childIndex]->mIndices.push_back(index);
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

    BVH3d* child(size_t index)
    {
        return mChildren[index];
    }

    std::vector<const BVH3d*> children() const
    {
        if (isLeaf()) return std::vector<const BVH3d*>();
        std::vector<const BVH3d*> children;
        for (size_t i = 0; i < NUM_CHILDREN; i++)
        {
            if (mChildren[i] != NULL)
                children.push_back(mChildren[i]);
        }
        return children;
    }

    const BVH3d* parent() const
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
    BVH3d *mRoot;
    BVH3d *mParent;
    BVH3d *mChildren[NUM_CHILDREN];
    Vector mCenter;
    double mSize;
    bool mLeaf;
    size_t mLevel;
    std::vector<size_t> mIndices;

    BVH3d(BVH3d *parent, const Vector &center, double size)
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
        for (size_t dim = 0; dim < 3; dim++)
        {
            for (size_t i = 0; i < NUM_CHILDREN; i++)
            {
                int signal = (((i & (1 << (3 - dim - 1))) >> (3 - dim - 1)) << 1) - 1;
                centers[i](dim) = mCenter(dim) + newSize * signal;
            }
        }
    }

    size_t calculateChildIndex(const Vector &position)
    {
        size_t childIndex = 0;
        for (size_t dim = 0; dim < 3; dim++)
        {
            childIndex |= (position(dim) > mCenter(dim)) << (3 - dim - 1);
        }
        return childIndex;
    }

};

#endif // BVH3d_H
