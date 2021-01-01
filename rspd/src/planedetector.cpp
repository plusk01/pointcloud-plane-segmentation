#include "rspd/planedetector.h"

#include "rspd/unionfind.h"

#include <chrono>
#include <iostream>
#include <unordered_map>
#include <queue>

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#include <open3d/Open3D.h>

inline static double deg2rad(double deg)
{
    return static_cast<double>(deg * M_PI / 180);
}

PlaneDetector::PlaneDetector(const PointCloudConstPtr& pointCloud, std::vector<std::vector<int>>& neighbors)
    : mPointCloud(pointCloud)
    , mMinNormalDiff(std::cos(deg2rad(60.0f)))
    , mMaxDist(std::cos(deg2rad(75.0f)))
    , mOutlierRatio(0.75f)
{
    mNeighbors.swap(neighbors);

    mBottomLeft = pointCloud->GetMinBound();
    mTopRight = pointCloud->GetMaxBound();
    mExtCenter = (mBottomLeft + mTopRight) / 2;

    double maxSize = 0;
    for (size_t dim = 0; dim<3; dim++) {
        maxSize = std::max(maxSize, mTopRight(dim) - mBottomLeft(dim));
    }
    mMaxSize = maxSize;
}

std::set<Plane*> PlaneDetector::detect()
{
    double timeDetectPatches = 0;
    double timeMerge = 0;
    double timeGrowth = 0;
    double timeRelaxedGrowth = 0;
    double timeUpdate = 0;
    double timeDelimit = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    size_t minNumPoints = 30; //std::max(size_t(10), size_t(pointCloud()->size() * 0.001f));
    // size_t minNumPoints = std::max(size_t(10), size_t(pointCloud()->size() * 0.001f));
    StatisticsUtils statistics(pointCloud()->points_.size());
    BVH3d octree(pointCloud());
    std::vector<PlanarPatch*> patches;
    detectPlanarPatches(&octree, &statistics, minNumPoints, patches);
    timeDetectPatches += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();

    mPatchPoints = std::vector<PlanarPatch*>(pointCloud()->points_.size(), NULL);
    for (PlanarPatch *patch : patches)
    {
        for (const size_t &point : patch->points())
        {
            mPatchPoints[point] = patch;
        }
    }

    std::cout << "#" << patches.size() << std::endl;
    bool changed;
    do
    {
        t1 = std::chrono::high_resolution_clock::now();
        growPatches(patches);
        timeGrowth += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();

        t1 = std::chrono::high_resolution_clock::now();
        mergePatches(patches);
        timeMerge += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
        std::cout << "#" << patches.size() << std::endl;

        t1 = std::chrono::high_resolution_clock::now();
        changed = updatePatches(patches);
        timeUpdate += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();
    } while (changed);

    t1 = std::chrono::high_resolution_clock::now();
    growPatches(patches, true);
    timeRelaxedGrowth += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();

    t1 = std::chrono::high_resolution_clock::now();
    std::vector<PlanarPatch*> truePositivePatches;
    for (PlanarPatch *patch : patches)
    {
        delimitPlane(patch);
        if (isFalsePositive(patch))
        {
            delete patch;
        }
        else
        {
            truePositivePatches.push_back(patch);
        }
    }
    patches = truePositivePatches;
    std::cout << "#" << patches.size() << std::endl;
    timeDelimit += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t1).count();

    std::set<Plane*> planes;
    for (PlanarPatch *patch : patches)
    {
        Plane *plane = new Plane(patch->plane());
        plane->inliers(patch->points());
        planes.insert(plane);
        delete patch;
    }

    std::cout << "#" << planes.size() << std::endl;
    std::cout << "Detect planar patches - Time elapsed: " << timeDetectPatches << "s" <<  std::endl;
    std::cout << "Merge patches - Time elapsed: " << timeMerge << "s" <<  std::endl;
    std::cout << "Grow patches - Time elapsed: " << timeGrowth << "s" <<  std::endl;
    std::cout << "Relaxed Grow patches - Time elapsed: " << timeRelaxedGrowth << "s" <<  std::endl;
    std::cout << "Update - Time elapsed: " << timeUpdate << "s" <<  std::endl;
    std::cout << "Total time elapsed: " << timeDetectPatches + timeMerge + timeGrowth + timeRelaxedGrowth + timeUpdate << "s" << std::endl;
    std::cout << "Delimit planes - Time elapsed: " << timeDelimit << "s" <<  std::endl;

    return planes;
}

bool PlaneDetector::detectPlanarPatches(BVH3d *node, StatisticsUtils *statistics, size_t minNumPoints, std::vector<PlanarPatch*> &patches)
{
    if (node->numPoints() < minNumPoints) return false;
    node->partition(1, minNumPoints);
    bool iHavePlanarPatch = false;
    bool childCouldHavePlanarPatch = false;
    for (size_t i = 0; i < 8; i++)
    {
        if (node->child(i) != NULL && detectPlanarPatches(node->child(i), statistics, minNumPoints, patches))
        {
            childCouldHavePlanarPatch = true;
        }
    }
    if (!childCouldHavePlanarPatch && node->level() > 2)
    {
        PlanarPatch *patch = new PlanarPatch(pointCloud(), statistics, node->points(), mMinNormalDiff, mMaxDist, mOutlierRatio);
        if (patch->isPlanar())
        {
            patches.push_back(patch);
            iHavePlanarPatch = true;
        }
        else
        {
            delete patch;
        }
    }
    return iHavePlanarPatch || childCouldHavePlanarPatch;
}

void PlaneDetector::growPatches(std::vector<PlanarPatch*> &patches, bool relaxed)
{
    std::sort(patches.begin(), patches.end(), [](const PlanarPatch *a, const PlanarPatch *b) {
       return a->minNormalDiff() > b->minNormalDiff();
    });
    std::queue<size_t> queue;
    for (PlanarPatch *patch : patches)
    {
        if (patch->stable()) continue;
        for (const size_t &point : patch->points())
        {
            queue.push(point);
        }
        while (!queue.empty())
        {
            size_t point = queue.front();
            queue.pop();
            for (int neighbor : mNeighbors[point])
            {
                if (mPatchPoints[neighbor] != NULL || (!relaxed && patch->isVisited(neighbor))) continue;
                if ((!relaxed && patch->isInlier(neighbor)) || (relaxed && std::abs(patch->plane().getSignedDistanceFromSurface(pointCloud()->points_[neighbor])) < patch->maxDistPlane()))
                {
                    queue.push(neighbor);
                    patch->addPoint(neighbor);
                    mPatchPoints[neighbor] = patch;
                }
                else
                {
                    patch->visit(neighbor);
                }
            }
        }
    }
}


void PlaneDetector::mergePatches(std::vector<PlanarPatch*> &patches)
{
    size_t n = patches.size();
    for (size_t i = 0; i < n; i++)
    {
        patches[i]->index(i);
    }
    std::vector<bool> graph(n * n, false);
    std::vector<bool> disconnectedPatches(n * n, false);
    for (size_t i = 0; i < patches.size(); i++)
    {
        for (size_t j = i + 1; j < patches.size(); j++)
        {
            double normalThreshold = std::min(patches[i]->minNormalDiff(), patches[j]->minNormalDiff());
            disconnectedPatches[i * n + j] = std::abs(patches[i]->plane().normal().dot(patches[j]->plane().normal())) < normalThreshold;
            disconnectedPatches[j * n + i] = disconnectedPatches[i * n + j];
        }
    }
    for (PlanarPatch *p : patches)
    {
        for (const size_t &point : p->points())
        {
            for (int neighbor : mNeighbors[point])
            {
                PlanarPatch *np = mPatchPoints[neighbor];
                if (p == np || np == NULL || graph[np->index() * n + p->index()] || graph[p->index() * n + np->index()] ||
                    disconnectedPatches[p->index() * n + np->index()] || p->isVisited(neighbor) || np->isVisited(point)) continue;
                p->visit(neighbor);
                np->visit(point);
                const Eigen::Vector3d &p1 = pointCloud()->points_[point];
                const Eigen::Vector3d &n1 = pointCloud()->normals_[point];
                const Eigen::Vector3d &p2 = pointCloud()->points_[neighbor];
                const Eigen::Vector3d &n2 = pointCloud()->normals_[neighbor];
                double distThreshold = std::max(p->maxDistPlane(), np->maxDistPlane());
                double normalThreshold = std::min(p->minNormalDiff(), np->minNormalDiff());
                graph[p->index() * n + np->index()] = std::abs(p->plane().normal().dot(n2)) > normalThreshold &&
                        std::abs(np->plane().normal().dot(n1)) > normalThreshold &&
                        std::abs(p->plane().getSignedDistanceFromSurface(p2)) < distThreshold &&
                        std::abs(np->plane().getSignedDistanceFromSurface(p1)) < distThreshold;
            }
        }
    }
    UnionFind uf(patches.size());
    for (size_t i = 0; i < patches.size(); i++)
    {
        for (size_t j = i + 1; j < patches.size(); j++)
        {
            if (graph[i * n + j] || graph[j * n + i])
            {
                uf.join(i, j);
            }
        }
    }
    std::vector<size_t> largestPatch(patches.size());
    std::iota(largestPatch.begin(), largestPatch.end(), 0);
    for (size_t i = 0; i < patches.size(); i++)
    {
        int root = uf.root(i);
        if (patches[largestPatch[root]]->points().size() < patches[i]->points().size())
        {
            largestPatch[root] = i;
        }
    }
    for (size_t i = 0; i < patches.size(); i++)
    {
        size_t root = largestPatch[uf.root(i)];
        if (root != i)
        {
            for (const size_t &point : patches[i]->points())
            {
                patches[root]->addPoint(point);
                mPatchPoints[point] = patches[root];
            }
            patches[root]->maxDistPlane(std::max(patches[root]->maxDistPlane(), patches[i]->maxDistPlane()));
            patches[root]->minNormalDiff(std::min(patches[root]->minNormalDiff(), patches[i]->minNormalDiff()));
            delete patches[i];
            patches[i] = NULL;
        }
    }
    patches.erase(std::remove_if(patches.begin(), patches.end(), [](PlanarPatch *patch) {
        return patch == NULL;
    }), patches.end());
}

bool PlaneDetector::updatePatches(std::vector<PlanarPatch*> &patches)
{
    bool changed = false;
    for (PlanarPatch *patch : patches)
    {
        if (patch->numNewPoints() > (patch->points().size() - patch->numNewPoints()) / 2)
        {
            patch->updatePlane();
            patch->stable() = false;
            changed = true;
        }
        else
        {
            patch->stable() = true;
        }
    }
    return changed;
}

void PlaneDetector::getPlaneOutlier(const PlanarPatch *patch, std::vector<size_t> &outlier)
{
    Eigen::Vector3d basisU, basisV;
    GeometryUtils::orthogonalBasis(patch->plane().normal(), basisU, basisV);
    std::vector<Eigen::Vector2d> projectedPoints(patch->points().size());
    for (size_t i = 0; i < patch->points().size(); i++)
    {
        Eigen::Vector3d position = pointCloud()->points_[patch->points()[i]];
        projectedPoints[i] = GeometryUtils::projectOntoOrthogonalBasis(position, basisU, basisV);
    }
    GeometryUtils::convexHull(projectedPoints, outlier);
    for (size_t i = 0; i < outlier.size(); i++)
    {
        outlier[i] = patch->points()[outlier[i]];
    }
}

void PlaneDetector::delimitPlane(PlanarPatch *patch)
{
    std::vector<size_t> outlier;
    getPlaneOutlier(patch, outlier);
    Eigen::Vector3d normal = patch->plane().normal();
    Eigen::Vector3d basisU, basisV;
    GeometryUtils::orthogonalBasis(normal, basisU, basisV);
    Eigen::Matrix3d basis;
    basis << basisU, basisV, normal;
    Eigen::Matrix3Xd matrix(3, outlier.size());
    for (size_t i = 0; i < outlier.size(); i++)
    {
        matrix.col(i) = pointCloud()->points_[outlier[i]];
    }
    double minAngle = 0;
    double maxAngle = 90;
    while (maxAngle - minAngle > 5)
    {
        double mid = (maxAngle + minAngle) / 2;
        double left = (minAngle + mid) / 2;
        double right = (maxAngle + mid) / 2;
        RotatedRect leftRect(matrix, basis, left);
        RotatedRect rightRect(matrix, basis, right);
        if (leftRect.area < rightRect.area)
        {
            maxAngle = mid;
        }
        else
        {
            minAngle = mid;
        }
    }
    patch->rect() = RotatedRect(matrix, basis, (minAngle + maxAngle) / 2);
    Eigen::Vector3d center = patch->plane().center();
    Eigen::Vector3d minBasisU = patch->rect().basis.col(0);
    Eigen::Vector3d minBasisV = patch->rect().basis.col(1);
    center -= minBasisU * minBasisU.dot(center);
    center -= minBasisV * minBasisV.dot(center);
    center += minBasisU * (patch->rect().bottomLeft(0) + patch->rect().topRight(0)) / 2;
    center += minBasisV * (patch->rect().bottomLeft(1) + patch->rect().topRight(1)) / 2;
    double lengthU = (patch->rect().topRight(0) - patch->rect().bottomLeft(0)) / 2;
    double lengthV = (patch->rect().topRight(1) - patch->rect().bottomLeft(1)) / 2;
    Plane newPlane(center, patch->plane().normal(), minBasisU * lengthU, minBasisV * lengthV);
    patch->plane(newPlane);
    // if (patch->rect().area > 20) {
    //     std::cout << std::endl << std::endl;
    //     std::cout << "----------------------------------------------------------" << std::endl;
    //     std::cout << "basis: " << std::endl << basis << std::endl << std::endl;
    //     std::cout << "matrix: " << std::endl << matrix << std::endl << std::endl;
    //     std::cout << "R: " << std::endl << patch->rect().R << std::endl << std::endl;
    //     std::cout << "R_12: " << std::endl << patch->rect().R_12 << std::endl << std::endl;
    //     std::cout << "new basis: " << std::endl << patch->rect().basis << std::endl << std::endl;
    //     std::cout << "R * basis: " << std::endl << patch->rect().R * basis << std::endl << std::endl;
    //     std::cout << "new matrix: " << std::endl << patch->rect().matrix << std::endl << std::endl;
    //     std::cout << "area: " << patch->rect().area << std::endl;
    //     std::cout << "rot: " << (minAngle + maxAngle) / 2 << std::endl;
    //     std::cout << "----------------------------------------------------------" << std::endl;
    //     std::cout << std::endl << std::endl;
    // }
}

bool PlaneDetector::isFalsePositive(PlanarPatch *patch)
{
    return patch->numUpdates() == 0 ||
            patch->getSize() / mMaxSize < 0.01f;
}