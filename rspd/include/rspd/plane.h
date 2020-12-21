#ifndef PLANE_H
#define PLANE_H

#include "geometryutils.h"

class Plane
{
public:
    Plane()
        : mCenter(Eigen::Vector3d::Zero())
    {

    }

    Plane(const Eigen::Vector3d &center, const Eigen::Vector3d &normal, Eigen::Vector3d basisU = Eigen::Vector3d::Zero(), Eigen::Vector3d basisV = Eigen::Vector3d::Zero())
        : mCenter(center)
        , mNormal(normal)
        , mBasisU(basisU)
        , mBasisV(basisV)
    {
        mDistanceFromOrigin = -mNormal.dot(center);
    }

    const Eigen::Vector3d& normal() const
    {
        return mNormal;
    }

    void normal(const Eigen::Vector3d &normal)
    {
        mNormal = normal;
        mDistanceFromOrigin = -mNormal.dot(center());
    }

    const Eigen::Vector3d& basisU() const
    {
        return mBasisU;
    }

    void basisU(const Eigen::Vector3d &basisU)
    {
        mBasisU = basisU;
    }

    const Eigen::Vector3d& basisV() const
    {
        return mBasisV;
    }

    void basisV(const Eigen::Vector3d &basisV)
    {
        mBasisV = basisV;
    }

    const Eigen::Vector3d& center() const
    {
        return mCenter;
    }

    void center(const Eigen::Vector3d &center)
    {
        mCenter = center;
        mDistanceFromOrigin = -mNormal.dot(center);
    }

    float distanceFromOrigin() const
    {
        return mDistanceFromOrigin;
    }

    float getSignedDistanceFromSurface(const Eigen::Vector3d &point) const
    {
        return mNormal.dot(point) + mDistanceFromOrigin;
    }

    const std::vector<size_t>& inliers() const
    {
        return mInliers;
    }

    void inliers(const std::vector<size_t> &inliers)
    {
        mInliers = inliers;
    }

private:
    Eigen::Vector3d mCenter;
    std::vector<size_t> mInliers;

    Eigen::Vector3d mNormal;
    Eigen::Vector3d mBasisU;
    Eigen::Vector3d mBasisV;
    float mDistanceFromOrigin;

};

//template class Plane<float>;
//template class Plane<double>;

#endif // PLANE_H
