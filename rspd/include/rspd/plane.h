#ifndef PLANE_H
#define PLANE_H

#include "geometryutils.h"

class Plane
{
public:
    Plane()
        : mCenter(Eigen::Vector3f::Zero())
    {

    }

    Plane(const Eigen::Vector3f &center, const Eigen::Vector3f &normal, Eigen::Vector3f basisU = Eigen::Vector3f::Zero(), Eigen::Vector3f basisV = Eigen::Vector3f::Zero())
        : mCenter(center)
        , mNormal(normal)
        , mBasisU(basisU)
        , mBasisV(basisV)
    {
        mDistanceFromOrigin = -mNormal.dot(center);
    }

    const Eigen::Vector3f& normal() const
    {
        return mNormal;
    }

    void normal(const Eigen::Vector3f &normal)
    {
        mNormal = normal;
        mDistanceFromOrigin = -mNormal.dot(center());
    }

    const Eigen::Vector3f& basisU() const
    {
        return mBasisU;
    }

    void basisU(const Eigen::Vector3f &basisU)
    {
        mBasisU = basisU;
    }

    const Eigen::Vector3f& basisV() const
    {
        return mBasisV;
    }

    void basisV(const Eigen::Vector3f &basisV)
    {
        mBasisV = basisV;
    }

    const Eigen::Vector3f& center() const
    {
        return mCenter;
    }

    void center(const Eigen::Vector3f &center)
    {
        mCenter = center;
        mDistanceFromOrigin = -mNormal.dot(center);
    }

    float distanceFromOrigin() const
    {
        return mDistanceFromOrigin;
    }

    float getSignedDistanceFromSurface(const Eigen::Vector3f &point) const
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
    Eigen::Vector3f mCenter;
    std::vector<size_t> mInliers;

    Eigen::Vector3f mNormal;
    Eigen::Vector3f mBasisU;
    Eigen::Vector3f mBasisV;
    float mDistanceFromOrigin;

};

//template class Plane<float>;
//template class Plane<double>;

#endif // PLANE_H
