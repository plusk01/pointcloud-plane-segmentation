#ifndef GEOMETRYUTILS_H
#define GEOMETRYUTILS_H

#include <vector>

#include <Eigen/Dense>

class GeometryUtils
{
public:
    /**
     * @brief Given a R3 vector (v1), calculate two orthogonal vectors (v2 and v3) to
     * form a orthogonal basis
     * @param normal
     *      Vector to which the other two orthogonal vectors will be calculated upon
     * @param basis1
     *      An orthogonal vector to normal and basis2
     * @param basis2
     *      An orthogonal vector to normal and basis1
     */
    inline static void orthogonalBasis(const Eigen::Vector3d &normal, Eigen::Vector3d &v2, Eigen::Vector3d &v3)
    {
        v2 = Eigen::Vector3d(normal.y() - normal.z(), -normal.x(), normal.x());
        v2 = v2.normalized();
        v3 = normal.cross(v2);
        v3 = v3.normalized();
    }

    inline static Eigen::Vector2d projectOntoOrthogonalBasis(const Eigen::Vector3d &vector, const Eigen::Vector3d &basisU, const Eigen::Vector3d &basisV)
    {
        float alpha = vector.dot(basisU);
        float beta = vector.dot(basisV);
        return Eigen::Vector2d(alpha, beta);
    }

    inline static Eigen::Vector2d projectOntoOrthogonalBasis(const Eigen::Vector3d &vector, const Eigen::Vector3d &normal)
    {
        Eigen::Vector3d basisU, basisV;
        orthogonalBasis(normal, basisU, basisV);
        return projectOntoOrthogonalBasis(vector, basisU, basisV);
    }

    // monotone chain algorithm for convex hull
    // https://en.wikipedia.org/wiki/Convex_hull_algorithms#Algorithms
    // https://github.com/MiguelVieira/ConvexHull2D/blob/master/ConvexHull.cpp#L119
    // https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C++
    static void convexHull(const std::vector<Eigen::Vector2d> &points, std::vector<size_t> &indices)
    {
        if (points.size() < 3)
        {
            for (size_t i = 0; i < points.size(); i++)
            {
                indices.push_back(i);
            }
            return;
        }
        std::vector<IndexedPoint2d*> indexedPoints(points.size());
        for (size_t i = 0; i < indexedPoints.size(); i++)
        {
            indexedPoints[i] = new IndexedPoint2d(i, points[i]);
        }
        std::sort(indexedPoints.begin(), indexedPoints.end(), [](const IndexedPoint2d *a, const IndexedPoint2d *b) {
            return a->point.x() < b->point.x() || (a->point.x() == b->point.x() && a->point.y() < b->point.y());
        });
        std::vector<IndexedPoint2d*> H(2*points.size());
        size_t k = 0;

        // Build lower hull
        for (size_t i = 0; i < indexedPoints.size(); i++)
        {
            while (k >= 2 && convexHullCross(H[k-2], H[k-1], indexedPoints[i]) <= 0) k--;
            H[k++] = indexedPoints[i];
        }

        // Build upper hull
        for (size_t i = indexedPoints.size()-1, t = k+1; i > 0; --i)
        {
            while (k >= t && convexHullCross(H[k-2], H[k-1], indexedPoints[i-1]) <= 0) k--;
            H[k++] = indexedPoints[i-1];
        }

        H.resize(k-1);
        indices = std::vector<size_t>(H.size());
        for (size_t i = 0; i < H.size(); i++)
        {
            indices[i] = H[i]->index;
        }

        for (size_t i = 0; i < indexedPoints.size(); i++)
        {
            delete indexedPoints[i];
        }
    }

private:
    struct IndexedPoint2d
    {
        size_t index;
        Eigen::Vector2d point;
        IndexedPoint2d(size_t index, const Eigen::Vector2d &point)
            : index(index)
            , point(point)
        {

        }
    };

    static float convexHullCross(const IndexedPoint2d *o, const IndexedPoint2d *a, const IndexedPoint2d *b)
    {
        return (a->point.x() - o->point.x()) * (b->point.y() - o->point.y()) -
                (a->point.y() - o->point.y()) * (b->point.x() - o->point.x());
    }

};

#endif // GEOMETRYUTILS_H
