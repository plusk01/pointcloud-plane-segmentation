#ifndef CUBE_H
#define CUBE_H

#include <vector>

#include "point.h"

template <size_t DIMENSION>
class Rect
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;

    Rect()
    {

    }

    Rect(const Vector &bottomLeft, const Vector &topRight)
        : mBottomLeft(bottomLeft)
        , mTopRight(topRight)
    {

    }

    Vector& bottomLeft()
    {
        return mBottomLeft;
    }

    Vector& topRight()
    {
        return mTopRight;
    }

    Vector center() const
    {
        return (mBottomLeft + mTopRight) / 2;
    }

    float maxSize() const
    {
        float maxSize = 0;
        for (size_t dim = 0; dim < DIMENSION; dim++)
        {
            maxSize = std::max(maxSize, mTopRight(dim) - mBottomLeft(dim));
        }
        return maxSize;
    }

    bool operator==(const Rect &other) const
    {
        return mBottomLeft == other.mBottomLeft &&
                mTopRight == other.mTopRight;
    }

private:
    Vector mBottomLeft;
    Vector mTopRight;

};

template class Rect<2>;
template class Rect<3>;

typedef Rect<2> Rect2d;
typedef Rect<3> Rect3d;

#endif // CUBE_H
