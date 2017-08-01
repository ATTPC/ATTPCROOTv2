#ifndef ATCUBICSPLINEFIT_H
#define ATCUBICSPLINEFIT_H

// implementation of the Catmull-Rom-Spline
// (see: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull.E2.80.93Rom_spline)
// though I used the method described on the german page ;)
// (see: https://de.wikipedia.org/wiki/Kubisch_Hermitescher_Spline#Catmull-Rom-Spline)

#include <vector>
#include <pcl/io/io.h>
#include <Eigen/Core>

template <class T>
static std::vector<Eigen::Vector3f> Cloud2Vectors(pcl::PointCloud<T> const &cloud)
{
    std::vector<Eigen::Vector3f> result;

    for (T const &pclPoint : cloud)
    {
        result.push_back(Eigen::Vector3f(
            pclPoint.x,
            pclPoint.y,
            pclPoint.z
        ));
    }

    return result;
}

template <class T>
class ATCubicSplineFit
{
public:
    struct SplineSegment
    {
        float p0Pos;
        float p1Pos;
        size_t p0Index;
        size_t p1Index;
        Eigen::Vector3f m0;
        Eigen::Vector3f m1;
    };

    using Spline = std::vector<SplineSegment>;
    using PositionFunction = std::function<float(std::vector<Eigen::Vector3f> const &, size_t)>;

    static inline float defaultPositionFunction(std::vector<Eigen::Vector3f> const &controlPoints, size_t index)
    {
        return static_cast<float>(index);
    }

    static Eigen::Matrix4f const hermitricMatix;

    ATCubicSplineFit(std::vector<Eigen::Vector3f> const &controlPoints, float tangentScale = 0.5f, size_t jump = 1, PositionFunction positionFunction = defaultPositionFunction)
        : _controlPoints(controlPoints), _positionFunction(positionFunction)
    {
        for (size_t k = 0; k < (controlPoints.size() - jump); k += jump)
        {
            SplineSegment splineSegment;
            splineSegment.p0Index = k;
            splineSegment.p1Index = k + jump;
            splineSegment.p0Pos = positionFunction(controlPoints, splineSegment.p0Index);
            splineSegment.p1Pos = positionFunction(controlPoints, splineSegment.p1Index);
            splineSegment.m0 = this->CalculateTangent(splineSegment.p0Index, jump, tangentScale);
            splineSegment.m1 = this->CalculateTangent(splineSegment.p1Index, jump, tangentScale);

            this->_spline.push_back(splineSegment);
        }
    }

    ATCubicSplineFit(pcl::PointCloud<T> const &cloud, float tangentScale = 0.5f, size_t jump = 1, PositionFunction positionFunction = defaultPositionFunction)
        : ATCubicSplineFit(Cloud2Vectors(cloud), tangentScale, jump, positionFunction)
    {
        // NOOP
    }

    Eigen::Vector3f GetPoint(float position) const
    {
        Spline const &spline = this->GetSpline();
        std::vector<Eigen::Vector3f> const &controlPoints = this->GetControlPoints();

        auto splineSegmentIt = std::lower_bound(spline.cbegin(), spline.cend(), position, [](SplineSegment const &lhs, float rhs)
        {
            return lhs.p1Pos < rhs;
        });

        if (splineSegmentIt == spline.cend())
            splineSegmentIt = spline.cend() - 1;

        SplineSegment const &splineSegment = *splineSegmentIt;

        /*
        std::cout << "P: " << position
            << " segment: "
            << splineSegment.p0Pos << " "
            << splineSegment.p1Pos << " "
            << splineSegment.p0Index << " "
            << "(" << controlPoints[splineSegment.p0Index](0) << " " << controlPoints[splineSegment.p0Index](1) << " " << controlPoints[splineSegment.p0Index](2) << ") "
            << splineSegment.p1Index << " "
            << "(" << controlPoints[splineSegment.p1Index](0) << " " << controlPoints[splineSegment.p1Index](1) << " " << controlPoints[splineSegment.p1Index](2) << ") "
            << "(" << splineSegment.m0(0) << " " << splineSegment.m0(1) << " " << splineSegment.m0(2) << ") "
            << "(" << splineSegment.m1(0) << " " << splineSegment.m1(1) << " " << splineSegment.m1(2) << ")"
            << std::endl;
        */

        float const scaledPosition =
            (position - this->_positionFunction(controlPoints, splineSegment.p0Index)) /
            (this->_positionFunction(controlPoints, splineSegment.p1Index) - this->_positionFunction(controlPoints, splineSegment.p0Index));

        Eigen::RowVector4f const tVector(
            scaledPosition * scaledPosition * scaledPosition,
            scaledPosition * scaledPosition,
            scaledPosition,
            1
        );

        Eigen::Matrix<float, 4, 3> segmentMatrix;
        segmentMatrix.row(0) = controlPoints[splineSegment.p0Index];
        segmentMatrix.row(1) = controlPoints[splineSegment.p1Index];
        segmentMatrix.row(2) = splineSegment.m0;
        segmentMatrix.row(3) = splineSegment.m1;

        return tVector * ATCubicSplineFit::hermitricMatix * segmentMatrix;
    }

    T GetPclPoint(float position) const
    {
        Eigen::Vector3f point = GetPoint(position);
        T pclPoint;
        pclPoint.x = point(0);
        pclPoint.y = point(1);
        pclPoint.z = point(2);

        return pclPoint;
    }

    Spline const &GetSpline() const
    {
        return this->_spline;
    }

    std::vector<Eigen::Vector3f> const &GetControlPoints() const
    {
        return this->_controlPoints;
    }

protected:
    Spline _spline;
    std::vector<Eigen::Vector3f> const _controlPoints;
    PositionFunction const _positionFunction;

    Eigen::Vector3f CalculateTangent(size_t pos, size_t jump, float scale) const
    {
        std::vector<Eigen::Vector3f> const &controlPoints = this->GetControlPoints();

        size_t const leftIndex = pos < jump ? 0 : pos - jump;
        size_t const rightIndex = std::min((pos + jump), (controlPoints.size() - 1));

        return scale * (controlPoints[rightIndex] - controlPoints[leftIndex]);
    }
};

// mind the reversion of columns and rows
// original matrix:
//  2 -2  1  1
// -3  3 -2 -1
//  0  0  1  0
//  1  0  0  0
static float const hermitricMatixValue[] = {
     2.0f, -3.0f,  0.0f,  1.0f,
    -2.0f,  3.0f,  0.0f,  0.0f,
     1.0f, -2.0f,  1.0f,  0.0f,
     1.0f, -1.0f,  0.0f,  0.0f
};

template <class T>
Eigen::Matrix4f const ATCubicSplineFit<T>::hermitricMatix(hermitricMatixValue);

#endif
