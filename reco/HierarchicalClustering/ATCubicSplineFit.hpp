#ifndef ATCUBICSPLINEFIT_H
#define ATCUBICSPLINEFIT_H

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
        // return controlPoints[index](2);
    }

    static Eigen::Matrix4f const hermitricMatix;

    ATCubicSplineFit(std::vector<Eigen::Vector3f> const &controlPoints, float tangentScale = 0.5f, PositionFunction positionFunction = defaultPositionFunction)
        : _controlPoints(controlPoints), _positionFunction(positionFunction)
    {
        float currentPosition = 0.0f;

        for (size_t k = 0; k < (controlPoints.size() - 1); ++k)
        {
            SplineSegment splineSegment;
            splineSegment.p0Index = k;
            splineSegment.p1Index = k + 1;
            splineSegment.p0Pos = currentPosition;
            float const distance = positionFunction(controlPoints, k + 1) - positionFunction(controlPoints, k);
            splineSegment.p1Pos = currentPosition + distance;
            splineSegment.m0 = this->CalculateTangent(k, tangentScale);
            splineSegment.m1 = this->CalculateTangent(k + 1, tangentScale);

            this->_spline.push_back(splineSegment);
            currentPosition += distance;
        }
    }

    ATCubicSplineFit(pcl::PointCloud<T> const &cloud, float tangentScale = 0.5f, PositionFunction positionFunction = defaultPositionFunction)
        : ATCubicSplineFit(Cloud2Vectors(cloud), tangentScale, positionFunction)
    {
        // NOOP
    }

    Eigen::Vector3f GetPoint(float position) const
    {
        std::vector<Eigen::Vector3f> const &controlPoints = this->GetControlPoints();

        // TODO: select proper segment depending on position
        size_t const k = static_cast<size_t>(position);

        SplineSegment splineSegment;

        splineSegment.p0Index = 0;
        splineSegment.p1Index = 1;
        splineSegment.m0 = Eigen::Vector3f(7.0f, 8.0f, 9.0f);
        splineSegment.m1 = Eigen::Vector3f(10.0f, 11.0f, 12.0f);
        // /end TODO

        float const scaledPosition = (position - this->_positionFunction(controlPoints, k)) / (this->_positionFunction(controlPoints, k + 1) - this->_positionFunction(controlPoints, k));

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

        std::cout
            << "tVector: " << tVector << std::endl
            << "segmentMatrix: " << segmentMatrix << std::endl
            << "hermitricMatix: " << hermitricMatix << std::endl;

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
    std::vector<Eigen::Vector3f> const &_controlPoints;
    PositionFunction const _positionFunction;

    Eigen::Vector3f CalculateTangent(size_t pos, float scale) const
    {
        std::vector<Eigen::Vector3f> const &controlPoints = this->GetControlPoints();

        size_t const leftIndex = std::max(pos - 1, (size_t)0);
        size_t const rightIndex = std::min(pos + 1, controlPoints.size() - 1);

        return scale * (controlPoints[leftIndex] - controlPoints[rightIndex]);
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
