#ifndef ATCUBICSPLINEFIT_H
#define ATCUBICSPLINEFIT_H

// implementation of the Catmull-Rom-Spline
// (see: https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull.E2.80.93Rom_spline)
// though I used the method described on the german page ;)
// (see: https://de.wikipedia.org/wiki/Kubisch_Hermitescher_Spline#Catmull-Rom-Spline)

#include <vector>
#include <Eigen/Core>

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
    using PositionFunction = std::function<float(Eigen::Vector3f const &, size_t)>;

    static inline float defaultPositionFunction(Eigen::Vector3f const &point, size_t index)
    {
        return static_cast<float>(index);
    }

    static Eigen::Matrix4f const hermitricMatrix;

    ATCubicSplineFit(std::vector<Eigen::Vector3f> const &controlPoints, float tangentScale = 0.5f, float minControlPointDistance = 0.01f, size_t jump = 1, PositionFunction positionFunction = defaultPositionFunction);

    float const &GetStartPosition() const;
    float const &GetEndPosition() const;
    Spline const &GetSpline() const;
    std::vector<Eigen::Vector3f> const &GetControlPoints() const;

    Eigen::Vector3f CalculatePoint(float position) const;
    Eigen::Vector3f CalculateDerivativePoint(float position, float delta = 0.1f) const;
    Eigen::Vector3f CalculateSecondDerivativePoint(float position, float delta = 0.1f) const;
    float CalculateArcLength(float startPosition, float endPosition, size_t sampleSize) const;
    float CalculateAverageCurvature(float const startPosition, float const endPosition, size_t const sampleSize, float const delta = 0.1f) const;

protected:
    Spline _spline;
    std::vector<Eigen::Vector3f> _controlPoints;
    PositionFunction _positionFunction;
    float _tangentScale;

    Eigen::Vector3f CalculateTangent(size_t pos, size_t jump, float scale) const;
};

#endif
