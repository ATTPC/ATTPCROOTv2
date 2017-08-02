#include "ATCubicSplineFit.hh"

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
Eigen::Matrix4f const ATCubicSplineFit::hermitricMatix(hermitricMatixValue);

ATCubicSplineFit::ATCubicSplineFit(std::vector<Eigen::Vector3f> const &controlPoints, float tangentScale, size_t jump, ATCubicSplineFit::PositionFunction positionFunction)
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

Eigen::Vector3f ATCubicSplineFit::GetPoint(float position) const
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

ATCubicSplineFit::Spline const &ATCubicSplineFit::GetSpline() const
{
    return this->_spline;
}

std::vector<Eigen::Vector3f> const &ATCubicSplineFit::GetControlPoints() const
{
    return this->_controlPoints;
}

Eigen::Vector3f ATCubicSplineFit::CalculateTangent(size_t pos, size_t jump, float scale) const
{
    std::vector<Eigen::Vector3f> const &controlPoints = this->GetControlPoints();

    size_t const leftIndex = pos < jump ? 0 : pos - jump;
    size_t const rightIndex = std::min((pos + jump), (controlPoints.size() - 1));

    return scale * (controlPoints[rightIndex] - controlPoints[leftIndex]);
}
