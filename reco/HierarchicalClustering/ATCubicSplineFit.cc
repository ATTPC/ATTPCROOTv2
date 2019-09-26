#include "ATCubicSplineFit.hh"

// #include <iostream>

// mind the reversion of columns and rows
// original matrix:
//  2 -2  1  1
// -3  3 -2 -1
//  0  0  1  0
//  1  0  0  0
static float const hermitricMatrixValue[] = {
     2.0f, -3.0f,  0.0f,  1.0f,
    -2.0f,  3.0f,  0.0f,  0.0f,
     1.0f, -2.0f,  1.0f,  0.0f,
     1.0f, -1.0f,  0.0f,  0.0f
};
Eigen::Matrix4f const ATCubicSplineFit::hermitricMatrix(hermitricMatrixValue);

ATCubicSplineFit::ATCubicSplineFit(std::vector<Eigen::Vector3f> const &controlPoints, float tangentScale, float minControlPointDistance, size_t jump, ATCubicSplineFit::PositionFunction positionFunction)
    : _controlPoints(controlPoints), _positionFunction(positionFunction)
{
    this->_tangentScale = tangentScale;

    std::stable_sort(this->_controlPoints.begin(), this->_controlPoints.end(), [&](Eigen::Vector3f const &lhs, Eigen::Vector3f const &rhs)
    {
        return positionFunction(lhs, 0) < positionFunction(rhs, 0);
    });

    // remove points which are to close together
    {
        size_t writeindex = 1;
        float lastWrittenPosition = positionFunction(this->_controlPoints[0], 0);

        for (size_t readIndex = 1; readIndex < this->_controlPoints.size(); ++readIndex)
        {
            float const position = positionFunction(this->_controlPoints[readIndex], readIndex);

            if (position >= (lastWrittenPosition + minControlPointDistance))
            {
                if (writeindex != readIndex)
                    this->_controlPoints[writeindex] = this->_controlPoints[readIndex];

                lastWrittenPosition = position;
                ++writeindex;
            }
        }

        // add last index if it is not at the same position as the last element
        size_t const lastIndex = this->_controlPoints.size() - 1;
        if (lastWrittenPosition < positionFunction(this->_controlPoints[lastIndex], lastIndex))
        {
            this->_controlPoints[writeindex] = this->_controlPoints[lastIndex];
            ++writeindex;
        }

        this->_controlPoints.resize(writeindex);
    }

    if (jump >= this->_controlPoints.size())
        throw std::runtime_error("Too few control points!");

    for (size_t k = 0; k < (this->_controlPoints.size() - jump); k += jump)
    {
        SplineSegment splineSegment;
        splineSegment.p0Index = k;
        splineSegment.p1Index = k + jump;
        splineSegment.p0Pos = positionFunction(this->_controlPoints[splineSegment.p0Index], splineSegment.p0Index);
        splineSegment.p1Pos = positionFunction(this->_controlPoints[splineSegment.p1Index], splineSegment.p1Index);
        splineSegment.m0 = this->CalculateTangent(splineSegment.p0Index, jump, tangentScale);
        splineSegment.m1 = this->CalculateTangent(splineSegment.p1Index, jump, tangentScale);

        this->_spline.push_back(splineSegment);
    }
}

float const &ATCubicSplineFit::GetStartPosition() const
{
    return this->GetSpline().front().p0Pos;
}

float const &ATCubicSplineFit::GetEndPosition() const
{
    return this->GetSpline().back().p1Pos;
}

ATCubicSplineFit::Spline const &ATCubicSplineFit::GetSpline() const
{
    return this->_spline;
}

std::vector<Eigen::Vector3f> const &ATCubicSplineFit::GetControlPoints() const
{
    return this->_controlPoints;
}

Eigen::Vector3f ATCubicSplineFit::CalculatePoint(float position) const
{
    Spline const &spline = this->GetSpline();
    std::vector<Eigen::Vector3f> const &controlPoints = this->GetControlPoints();

    // handle out of range cases
    // use linear interpolation
    if (position < this->GetStartPosition())
    {
        SplineSegment const &firstSplineSegment = spline.front();

        float const scaledPosition = (position - firstSplineSegment.p0Pos) / (firstSplineSegment.p1Pos - firstSplineSegment.p0Pos);

        return controlPoints[firstSplineSegment.p0Index] + (scaledPosition / this->_tangentScale * firstSplineSegment.m0);
    }
    else if (position > this->GetEndPosition())
    {
        SplineSegment const &lastSplineSegment = spline.back();

        float const scaledPosition = (position - lastSplineSegment.p1Pos) / (lastSplineSegment.p1Pos - lastSplineSegment.p0Pos);

        return controlPoints[lastSplineSegment.p1Index] + (scaledPosition / this->_tangentScale * lastSplineSegment.m1);
    }

    // normal spline interpolation
    auto splineSegmentIt = std::lower_bound(spline.cbegin(), spline.cend(), position, [](SplineSegment const &lhs, float rhs)
    {
        return lhs.p1Pos < rhs;
    });

    // this should never happen
    if (splineSegmentIt == spline.cend())
        throw std::runtime_error("Illegal splineSegmentIt!");

    SplineSegment const &splineSegment = *splineSegmentIt;

    float const scaledPosition = (position - splineSegment.p0Pos) / (splineSegment.p1Pos - splineSegment.p0Pos);

    /*
    std::cout << "P: " << position << " (" << scaledPosition << ") "
        << "segment: "
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

    return tVector * ATCubicSplineFit::hermitricMatrix * segmentMatrix;
}


Eigen::Vector3f ATCubicSplineFit::CalculateDerivativePoint(float position, float delta) const
{
    return (this->CalculatePoint(position + delta) - this->CalculatePoint(position - delta)) / (delta + delta);
}

Eigen::Vector3f ATCubicSplineFit::CalculateSecondDerivativePoint(float position, float delta) const
{
    return (this->CalculateDerivativePoint(position + delta, delta) - this->CalculateDerivativePoint(position - delta, delta)) / (delta + delta);
}

float ATCubicSplineFit::CalculateAverageCurvature(float const startPosition, float const endPosition, size_t const sampleSize, float const delta) const
{
    float result = 0.0f;

    // respect the borders needed for derivating
    float const croppedStartPosition = startPosition + (delta + delta);
    float const croppedEndPosition = endPosition - (delta + delta);
    float const stepSize = (croppedEndPosition - croppedStartPosition) / static_cast<float>(sampleSize);

    // caluclate curvature as norm of the second derivative
    for (float position = croppedStartPosition; position <= croppedEndPosition; position += stepSize)
    {
        result += this->CalculateSecondDerivativePoint(position, delta).norm();
    }

    return result / static_cast<float>(sampleSize);
}

float ATCubicSplineFit::CalculateArcLength(float startPosition, float endPosition, size_t sampleSize) const
{
    float result = 0.0f;

    float const stepSize = (endPosition - startPosition) / static_cast<float>(sampleSize);

    Eigen::Vector3f lastPoint = this->CalculatePoint(startPosition);
    for (float position = startPosition + stepSize; position <= endPosition; position += stepSize)
    {
        Eigen::Vector3f point = this->CalculatePoint(position);

        result += (point - lastPoint).norm();

        lastPoint = point;
    }

    return result;
}

Eigen::Vector3f ATCubicSplineFit::CalculateTangent(size_t pos, size_t jump, float scale) const
{
    std::vector<Eigen::Vector3f> const &controlPoints = this->GetControlPoints();

    size_t const leftIndex = pos < jump ? 0 : pos - jump;
    size_t const rightIndex = std::min((pos + jump), (controlPoints.size() - 1));

    return scale * (controlPoints[rightIndex] - controlPoints[leftIndex]);
}
