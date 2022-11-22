#ifndef ATPATTERNLINE_H
#define ATPATTERNLINE_H

#include "AtPattern.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for THashConsistencyHolder, ClassDefOverride

#include <memory> // for make_unique, unique_ptr
#include <vector> // for vector

class TBuffer;
class TClass;
class TMemberInspector;
class TEveElement;
class TEveLine;

namespace AtPatterns {

/**
 * @brief Describes a linear track
 *
 * @ingroup AtPattern
 */
class AtPatternLine : public AtPattern {
public:
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   AtPatternLine();

   XYZPoint GetPoint() const { return {fPatternPar[0], fPatternPar[1], fPatternPar[2]}; }
   XYZVector GetDirection() const { return {fPatternPar[3], fPatternPar[4], fPatternPar[5]}; }

   virtual void DefinePattern(const std::vector<XYZPoint> &points) override;
   virtual Double_t DistanceToPattern(const XYZPoint &point) const override;
   virtual XYZPoint ClosestPointOnPattern(const XYZPoint &point) const override;
   virtual XYZPoint GetPointAt(double z) const override;
   virtual TEveElement *GetEveElement() const override;
   virtual std::unique_ptr<AtPattern> Clone() const override { return std::make_unique<AtPatternLine>(*this); }

   TEveLine *GetEveLine(Double_t rMax = 250) const;

protected:
   std::vector<Double_t> lineIntersecR(Double_t rMax, Double_t tMin, Double_t tMax) const;

   virtual void FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge) override;
   double parameterAtPoint(const XYZPoint &point) const;
   ClassDefOverride(AtPatternLine, 1)
};
} // namespace AtPatterns
#endif //#ifndef ATPATTERNLINE_H
