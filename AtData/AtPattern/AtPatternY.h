#ifndef ATPATTERNY_H
#define ATPATTERNY_H

#include "AtPattern.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for THashConsistencyHolder, ClassDefOverride

#include <memory> // for make_unique, unique_ptr
#include <vector> // for vector
class TEveLine;

class TBuffer;
class TClass;
class TMemberInspector;

namespace AtPatterns {

/**
 * @brief Describes a Y track
 *
 * @ingroup AtPattern
 */
class AtPatternY : public AtPattern {
public:
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   AtPatternY();

   XYZPoint GetVertex() const { return {fPatternPar[0], fPatternPar[1], fPatternPar[2]}; }
   XYZVector GetDirection(int line) const;

   virtual void DefinePattern(const std::vector<XYZPoint> &points) override;
   virtual Double_t DistanceToPattern(const XYZPoint &point) const override;
   virtual XYZPoint ClosestPointOnPattern(const XYZPoint &point) const override;
   virtual XYZPoint GetPointAt(double z) const override;
   virtual TEveLine *GetEveLine() const override;
   virtual std::unique_ptr<AtPattern> Clone() const override { return std::make_unique<AtPatternY>(*this); }

protected:
   virtual void FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge) override;

   ClassDefOverride(AtPatternY, 1)
};
} // namespace AtPatterns

#endif //#ifndef ATPATTERNY_H
