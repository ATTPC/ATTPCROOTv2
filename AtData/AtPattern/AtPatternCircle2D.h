#ifndef ATPATTERNCIRCLE2D_H
#define ATPATTERNCIRCLE2D_H

#include "AtPattern.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Rtypes.h>          // for THashConsistencyHolder, ClassDefOverride

#include <memory> // for make_unique, unique_ptr
#include <vector> // for vector

class TEveElement;
class TBuffer;
class TClass;
class TMemberInspector;

using XYZPoint = ROOT::Math::XYZPoint;

namespace AtPatterns {
/**
 * @brief Describes a circle track projected to the XY plane
 *
 * @ingroup AtPattern
 */
class AtPatternCircle2D : public AtPattern {
public:
   AtPatternCircle2D();

   XYZPoint GetCenter() const { return {fPatternPar[0], fPatternPar[1], 0}; }
   double GetRadius() const { return fPatternPar[2]; }

   virtual void DefinePattern(const std::vector<XYZPoint> &points) override;
   virtual Double_t DistanceToPattern(const XYZPoint &point) const override;
   virtual XYZPoint ClosestPointOnPattern(const XYZPoint &point) const override;
   virtual XYZPoint GetPointAt(double theta) const override;
   virtual TEveElement *GetEveElement() const override;
   virtual std::unique_ptr<AtPattern> Clone() const override { return std::make_unique<AtPatternCircle2D>(*this); }

protected:
   virtual void FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge) override;

   ClassDefOverride(AtPatternCircle2D, 1)
};
} // namespace AtPatterns
#endif //#ifndef ATPATTERNCIRCLE2D_H
