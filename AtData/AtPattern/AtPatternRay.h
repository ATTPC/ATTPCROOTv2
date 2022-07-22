#ifndef ATPATTERNRAY_H
#define ATPATTERNRAY_H

#include "AtPatternLine.h"

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride

#include <memory> // for make_unique, unique_ptr
#include <vector> // for vector

class TBuffer;
class TClass;
class TEveElement;
class TMemberInspector;

namespace AtPatterns {
class AtPattern;
}

namespace AtPatterns {

/**
 * @brief Describes a linear track with an end point.
 *
 * @ingroup AtPattern
 */
class AtPatternRay : public AtPatternLine {
public:
   AtPatternRay();

   void DefinePattern(XYZPoint point, XYZVector direction);
   virtual void DefinePattern(const std::vector<XYZPoint> &points) override { AtPatternLine::DefinePattern(points); }
   virtual Double_t DistanceToPattern(const XYZPoint &point) const override;
   virtual XYZPoint ClosestPointOnPattern(const XYZPoint &point) const override;
   virtual XYZPoint GetPointAt(double z) const override;
   virtual TEveElement *GetEveElement() const override;
   virtual std::unique_ptr<AtPattern> Clone() const override { return std::make_unique<AtPatternRay>(*this); }

   ClassDefOverride(AtPatternRay, 1)
};

} // namespace AtPatterns

#endif //#ifndef ATPATTERNRAY_H
