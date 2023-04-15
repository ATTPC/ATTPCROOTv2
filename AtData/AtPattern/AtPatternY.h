#ifndef ATPATTERNY_H
#define ATPATTERNY_H

#include "AtPattern.h"
#include "AtPatternRay.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for THashConsistencyHolder, ClassDefOverride

#include <array>
#include <memory> // for make_unique, unique_ptr
#include <vector> // for vector
class TEveElement;

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
protected:
   AtPatternRay fBeam;
   std::array<AtPatternRay, 2> fFragments;

public:
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   AtPatternY();

   /**
    * @brief Get the vertex of the Y shape.
    */
   XYZPoint GetVertex() const { return fBeam.GetPoint(); }

   /**
    * @brief Get the direction of the beam ray.
    * @return Direction of beam (points from vertex to window).
    */
   XYZVector GetBeamDirection() const { return fBeam.GetDirection(); }

   /**
    * @brief Get the direction of the fragment rays.
    * @param[in] frag ID of the fragment (0 or 1)
    * @return Direction of fragment (points away from vertex).
    */
   XYZVector GetFragmentDirection(int frag) const { return fFragments.at(frag).GetDirection(); }

   /**
    * Returns an integer specifying which ray the point is closest to.
    * 0 -> fragment 0. 1 -> fragment 1. 2 -> beam.
    */
   int GetPointAssignment(const XYZPoint &point) const;

   virtual void DefinePattern(const std::vector<XYZPoint> &points) override;
   virtual void DefinePattern(std::vector<double> par) override;
   virtual Double_t DistanceToPattern(const XYZPoint &point) const override;
   virtual XYZPoint ClosestPointOnPattern(const XYZPoint &point) const override;
   virtual XYZPoint GetPointAt(double z) const override;
   virtual TEveElement *GetEveElement() const override;
   virtual std::unique_ptr<AtPattern> Clone() const override { return std::make_unique<AtPatternY>(*this); }
   virtual std::vector<double> GetPatternPar() const override;

protected:
   virtual void FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge) override;

   void DefinePattern(const XYZPoint &vertex, const XYZVector &beamDir, const std::array<XYZVector, 2> &fragDir);

   ClassDefOverride(AtPatternY, 1)
};
} // namespace AtPatterns

#endif //#ifndef ATPATTERNY_H
