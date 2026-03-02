#ifndef ATPPACPAIR_H
#define ATPPACPAIR_H

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h>
#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <Rtypes.h>
#include <TObject.h>

class AtPPACPair : public TObject {
public:
   using XYPoint = ROOT::Math::XYPoint;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;

protected:
   // Positions, direction and angles.
   XYPoint fEntrancePosition;
   XYPoint fExitPosition;
   XYZVector fTrackDirection;
   Double_t fTrackPolarAngle;
   Double_t fTrackAzimutalAngle;

   // Geometrical parameters.
   Double_t fSeparationDistance{500}; // [mm]. Distance between the 2 PPACs.

public:
   AtPPACPair() {}
   AtPPACPair(XYPoint entrancePosition, XYPoint exitPosition, Double_t separationDistance = 500)
   {
      fEntrancePosition = entrancePosition;
      fExitPosition = exitPosition;
      fSeparationDistance = separationDistance;
      CalculateDirection();
   }
   AtPPACPair(const AtPPACPair &) = default;
   AtPPACPair(AtPPACPair &&) = default;
   AtPPACPair &operator=(const AtPPACPair &) = default;
   AtPPACPair &operator=(AtPPACPair &&) = default;
   virtual ~AtPPACPair() = default;
   virtual std::unique_ptr<AtPPACPair> Clone();

   void SetEntrancePosition(XYPoint position)
   {
      fEntrancePosition = position;
      CalculateDirection();
   }
   void SetExitPosition(XYPoint position)
   {
      fExitPosition = position;
      CalculateDirection();
   }
   void SetSeparationDistance(Double_t separationDistance)
   {
      fSeparationDistance = separationDistance;
      CalculateDirection();
   }

   const XYPoint GetEntrancePosition() const { return fEntrancePosition; }
   const XYPoint GetExitPosition() const { return fExitPosition; }
   const XYZVector GetTrackDirection() const { return fTrackDirection; }
   const Double_t GetTrackPolarAngle() const { return fTrackPolarAngle; }
   const Double_t GetTrackAzimutalAngle() const { return fTrackAzimutalAngle; }
   const Double_t GetSeparationDistance() const { return fSeparationDistance; }

private:
   void CalculateDirection();

public:
   ClassDef(AtPPACPair, 1);
};

#endif
