#ifndef ATTABFF_H
#define ATTABFF_H

#include "AtDataObserver.h"
#include "AtEvent.h"
#include "AtFissionEvent.h"
#include "AtPadReference.h" // for AtPadReference
#include "AtTabCanvas.h"
#include "AtTabInfo.h" // for AtTabInfoFairRoot

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for Color_t
#include <THStack.h>

class AtTabFF : public AtTabCanvas, public DataHandling::AtObserver {
protected:
   using XYZVector = ROOT::Math::XYZVector;
   using XYZPoint = ROOT::Math::XYZPoint;
   using TH1Ptr = std::unique_ptr<TH1F>;
   using HitVector = std::vector<AtHit *>;

   AtTabInfoFairRoot<AtEvent> fEvent;
   AtTabInfoFairRoot<AtFissionEvent> fFissionEvent;
   DataHandling::AtTreeEntry &fEntry;

   std::array<TH1Ptr, 2> fSimdQdZ;
   std::array<TH1Ptr, 2> fExpdQdZ;
   std::array<std::set<int>, 2> fCurrPads;

public:
   AtTabFF(DataHandling::AtBranch &fissionBranch);
   ~AtTabFF();

   void Exec() override {}
   void InitTab() override {}
   void Update(DataHandling::AtSubject *sub) override;

protected:
   void UpdateSimEvent();
   void UpdateExpEvent();
   void DrawCanvas();

   ClassDefOverride(AtTabFF, 1);
};

#endif //#ifndef ATTABFF_H
