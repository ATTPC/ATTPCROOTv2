#ifndef ATTABFF_H
#define ATTABFF_H

#include "AtDataObserver.h"
#include "AtEvent.h"
#include "AtFissionEvent.h"
#include "AtRawEvent.h"
#include "AtTabCanvas.h"
#include "AtTabInfo.h" // for AtTabInfoFairRoot

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for Color_t

#include <array>  // for array
#include <memory> // for unique_ptr
#include <set>    // for set
#include <vector> // for vector
class AtHit;
class TBuffer;
class TClass;
class TH1F;
class TMemberInspector;
class THStack;
namespace DataHandling {
class AtBranch;
}
namespace DataHandling {
class AtSubject;
}
namespace DataHandling {
class AtTreeEntry;
}

class AtTabFF : public AtTabCanvas, public DataHandling::AtObserver {
protected:
   using XYZVector = ROOT::Math::XYZVector;
   using XYZPoint = ROOT::Math::XYZPoint;
   using TH1Ptr = std::unique_ptr<TH1F>;
   using HitVector = std::vector<AtHit *>;
   using THStackPtr = std::unique_ptr<THStack>;

   AtTabInfoFairRoot<AtEvent> fEvent;
   AtTabInfoFairRoot<AtRawEvent> fRawEvent;
   AtTabInfoFairRoot<AtFissionEvent> fFissionEvent;
   DataHandling::AtTreeEntry &fEntry;

   std::array<TH1Ptr, 2> fSimdQdZ;
   std::array<TH1Ptr, 2> fExpdQdZ;
   std::array<TH1Ptr, 2> fExpADCSum;
   std::array<std::set<int>, 2> fCurrPads;
   std::array<THStackPtr, 4> fStacks;

public:
   AtTabFF(DataHandling::AtBranch &fissionBranch, bool plotADC = false);
   ~AtTabFF();

   void Exec() override {}
   void InitTab() override {}
   void Update(DataHandling::AtSubject *sub) override;

protected:
   void UpdateEvent();
   void DrawCanvas();

   std::vector<AtHit *> GetFragmentHits(AtEvent *event);
   ClassDefOverride(AtTabFF, 1);
};

#endif //#ifndef ATTABFF_H
