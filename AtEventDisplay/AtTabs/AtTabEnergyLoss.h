#ifndef ATTABENERGYLOSS_H
#define ATTABENERGYLOSS_H
#include "AtDataObserver.h" // for AtObserver
#include "AtDataSubject.h"  // for AtSimpleType, AtSubject (ptr only)
#include "AtFissionEvent.h"
#include "AtPadReference.h" // for AtPadReference
#include "AtRawEvent.h"
#include "AtTabCanvas.h"
#include "AtTabInfo.h" // for AtTabInfoFairRoot

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for Color_t
#include <THStack.h>

#include <array>  // for array
#include <memory> // for unique_ptr
#include <vector> // for vector

class AtHit;
class TF1;
class TH1F;
namespace DataHandling {
class AtTreeEntry;
}
namespace DataHandling {
class AtBranch;
}
/**
 * @brief Class to calculate dE/dx for fission fragments (but possibly generally).
 *
 */
class AtTabEnergyLoss : public AtTabCanvas, public DataHandling::AtObserver {
protected:
   using XYZVector = ROOT::Math::XYZVector;
   using XYZPoint = ROOT::Math::XYZPoint;
   using TH1Ptr = std::unique_ptr<TH1F>;
   using HitVector = std::vector<AtHit *>;

   AtTabInfoFairRoot<AtRawEvent> fRawEvent;         //< Points to selected RawEventBranch
   AtTabInfoFairRoot<AtFissionEvent> fFissionEvent; //< Fision event to analyse

   DataHandling::AtTreeEntry &fEntry;           //< Tracks current entry
   DataHandling::AtSimpleType<float> fBinWidth; //< Width of binning in mm

   /// Number of std dev to go after the mean of the first hit before calcualting the ratio of the
   /// histograms.
   DataHandling::AtSimpleType<float> fSigmaFromHit;
   DataHandling::AtSimpleType<int> fTBtoAvg; //< Number of TB from track start to average for ratio.

   // Helpful things
   const std::array<Color_t, 2> fHistColors = {9, 31};

   THStack dEdxStack{"hs", "Stacked dE/dx curves"};
   std::array<TH1Ptr, 2> dEdx;

   THStack dEdxStackZ{"hsz", "Stacked dE/dx curves bin in Z"};
   std::array<TH1Ptr, 2> dEdxZ;

   // Histograms to fill with charge sum
   THStack dEdxStackSum{"hsSum", "dQ/dZ (summing charge)"};
   std::array<TH1Ptr, 2> fSumQ;

   THStack dEdxStackFit{"hsFit", "dQ/dZ (summing gaussian fits)"};
   std::array<TH1Ptr, 2> fSumFit;

   TH1Ptr fRatioQ;   //<Ratio of max(fSumQ)/min(fSumQ)
   TH1Ptr fRatioFit; //<Ratio of max(fSumFit)/min(fSumFit)
   TH1Ptr fProxy;    //<Proxy for Z in an event
   TH1Ptr fZHist;    //<Z in an event

   std::array<float, 2> fTrackStart{};
   std::array<AtHit *, 2> fFirstHit{nullptr, nullptr}; //< First hit calculated according to the gaussian fits

   std::unique_ptr<TF1> fRatioFunc;
   std::unique_ptr<TF1> fProxyFunc;
   std::unique_ptr<TF1> fZFunc;

   std::vector<AtPadReference> fVetoPads;

public:
   AtTabEnergyLoss(DataHandling::AtBranch &fissionBranch);
   ~AtTabEnergyLoss();
   void InitTab() override;
   void Exec() override{};
   void Update(DataHandling::AtSubject *sub) override;

   static float GetZ(int Zcn, float proxy);

private:
   void SetStyle(std::array<TH1Ptr, 2> &hists, THStack &stack);
   void Update();
   void setdEdX();
   double getHitDistanceFromVertex(const AtHit &hit);
   double getHitDistanceFromVertexAlongZ(const AtHit &hit);

   void FillSums(float threshold = 15);

   void FillRatio();
};

#endif //#ifndef ATTABENERGYLOSS_H
