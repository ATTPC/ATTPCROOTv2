#ifndef AtPSA_H
#define AtPSA_H

#include "AtHit.h" // IWYU pragma: keep
#include "AtDataObserver.h"
#include "AtDataSubject.h"

#include <Rtypes.h>

#include <cstddef>
#include <map>
#include <memory>
#include <utility>
#include <vector>

class TClonesArray;
class AtRawEvent;
class AtEvent;
class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;
namespace DataHandling {
class ObserverPSA;
}

/**
 * @brief Abstract base class for processing AtPads (traces) into AtHits.
 *
 * @defgroup PSA
 */
class AtPSA {
private:
   // Access in PSA methods through getThreshold()
   Int_t fThreshold{-1};    ///< threshold of ADC value
   Int_t fThresholdlow{-1}; ///< threshold for Central pads

protected:
   TClonesArray *fMCSimPointArray{};

   Bool_t fUsingLowThreshold{false};

   // Variables from parameter file
   Double_t fBField{};
   Double_t fEField{};
   Int_t fTB0{};

   // This was hard coded too many places to leave as a variable...
   Int_t fNumTbs{512};        //< the number of time buckets used in taking data
   Int_t fTBTime{};           //< time duration of a time bucket in ns
   Int_t fEntTB{};            //< Timebucket of the entrance window
   Double_t fDriftVelocity{}; //< drift velocity of electron in cm/us
   Double_t fZk{};            //< Relative position of micromegas-cathode

   using HitVector = std::vector<std::unique_ptr<AtHit>>;

public:
   AtPSA() = default;
   virtual ~AtPSA() = default;

   virtual void Init();

   void SetThreshold(Int_t threshold);
   void SetThresholdLow(Int_t thresholdlow);

   void SetSimulatedEvent(TClonesArray *MCSimPointArray);

   virtual void Analyze(AtRawEvent *rawEvent, AtEvent *event);
   virtual HitVector AnalyzePad(AtPad *pad) = 0;

   // virtual HitVector AnalyzeTrace(const std::vector<double> &trace) = 0;
   virtual std::unique_ptr<AtPSA> Clone() = 0;

   friend class DataHandling::ObserverPSA;

protected:
   // Protected functions
   void TrackMCPoints(std::multimap<Int_t, std::size_t> &map, AtHit &hit); //< Assign MC Points kinematics to each hit.

   [[deprecated]] Double_t CalculateZ(Double_t peakIdx); ///< Calculate z position in mm using the peak index.

   Double_t CalculateZGeo(Double_t peakIdx);
   Double_t getThreshold(int padSize = -1);

   virtual double getZhitVariance(double zLoc, double zLocVar) const;
   virtual std::pair<double, double> getXYhitVariance() const;
   ClassDef(AtPSA, 5)
};

namespace DataHandling {

class SubjectPSA : public Subject {
protected:
   Int_t fThreshold{-1};    ///< threshold of ADC value
   Int_t fThresholdlow{-1}; ///< threshold for Central pads
public:
   SubjectPSA(int threshold, int thresholdLow = -1) : fThreshold(threshold), fThresholdlow(thresholdLow) {}

   int GetThreshold() { return fThreshold; }
   int GetThresholdLow() { return fThresholdlow; }
};

class ObserverPSA : public Observer {
protected:
   AtPSA *fPSA;

public:
   ObserverPSA(AtPSA *psa) : fPSA(psa) {}

   void Update(Subject *subject)
   {
      auto sub = dynamic_cast<SubjectPSA *>(subject);
      if (sub == nullptr)
         return;

      fPSA->SetThreshold(sub->GetThreshold());
      if (fPSA->fUsingLowThreshold)
         fPSA->SetThresholdLow(sub->GetThresholdLow());
   }
};

} // namespace DataHandling

#endif
