#ifndef AtPSA_H
#define AtPSA_H

#include <Rtypes.h>

#include <cstddef>
#include <map>
#include <memory>
#include <vector>

class TClonesArray;
class AtRawEvent;
class AtEvent;
class AtHit;
class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

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

   void Init();

   void SetThreshold(Int_t threshold);
   void SetThresholdLow(Int_t thresholdlow);

   void SetSimulatedEvent(TClonesArray *MCSimPointArray);

   virtual void Analyze(AtRawEvent *rawEvent, AtEvent *event);
   virtual HitVector AnalyzePad(AtPad *pad) = 0;
   virtual std::unique_ptr<AtPSA> Clone() = 0;

protected:
   // Protected functions
   void TrackMCPoints(std::multimap<Int_t, std::size_t> &map, AtHit &hit); //< Assign MC Points kinematics to each hit.

   [[deprecated]] Double_t CalculateZ(Double_t peakIdx); ///< Calculate z position in mm using the peak index.

   Double_t CalculateZGeo(Double_t peakIdx);
   Double_t getThreshold(int padSize = -1);

   ClassDef(AtPSA, 5)
};

#endif
