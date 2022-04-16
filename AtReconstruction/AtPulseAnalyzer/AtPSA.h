#ifndef AtPSA_H
#define AtPSA_H

#include <Rtypes.h>
#include <TString.h>
#include <TVector3.h>
// STL
#include "AtCalibration.h"

#include <cstddef>
#include <map>
#include <memory>
#include <utility>

class TClonesArray;
// AtTPCROOT classes
class AtRawEvent;
class AtEvent;
class AtDigiPar;
class AtHit;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSA {
public:
   AtPSA() = default;
   virtual ~AtPSA() = default;

   void Init();
   //! Setting threshold
   void SetThreshold(Int_t threshold);
   void SetThresholdLow(Int_t thresholdlow);

   void SetGainCalibration(TString gainFile);
   void SetJitterCalibration(TString jitterFile);

   void SetTBLimits(std::pair<Int_t, Int_t> limits);

   void SetSimulatedEvent(TClonesArray *MCSimPointArray);

   virtual void Analyze(AtRawEvent *rawEvent, AtEvent *event) = 0;

protected:
   AtDigiPar *fPar{}; ///< parameter container

   Bool_t fIsGainCalibrated{false};
   Bool_t fIsJitterCalibrated{false};

   std::unique_ptr<AtCalibration> fCalibration;
   TClonesArray *fMCSimPointArray{};

   Int_t fThreshold{-1};    ///< threshold of ADC value
   Int_t fThresholdlow{-1}; ///< threshold for Central pads
   Bool_t fUsingLowThreshold{false};

   Int_t fIniTB{0};   /// First TB for charge integration
   Int_t fEndTB{512}; /// Last TB for charge integration

   // Variables from parameter file
   Double_t fBField{};
   Double_t fEField{};
   Double_t fTiltAng{};
   TVector3 fLorentzVector;
   Int_t fTB0{};
   Double_t fThetaPad{};

   Int_t fPadPlaneX{}; ///< pad plane size x in mm
   Int_t fPadSizeX{};  ///< pad size x in mm
   Int_t fPadSizeZ{};  ///< pad size y in mm
   Int_t fPadRows{};   ///< number of total pad rows
   Int_t fPadLayers{}; ///< number of total pad layers

   // This was hard coded too many places to leave as a variable...
   Int_t fNumTbs{512}; ///< the number of time buckets used in taking data
   Int_t fTBTime{};    ///< time duration of a time bucket in ns
   Int_t fEntTB{};
   Double_t fDriftVelocity{};  ///< drift velocity of electron in cm/us
   Double_t fMaxDriftLength{}; ///< maximum drift length in mm
   Double_t fZk{};             // Relative position of micromegas-cathode

   // Protected functions
   void TrackMCPoints(std::multimap<Int_t, std::size_t> &map, AtHit &hit); //< Assign MC Points kinematics to each hit.

   Double_t
   CalculateX(Double_t row); ///< Calculate x position in mm. This returns the center position of given pad row.
   Double_t CalculateZ(Double_t peakIdx); ///< Calculate z position in mm using the peak index.
   Double_t
   CalculateY(Double_t layer); ///< Calculate y position in mm. This returns the center position of given pad layer.

   Double_t CalculateZGeo(Double_t peakIdx);

   Double_t CalculateXCorr(Double_t xval, Int_t Tbx);
   Double_t CalculateYCorr(Double_t yval, Int_t Tby);
   Double_t CalculateZCorr(Double_t zval, Int_t Tbz);

   void CalcLorentzVector();

   TVector3 RotateDetector(Double_t x, Double_t y, Double_t z, Int_t tb);

   ClassDef(AtPSA, 4)
};

#endif
