#ifndef ATPULSE_H
#define ATPULSE_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <TF1.h> //Needed for unique_ptr<TF1>
#include <TH1.h> //Needed for unique_ptr<TH1F>

#include <functional> // for function
#include <memory>     // for unique_ptr, shared_ptr
#include <set>
#include <type_traits> // for add_pointer_t
#include <vector>      // for vector
class AtMap;
class AtSimulatedPoint;
class AtRawEvent;
class AtDigiPar;
class AtPad;

class AtPulse {
protected:
   using AtMapPtr = std::shared_ptr<AtMap>;
   using ResponseFunctionType = std::add_pointer_t<double(double)>;
   /// Function (or callable object) to use as the response function. Parameters are padNum and time (us)
   using ResponseFunc = std::function<double(int, double)>;
   using XYZVector = ROOT::Math::XYZVector;
   using XYZPoint = ROOT::Math::XYZPoint;
   using SimPointPtr = std::unique_ptr<AtSimulatedPoint>;

   AtMapPtr fMap; //!< AtTPC map

   int fEventID = 0;          //! EventID
   double fGain = 0;          //! Micromegas gain.
   double fLowGainFactor = 0; //! If pad is AtMap::kLowGain multiply gain by this factor
   double fGETGain = 0;       //! GET Gain (ADC ch/electron).
   double fPeakingTime = 0;   //! Electronic peaking time in us
   double fTBTime = 0;        //! Time bucket size in us
   int fNumTbs{512};          //! Number of time buckers
   int fTBEntrance = 0;       //! Window location in timebuckets (from config)
   int fTBPadPlane = 0;       //! Pad plane location in TBs (calculated from DriftVelocity, TBEntrance, ZPadPlane

   ResponseFunc fResponse; //! Response function of the electronics
   bool fUseFastGain = true;
   double fNoiseSigma = 0; //! Sigma of random gaussian noise to apply to trace
   bool fSaveCharge = true;
   bool fDoConvolution{true}; //< Whether we should set the ADC by doing a convolution of the charge with the response

   std::vector<std::unique_ptr<TH1F>> fPadCharge; //!<
   std::set<int> fPadsWithCharge;                 //!<

   std::unique_ptr<TF1> fGainFunc; //!<
   double fAvgGainDeviation{};

public:
   AtPulse(AtMapPtr map, ResponseFunc response = nullptr);
   AtPulse(const AtPulse &);

   void SetParameters(const AtDigiPar *fPar);
   AtMapPtr GetMap() { return fMap; }
   void UseFastGain(bool val) { fUseFastGain = val; }
   void SetNoiseSigma(double val) { fNoiseSigma = val; }
   void SetSaveCharge(bool val) { fSaveCharge = val; }
   void SetDoConvolution(bool val) { fDoConvolution = val; }
   void SetLowGain(double val) { fLowGainFactor = val; }

   AtRawEvent GenerateEvent(std::vector<SimPointPtr> &vec);
   virtual AtRawEvent GenerateEvent(std::vector<AtSimulatedPoint *> &vec);

   virtual std::shared_ptr<AtPulse> Clone() const { return std::make_shared<AtPulse>(*this); }

protected:
   void Reset();
   virtual bool AssignElectronsToPad(AtSimulatedPoint *point);
   double GetGain(int padNum, int numElectrons);
   void GenerateTraceFromElectrons();
   void FillPad(AtPad &pad, TH1F &hist);
   void ApplyNoise(AtPad &pad);
};

#endif // ATPULSE_H
