#include "AbsKalmanFitter.h"
#include "KalmanFitterRefTrack.h"
#include "DAF.h"
#include "ConstField.h"
#include "FieldManager.h"
#include "MaterialEffects.h"
#include "TGeoMaterialInterface.h"
#include "MeasurementFactory.h"
#include "MeasurementProducer.h"
#include "EventDisplay.h"
#include "KalmanFitStatus.h"
#include "FitStatus.h"
#include "AbsFitterInfo.h"
#include "KalmanFitterInfo.h"
#include "MeasuredStateOnPlane.h"
#include "MeasurementOnPlane.h"
#include "TrackPoint.h"
#include "Exception.h"

#include <ios>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <vector>

#include "TClonesArray.h"
#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TTreeReader.h"
#include "TTreePlayer.h"
#include "TTreeReaderValue.h"
#include "TSystem.h"
#include "TH1F.h"
#include "TH2F.h"
#include "TCanvas.h"
#include "TStopwatch.h"
#include "TGeoManager.h"
#include "TSpectrum.h"

#include "FairRootManager.h"
#include "FairLogger.h"
#include "FairRun.h"
#include "FairRunAna.h"

#include "AtTpcPoint.h"
#include "AtEvent.h"
#include "AtPad.h"
#include "AtHit.h"
#include "AtTrack.h"
#include "AtPatternEvent.h"

// ROOT
#include "TGraph.h"
#include "TCanvas.h"
#include "TApplication.h"
#include "TMath.h"
#include "TF1.h"
#include "TAxis.h"

#include "Math/Minimizer.h"
#include "Math/Factory.h"
#include "Math/Functor.h"
#include "Fit/Fitter.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TVectorD.h"

#include "Math/GenVector/Rotation3D.h"
#include "Math/GenVector/EulerAngles.h"
#include "Math/GenVector/AxisAngle.h"
#include "Math/GenVector/Quaternion.h"
#include "Math/GenVector/RotationX.h"
#include "Math/GenVector/RotationY.h"
#include "Math/GenVector/RotationZ.h"
#include "Math/GenVector/RotationZYX.h"

struct trackSegment {
   Double_t eLoss;
   TVector3 iniPos;
   TVector3 deltaMom;
   TVector3 deltaPos;
   Double_t theta;
   Double_t phi;
   UInt_t id;

   friend std::ostream &operator<<(std::ostream &os, const trackSegment &ts);
};

std::ostream &operator<<(std::ostream &os, const trackSegment &ts)
{
   os << "\n";
   os << " Track segment : " << ts.id << " - Momentum:  " << ts.deltaMom.X() << " - " << ts.deltaMom.Y() << " - "
      << ts.deltaMom.Z() << " - Energy Loss : " << ts.eLoss << "\n";
   os << " =============   - Position :  " << ts.iniPos.X() << " - " << ts.iniPos.Y() << " - " << ts.iniPos.Z()
      << " . Mag Dir : " << ts.iniPos.Mag() << "\n";
   os << " =============   - Position direction :  " << ts.deltaPos.X() << " - " << ts.deltaPos.Y() << " - "
      << ts.deltaPos.Z() << " . Mag Dir : " << ts.deltaPos.Mag() << "\n";
   os << " =============   - Theta    :  " << ts.theta * TMath::RadToDeg() << " - Phi : " << ts.phi * TMath::RadToDeg()
      << "\n";
   return os;
}

struct firstOrbit {
   Double_t POCA;
   Double_t Z;
   Double_t phi;
   Double_t length;
   Double_t eLoss;
};

std::tuple<Double_t, Double_t>
GetMomFromBrho(Double_t A, Double_t Z, Double_t brho); ///< Returns momentum (in GeV) from Brho assuming M (amu) and Z;
double
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);
Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}
Double_t GetNPeaksHRS(std::vector<Int_t> *timeMax, std::vector<Float_t> *adcMax, double *adc_test);
double GetMaximum(double *adc);
void ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius);
void Clusterize3D(AtTrack &track, Float_t distance, Float_t radius);
void Clusterize(AtTrack &track);
firstOrbit GetFirstOrbit(genfit::Track *track, genfit::AbsTrackRep *rep, TVector3 vertex);
void ConstructTrack(const genfit::StateOnPlane *prevState, const genfit::StateOnPlane *state,
                    const genfit::AbsTrackRep *rep, std::vector<TVector3> &track, std::vector<trackSegment> &segments);
