#include "AtEvent.h"
#include "AtHit.h"
#include "AtMCPoint.h"
#include "AtPad.h"
#include "AtPatternEvent.h"
#include "AtTrack.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRunAna.h>

#include <TCanvas.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TGeoManager.h>
#include <TGeoMaterialInterface.h>
#include <TH1F.h>
#include <TH2F.h>
#include <TStopwatch.h>
#include <TString.h>
#include <TSystem.h>
#include <TTree.h>
#include <TTreePlayer.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>

#include "AbsFitterInfo.h"
#include "AbsKalmanFitter.h"
#include "ConstField.h"
#include "DAF.h"
#include "EventDisplay.h"
#include "Exception.h"
#include "FieldManager.h"
#include "FitStatus.h"
#include "KalmanFitStatus.h"
#include "KalmanFitterInfo.h"
#include "KalmanFitterRefTrack.h"
#include "MaterialEffects.h"
#include "MeasurementFactory.h"
#include "MeasurementProducer.h"

#include <ios>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <vector>

// ROOT
#include <TApplication.h>
#include <TArrayD.h>
#include <TAxis.h>
#include <TCanvas.h>
#include <TF1.h>
#include <TGraph.h>
#include <TMath.h>
#include <TMatrixD.h>
#include <TRotation.h>
#include <TVectorD.h>

#include "Fit/Fitter.h"
#include "Math/Factory.h"
#include "Math/Functor.h"
#include "Math/GenVector/AxisAngle.h"
#include "Math/GenVector/EulerAngles.h"
#include "Math/GenVector/Quaternion.h"
#include "Math/GenVector/Rotation3D.h"
#include "Math/GenVector/RotationX.h"
#include "Math/GenVector/RotationY.h"
#include "Math/GenVector/RotationZ.h"
#include "Math/GenVector/RotationZYX.h"
#include "Math/Minimizer.h"

std::tuple<Double_t, Double_t>
GetMomFromBrho(Double_t A, Double_t Z, Double_t brho); ///< Returns momentum (in GeV) from Brho assuming M (amu) and Z;
double
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);
Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}
double GetMaximum(const double *adc);
