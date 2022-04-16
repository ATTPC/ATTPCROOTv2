#include "AtEvent.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtPatternEvent.h"
#include "AtTpcPoint.h"
#include "AtTrack.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRunAna.h>

#include <TCanvas.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TGeoManager.h>
#include <TH1F.h>
#include <TH2F.h>
#include <TSpectrum.h>
#include <TStopwatch.h>
#include <TString.h>
#include <TSystem.h>
#include <TTree.h>
#include <TTreePlayer.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>

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
