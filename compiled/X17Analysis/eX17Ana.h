#include <ios>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <vector>

#include <TClonesArray.h>
#include <TString.h>
#include <TFile.h>
#include <TTree.h>
#include <TTreeReader.h>
#include <TTreePlayer.h>
#include <TTreeReaderValue.h>
#include <TSystem.h>
#include <TH1F.h>
#include <TH2F.h>
#include <TCanvas.h>
#include <TStopwatch.h>
#include <TGeoManager.h>
#include <TSpectrum.h>

#include <FairRootManager.h>
#include <FairLogger.h>
#include <FairRun.h>
#include <FairRunAna.h>

#include "AtTpcPoint.h"
#include "AtEvent.h"
#include "AtPad.h"
#include "AtHit.h"
#include "AtTrack.h"
#include "AtPatternEvent.h"

// ROOT
#include <TGraph.h>
#include <TCanvas.h>
#include <TApplication.h>
#include <TMath.h>
#include <TF1.h>
#include <TAxis.h>

#include "Math/Minimizer.h"
#include "Math/Factory.h"
#include "Math/Functor.h"
#include "Fit/Fitter.h"
#include <TRotation.h>
#include <TMatrixD.h>
#include <TArrayD.h>
#include <TVectorD.h>

#include "Math/GenVector/Rotation3D.h"
#include "Math/GenVector/EulerAngles.h"
#include "Math/GenVector/AxisAngle.h"
#include "Math/GenVector/Quaternion.h"
#include "Math/GenVector/RotationX.h"
#include "Math/GenVector/RotationY.h"
#include "Math/GenVector/RotationZ.h"
#include "Math/GenVector/RotationZYX.h"
