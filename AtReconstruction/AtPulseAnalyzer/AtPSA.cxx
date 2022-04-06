#include "AtPSA.h"

#include <fairlogger/Logger.h>
#include <pstl/glue_algorithm_defs.h>
#include <iostream>
#include <iterator>

// FairRoot classes
#include <FairRuntimeDb.h>
#include <FairRun.h>
// ROOT classes
#include <TClonesArray.h>
#include <TVector3.h>
#include <TMath.h>
#include "AtDigiPar.h"
#include "AtCalibration.h"
#include "AtHit.h"
#include "AtMCPoint.h"
#include "Rtypes.h"

using std::distance;
using std::max_element;
using std::min_element;

AtPSA::AtPSA()
{
   std::cout << "Calling AtPSA Constructor" << std::endl;

   // TODO:Move to class that needs them
   fIniTB = 0;
   fEndTB = 512;

   fThreshold = -1;
   fThresholdlow = -1;
   fUsingLowThreshold = kFALSE;

   fIsGainCalibrated = kFALSE;
   fIsJitterCalibrated = kFALSE;
}

AtPSA::~AtPSA()
{

   delete fCalibration;
}

void AtPSA::Init()
{
   fCalibration = new AtCalibration();

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(FATAL) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(FATAL) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      LOG(FATAL) << "AtDigiPar not found!!";

   fPadPlaneX = fPar->GetPadPlaneX();
   fPadSizeX = fPar->GetPadSizeX();
   fPadSizeZ = fPar->GetPadSizeZ();
   fPadRows = fPar->GetPadRows();
   fPadLayers = fPar->GetPadLayers();
   fNumTbs = fPar->GetNumTbs();
   fTBTime = fPar->GetTBTime();
   fDriftVelocity = fPar->GetDriftVelocity();
   fMaxDriftLength = fPar->GetDriftLength();
   fBField = fPar->GetBField();
   fEField = fPar->GetEField();
   fTiltAng = fPar->GetTiltAngle();
   fTB0 = fPar->GetTB0();
   fZk = fPar->GetZPadPlane();
   fEntTB = (Int_t)fPar->GetTBEntrance();

   fThetaPad = -103.0 * TMath::Pi() / 180.0;

   std::cout << " ==== Parameters for Pulse Shape Analysis Task ==== " << std::endl;
   std::cout << " ==== Magnetic Field : " << fBField << " T " << std::endl;
   std::cout << " ==== Electric Field : " << fEField << " V/cm " << std::endl;
   std::cout << " ==== Sampling Rate : " << fTBTime << " ns " << std::endl;
   std::cout << " ==== Tilting Angle : " << fTiltAng << " deg " << std::endl;
   std::cout << " ==== Drift Velocity : " << fDriftVelocity << " cm/us " << std::endl;
   std::cout << " ==== TB0 : " << fTB0 << std::endl;
   std::cout << " ==== NumTbs : " << fNumTbs << std::endl;
}

void AtPSA::SetSimulatedEvent(TClonesArray *MCSimPointArray)
{
   fMCSimPointArray = MCSimPointArray;
}

void AtPSA::SetThreshold(Int_t threshold)
{
   fThreshold = threshold;
   if (!fUsingLowThreshold)
      fThresholdlow = threshold;
}

void AtPSA::SetThresholdLow(Int_t thresholdlow)
{
   fThresholdlow = thresholdlow;
   fUsingLowThreshold = kTRUE;
}

Double_t AtPSA::CalculateX(Double_t row)
{
   return (row + 0.5) * fPadSizeX - fPadPlaneX / 2.;
}

Double_t AtPSA::CalculateZ(Double_t peakIdx)
{
   // DEPRECAtED
   return (fNumTbs - peakIdx) * fTBTime * fDriftVelocity / 100.;
}

Double_t AtPSA::CalculateZGeo(Double_t peakIdx)
{

   // This function must be consistent with the re-calibrations done before.
   return fZk - (fEntTB - peakIdx) * fTBTime * fDriftVelocity / 100.;
}

Double_t AtPSA::CalculateY(Double_t layer)
{
   return (layer + 0.5) * fPadSizeZ;
}

Double_t
AtPSA::CalculateXCorr(Double_t xvalue,
                      Int_t Tbx) // TODO: Yes, this can be done with one stupid function but try walking on my shoes...
{

   Double_t xcorr = xvalue - fLorentzVector.X() * (Tbx - fTB0) * fTBTime * 1E-2; // Convert from ns to us and cm to mm
   return xcorr;
}

Double_t AtPSA::CalculateYCorr(Double_t yvalue, Int_t Tby)
{
   Double_t ycorr = yvalue + fLorentzVector.Y() * (Tby - fTB0) * fTBTime * 1E-2;
   return ycorr;
}

Double_t AtPSA::CalculateZCorr(Double_t zvalue, Int_t Tbz)
{
   Double_t zcorr = fLorentzVector.Z() * (Tbz - fTB0) * fTBTime * 1E-2;
   return zcorr;
}

void AtPSA::CalcLorentzVector()
{

   // fDriftVelocity*=-1;// TODO: Check sign of the Vd
   Double_t ot = (fBField / fEField) * fDriftVelocity * 1E4;
   Double_t front = fDriftVelocity / (1 + ot * ot);
   Double_t TiltRad = fTiltAng * TMath::Pi() / 180.0;

   Double_t x = front * ot * TMath::Sin(TiltRad);
   Double_t y = front * ot * ot * TMath::Cos(TiltRad) * TMath::Sin(TiltRad);
   Double_t z = front * (1 + ot * ot * TMath::Cos(TiltRad) * TMath::Cos(TiltRad));

   fLorentzVector.SetXYZ(x, y, z);

   //  std::cout<<" Vdx : "<<x<<std::endl;
   //   std::cout<<" Vdy : "<<y<<std::endl;
   //  std::cout<<" Vdz : "<<z<<std::endl;
}

TVector3 AtPSA::RotateDetector(Double_t x, Double_t y, Double_t z, Int_t tb)
{

   // DEPRECAtED because of timebucket calibration (-271.0)
   TVector3 posRot;
   TVector3 posDet;

   posRot.SetX(x * TMath::Cos(fThetaPad) - y * TMath::Sin(fThetaPad));
   posRot.SetY(x * TMath::Sin(fThetaPad) + y * TMath::Cos(fThetaPad));
   posRot.SetZ((-271.0 + tb) * fTBTime * fDriftVelocity / 100. + fZk);

   Double_t TiltAng = -fTiltAng * TMath::Pi() / 180.0;

   posDet.SetX(posRot.X());
   posDet.SetY(-(fZk - posRot.Z()) * TMath::Sin(TiltAng) + posRot.Y() * TMath::Cos(TiltAng));
   posDet.SetZ(posRot.Z() * TMath::Cos(TiltAng) - posRot.Y() * TMath::Sin(TiltAng));

   return posDet;
}

void AtPSA::SetGainCalibration(TString gainFile)
{
   fCalibration->SetGainFile(gainFile);
}

void AtPSA::SetJitterCalibration(TString jitterFile)
{
   fCalibration->SetJitterFile(jitterFile);
}

void AtPSA::SetTBLimits(std::pair<Int_t, Int_t> limits)
{
   if (limits.first >= limits.second) {
      std::cout << " Warning AtPSA::SetTBLimits -  Wrong Time Bucket limits. Setting default limits (0,512) ... "
                << "\n";
      fIniTB = 0;
      fEndTB = 512;

   } else {
      fIniTB = limits.first;
      fEndTB = limits.second;
   }
}

void AtPSA::TrackMCPoints(std::multimap<Int_t, std::size_t> &map, AtHit &hit)
{
   auto padNum = hit.GetPadNum();
   for (auto it = map.lower_bound(padNum); it != map.upper_bound(padNum); ++it) {

      if (fMCSimPointArray != nullptr) {
         AtMCPoint *MCPoint = (AtMCPoint *)fMCSimPointArray->At(it->second);

         AtHit::MCSimPoint mcpoint(it->second, MCPoint->GetTrackID(), MCPoint->GetEIni(), MCPoint->GetEnergyLoss(),
                                   MCPoint->GetAIni(), MCPoint->GetMassNum(), MCPoint->GetAtomicNum());
         hit.AddMCSimPoint(mcpoint);

         // std::cout << " Pad Num : "<<hit->GetHitPadNum()<<" MC Point ID : "<<it->second << std::endl;
         // std::cout << " Track ID : "<<MCPoint->GetTrackID()<<" Energy (MeV) : "<<MCPoint->GetEIni()<<" Angle (deg) :
         // "<<MCPoint->GetAIni()<<"\n"; std::cout << " Mass Number : "<<MCPoint->GetMassNum()<<" Atomic Number
         // "<<MCPoint->GetAtomicNum()<<"\n";
      }
      //  }
   }
}

ClassImp(AtPSA)
