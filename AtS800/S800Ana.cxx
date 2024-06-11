#include "S800Ana.h"

#include <FairLogger.h>

#include <TCollection.h>
#include <TCutG.h>
#include <TFile.h>
#include <TKey.h>
#include <TList.h>
#include <TObject.h>
#include <TString.h>

#include "S800Calc.h"

#include <algorithm>
#include <cmath>
#include <iostream>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(S800Ana);

S800Ana::S800Ana()
   : fLogger(FairLogger::GetLogger()), fXfObj_ToF(-999), fObjCorr_ToF(-999), fICSum_E(-999), fX0(-999), fX1(-999),
     fAfp(-999)
{
   fcutPID1.clear();
   fcutPID2.clear();
   fcutPID3.clear();

   fParameters.clear();
   fTofObjCorr.clear();
   fMTDCObjRange.clear();
   fMTDCXfRange.clear();
}

void S800Ana::Reset()
{
   fXfObj_ToF = -999;
   fObjCorr_ToF = -999;
   fICSum_E = -999;
   fX0 = -999;
   fX1 = -999;
   fY0 = -999;
   fY1 = -999;
   fAfp = -999;
   fBfp = -999;
   return;
}

void S800Ana::SetPID1cut(std::vector<TString> files)
{
   for (auto &w : files) {
      TFile f(w);
      TIter next(f.GetListOfKeys());
      TKey *key = nullptr;

      while ((key = dynamic_cast<TKey *>(next()))) {
         LOG(info) << "PID1 Loading Cut file:  " << key->GetName();
         fcutPID1.push_back(dynamic_cast<TCutG *>(f.Get(key->GetName())));
      }
   }
}
void S800Ana::SetPID2cut(std::vector<TString> files)
{
   for (auto &w : files) {
      TFile f(w);
      TIter next(f.GetListOfKeys());
      TKey *key = nullptr;

      while ((key = dynamic_cast<TKey *>(next()))) {
         LOG(info) << "PID2 Loading Cut file:  " << key->GetName();
         fcutPID2.push_back(dynamic_cast<TCutG *>(f.Get(key->GetName())));
      }
   }
}
void S800Ana::SetPID3cut(std::vector<TString> files)
{
   for (auto &w : files) {
      TFile f(w);
      TIter next(f.GetListOfKeys());
      TKey *key = nullptr;

      while ((key = dynamic_cast<TKey *>(next()))) {
         LOG(info) << "PID3 Loading Cut file:  " << key->GetName();
         fcutPID3.push_back(dynamic_cast<TCutG *>(f.Get(key->GetName())));
      }
   }
}
void S800Ana::SetParameters(std::vector<Double_t> vec)
{
   fParameters = vec;
}
void S800Ana::SetTofObjCorr(std::vector<Double_t> vec)
{
   fTofObjCorr = vec;
}
void S800Ana::SetMTDCObjRange(std::vector<Double_t> vec)
{
   fMTDCObjRange = vec;
}
void S800Ana::SetMTDCXfRange(std::vector<Double_t> vec)
{
   fMTDCXfRange = vec;
}

vector<Double_t> S800Ana::GetTofObjCorr()
{
   return fTofObjCorr;
}
vector<Double_t> S800Ana::GetMTDCObjRange()
{
   return fMTDCObjRange;
}
vector<Double_t> S800Ana::GetMTDCXfRange()
{
   return fMTDCXfRange;
}
vector<Double_t> S800Ana::GetParameters()
{
   return fParameters;
}
Double_t S800Ana::GetXfObj_ToF()
{
   return fXfObj_ToF;
}
Double_t S800Ana::GetObjCorr_ToF()
{
   return fObjCorr_ToF;
}
Double_t S800Ana::GetICSum_E()
{
   return fICSum_E;
}
std::vector<Double_t> S800Ana::GetFpVariables()
{
   std::vector<Double_t> result;
   result.push_back(fX0);
   result.push_back(fX1);
   result.push_back(fY0);
   result.push_back(fY1);
   result.push_back(fAfp);
   result.push_back(fBfp);
   return result;
}

void S800Ana::Calc(S800Calc *s800calc)
{
   Reset();
   if (fMTDCXfRange.size() < 2 || fMTDCObjRange.size() < 2 || fTofObjCorr.size() < 2) {
      LOG(warning) << "S800Ana::Calc : MTDCXfRange, MTDCObjRange, TofObjCorr size must be 2 : " << fMTDCXfRange.size()
                   << " " << fMTDCObjRange.size() << " " << fTofObjCorr.size() << " ... skip event";
      return;
   }
   // Double_t S800_timeRf = s800calc->GetMultiHitTOF()->GetFirstRfHit();
   // Double_t S800_timeE1up = s800calc->GetMultiHitTOF()->GetFirstE1UpHit();
   // Double_t S800_timeE1down = s800calc->GetMultiHitTOF()->GetFirstE1DownHit();
   // Double_t S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
   // Double_t S800_timeXf = s800calc->GetMultiHitTOF()->GetFirstXfHit();
   // Double_t S800_timeObj = s800calc->GetMultiHitTOF()->GetFirstObjHit();
   vector<Float_t> S800_timeMTDCObj = s800calc->GetMultiHitTOF()->GetMTDCObj();
   vector<Float_t> S800_timeMTDCXf = s800calc->GetMultiHitTOF()->GetMTDCXf();
   Float_t S800_timeObjSelect = -999;
   Float_t S800_timeXfSelect = -999;
   Int_t CondMTDCXfObj = 0;

   fX0 = s800calc->GetCRDC(0)->GetX();
   fX1 = s800calc->GetCRDC(1)->GetX();
   fY0 = s800calc->GetCRDC(0)->GetY();
   fY1 = s800calc->GetCRDC(1)->GetY();
   // Double_t S800_E1up = s800calc->GetSCINT(0)->GetDEup();
   // Double_t S800_E1down = s800calc->GetSCINT(0)->GetDEdown();

   fICSum_E = s800calc->GetIC()->GetSum();

   fAfp = atan((fX1 - fX0) / 1073.);
   fBfp = atan((fY1 - fY0) / 1073.);
   // Double_t S800_tofCorr = S800_tof + x0_corr_tof*fX0 + afp_corr_tof*fAfp;// - rf_offset;
   // Double_t S800_dE = s800calc->GetSCINT(0)->GetDE();//check if is this scint (0)
   // Double_t S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
   // Double_t S800_dECorr = S800_dE + afp_corr_dE*fAfp + x0_corr_dE*fabs(fX0);

   for (float k : S800_timeMTDCXf) {
      if (k > fMTDCXfRange.at(0) && k < fMTDCXfRange.at(1))
         S800_timeXfSelect = k;
   }
   for (float k : S800_timeMTDCObj) {
      if (k > fMTDCObjRange.at(0) && k < fMTDCObjRange.at(1))
         S800_timeObjSelect = k;
   }

   fXfObj_ToF = S800_timeXfSelect - S800_timeObjSelect;

   if (S800_timeXfSelect != -999 && S800_timeObjSelect != -999) {
      fXfObj_ToF = S800_timeXfSelect - S800_timeObjSelect;
      CondMTDCXfObj = 1;
   }

   if (CondMTDCXfObj && std::isnan(fX0) == 0 && std::isnan(fAfp) == 0 && std::isnan(fICSum_E) == 0) {
      fObjCorr_ToF = S800_timeObjSelect + fTofObjCorr.at(0) * fAfp + fTofObjCorr.at(1) * fX0;
   }

   return;
}

Bool_t S800Ana::isInPID(S800Calc *s800calc)
{

   Calc(s800calc);

   Bool_t is = kFALSE;
   Int_t InCondition1 = 0;
   Int_t InCondition2 = 0;
   Int_t InCondition3 = 0;

   // std::cout << " S800Ana : Test var  " << fObjCorr_ToF << " " << fXfObj_ToF << " " << fX0 << " "
   //           << fAfp << " " << fObjCorr_ToF << " " << fICSum_E << '\n';

   for (auto &w : fcutPID1)
      if (fObjCorr_ToF != -999 && w->IsInside(fObjCorr_ToF, fXfObj_ToF))
         InCondition1 += 1; // or of PID1
   for (auto &w : fcutPID2)
      if (fObjCorr_ToF != -999 && w->IsInside(fX0, fAfp))
         InCondition2 += 1; // or of PID2
   for (auto &w : fcutPID3)
      if (fObjCorr_ToF != -999 && w->IsInside(fObjCorr_ToF, fICSum_E))
         InCondition3 += 1; // or of PID3

   if (fcutPID1.size() == 0)
      InCondition1 = 1;
   if (fcutPID2.size() == 0)
      InCondition2 = 1;
   if (fcutPID3.size() == 0)
      InCondition3 = 1;

   // std::cout << " S800Ana : Number of TCutG files  " << fcutPID1.size() << " " << fcutPID2.size() << " " <<
   // fcutPID3.size() << " "
   //           << InCondition1 << " " << InCondition2 << " " << InCondition3 << '\n';

   Int_t AndofCon = InCondition1 * InCondition2 * InCondition3;

   if (AndofCon) {
      is = kTRUE;
      std::cout << cGREEN << "'And' of S800 PID gates TRUE" << cNORMAL << std::endl;
   }
   //----------- New 10/01 -------------------------------------
   return is;
}
