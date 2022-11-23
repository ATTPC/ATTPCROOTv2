#include "AtTPCIonGeneratorS800.h"

#include "AtVertexPropagator.h"

#include <FairLogger.h>

#include <TFile.h>
#include <TMath.h>
#include <TObject.h> // for TObject
#include <TRandom.h>

#include <cmath> // for tan, sqrt, pow, atan, fabs

ClassImp(AtTPCIonGeneratorS800);

AtTPCIonGeneratorS800::AtTPCIonGeneratorS800() : AtTPCIonGenerator(), fAta(nullptr), fBta(nullptr) {}

AtTPCIonGeneratorS800::AtTPCIonGeneratorS800(const char *name, Int_t z, Int_t a, Int_t q, Int_t mult, Double_t px,
                                             Double_t py, Double_t pz, Double_t Ex, Double_t m, Double_t ener,
                                             Double_t eLoss, TString sata, TString sbta)
   : AtTPCIonGenerator(name, z, a, q, mult, px, py, pz, Ex, m, eLoss), fAta(nullptr), fBta(nullptr)

{

   TFile fileAta(sata, "READ");
   TFile fileBta(sbta, "READ");
   if (fileAta.IsZombie() || fileBta.IsZombie())
      LOG(error) << "AtTPCIonGenerator - ata and bta distribution files (S800) not found";
   else {
      fAta = std::unique_ptr<TH1F>(dynamic_cast<TH1F *>(fileAta.Get("h")));
      fBta = std::unique_ptr<TH1F>(dynamic_cast<TH1F *>(fileBta.Get("h1")));
      fAta->SetDirectory(nullptr);
      fBta->SetDirectory(nullptr);
   }

   fPx0 = a * px;
   fPy0 = a * py;
   fPz0 = a * pz;
}

void AtTPCIonGeneratorS800::SetBeamEmittance(Double32_t val1, Double32_t val2, Double32_t val3, Double32_t val4,
                                             Double_t val5, Double_t val6, Double_t val7, Double_t val8, Double_t val9)
{
   fWhmFocus = val1;
   fDiv = val2;
   fZFocus = val3;
   fRHole = val4;
   fMomAcc = val5;
   fBeamAx = val6;
   fBeamAy = val7;
   fBeamOx = val8;
   fBeamOy = val9;
}

void AtTPCIonGeneratorS800::SetVertexCoordinates()
{
   // TStopwatch timer;
   // timer.Start();
   gRandom->SetSeed(0);
   Double_t x = 0., y = 0., xFocus = 0., yFocus = 0., Ax = 0., Ay = 0., BeamAx = 0., BeamAy = 0., BeamOx = 0.,
            BeamOy = 0.;
   Double_t ptot = sqrt(pow(fPx0, 2) + pow(fPy0, 2) + pow(fPz0, 2));
   // ptot=gRandom->Uniform(ptot*(1.-fMomAcc),ptot*(1.+fMomAcc));
   // following "do wile" for gaussian beam momentum distribution with boundaries
   do {
      ptot = gRandom->Gaus(ptot, ptot * fMomAcc / 2.355);
   } while (ptot < ptot * (1. - 2. * fMomAcc) || ptot > ptot * (1. + 2. * fMomAcc));
   BeamAx = fBeamAx * TMath::DegToRad();
   BeamAy = fBeamAy * TMath::DegToRad();

   // x is a coordinate of beam particle at ATTPC entrance, xFocus is a coordinate at focus.
   do {
      xFocus = gRandom->Gaus(fBeamOx, fWhmFocus / 2.355) + fZFocus * tan(BeamAx);
      yFocus = gRandom->Gaus(fBeamOy, fWhmFocus / 2.355) + fZFocus * tan(BeamAy);
   } // beam spot smaller than the entrance hole
   while (sqrt(pow((xFocus - fZFocus * tan(BeamAx)), 2) + pow((yFocus - fZFocus * tan(BeamAy)), 2)) > fRHole);

   if (fAta != nullptr && fBta != nullptr) { // with ata and bta distributions from S800 data
      do {
         Ax = fAta->GetRandom() + 0.0019; // offset between angle of the beam in tpc frame and S800 frame,
         // Ax = fAta.GetRandom() + 0.0019; // offset between angle of the beam in tpc frame and S800 frame,
         //(+) because angle more negative in S800 frame than in TPC frame. Needs to correct back this offset in this
         // analysis
         Ay = fBta->GetRandom();
         // Ay = fBta.GetRandom();
         x = xFocus - fZFocus * tan(Ax);
         y = yFocus - fZFocus * tan(Ay);
      } // beam at entrance smaller than the hole
      while (sqrt(pow(x, 2) + pow(y, 2)) > fRHole);
   } else {
      do {
         x = gRandom->Gaus(fBeamOx, fWhmFocus / 2.355 + fZFocus * tan(fDiv));
         y = gRandom->Gaus(fBeamOy, fWhmFocus / 2.355 + fZFocus * tan(fDiv));
         Ax = atan((xFocus - x) / fZFocus);
         Ay = atan((yFocus - y) / fZFocus);
      } while (sqrt(pow(x, 2) + pow(y, 2)) > fRHole ||
               sqrt(pow(tan(Ax - BeamAx), 2) + pow(tan(Ay - BeamAy), 2)) > fabs(tan(fDiv)));
   }

   fVx = x;
   fVy = y;
   fVz = 0.;

   // std::cout<<"ATTPCIonGenerator beam X,Y at entrance "<<x<<" "<<y<<" at focus "<<xFocus<<" "<<yFocus<<std::endl;

   fPz = ptot / sqrt(1. + pow(tan(Ax), 2) + pow(tan(Ay), 2));
   fPx = fPz * tan(Ax);
   fPy = fPz * tan(Ay);

   AtVertexPropagator::Instance()->Setd2HeVtx(fVx, fVy, Ax, Ay);

   // timer.Stop();
   // Double_t rtime = timer.RealTime();
   // Double_t ctime = timer.CpuTime();

   // std::cout << "Real time " << rtime << " s, CPU time " << ctime << "s" <<std::endl;
}
