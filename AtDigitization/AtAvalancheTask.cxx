#include "AtAvalancheTask.h"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "AtMCPoint.h"
#include "AtVertexPropagator.h"

// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TRandom.h"
#include "TMath.h"

AtAvalancheTask::AtAvalancheTask() : FairTask("AtAvalanacheTask"), fEventID(0) {}

AtAvalancheTask::~AtAvalancheTask()
{
   LOG(debug) << "Destructor of AtAvalancheTask";
}

void AtAvalancheTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtAvalancheTask";

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = (AtDigiPar *)rtdb->getContainer("AtDigiPar");
}

InitStatus AtAvalancheTask::Init()
{
   LOG(debug) << "Initilization of AtAvalancheTask";

   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = (TClonesArray *)ioman->GetObject("AtMCPoint");
   if (fMCPointArray == 0) {
      LOG(error) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   // fElectronArray = new TClonesArray("STDriftedElectron");
   // ioman->Register("STDriftedElectron","ST",fElectronArray,fInputPersistance);

   fGas = fPar->GetGas();

   fEIonize = fGas->GetEIonize() * 1.E6;               // [MeV] to [eV]
   fVelDrift = fGas->GetDriftVelocity() / 100.;        // [cm/us] to [mm/ns]
   fCoefT = fGas->GetCoefDiffusionTrans() * sqrt(10.); // [cm^(-1/2)] to [mm^(-1/2)]
   fCoefL = fGas->GetCoefDiffusionLong() * sqrt(10.);  // [cm^(-1/2)] to [mm^(-1/2)]
   fGain = fGas->GetGain();

   return kSUCCESS;
}

void AtAvalancheTask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtAvalancheTask";

   /* if(!fElectronArray)
      fLogger->Fatal(MESSAGE_ORIGIN,"No DigitizedElectronArray!");
    fElectronArray -> Delete();*/

   Int_t nMCPoints = fMCPointArray->GetEntries();
   std::cout << " Number of MC Points " << nMCPoints << std::endl;
   if (nMCPoints < 10) {
      LOG(warning) << "Not enough hits for digitization! (<10)";
      return;
   }

   /**
    * NOTE! that fMCPoint has unit of [cm] for length scale,
    * [GeV] for energy and [ns] for time.
    */

   for (Int_t iPoint = 0; iPoint < nMCPoints; iPoint++) {
      fMCPoint = (AtMCPoint *)fMCPointArray->At(iPoint);
      Double_t eLoss = (fMCPoint->GetEnergyLoss()) * 1.E9; // [GeV] to [eV]

      //  std::cout<<" Base GetZ : "<<fMCPoint->FairMCPoint::GetZ()<<std::endl;
      // std::cout<<" Derived GetZIn : "<<fMCPoint->GetZIn()<<std::endl;

      Double_t lDrift = (fMCPoint->GetZ()) * 10; // drift length [mm]
      Double_t tDrift = lDrift / fVelDrift;      // drift time [ns]
      Double_t sigmaL = fCoefL * sqrt(lDrift);   // sigma in longitudinal direction
      Double_t sigmaT = fCoefT * sqrt(lDrift);   // sigma in transversal direction

      Int_t nElectrons = (Int_t)floor(fabs(eLoss / fEIonize)); // TODO: Include fluctuation

      for (Int_t iElectron = 0; iElectron < nElectrons; iElectron++) {
         Int_t gain = gRandom->Gaus(fGain, 50); // TODO : Gain function is neede.
         if (gain <= 0)
            continue;

         Double_t dr = gRandom->Gaus(0, sigmaT);             // displacement in radial direction
         Double_t angle = gRandom->Uniform(2 * TMath::Pi()); // random angle

         Double_t dx = dr * TMath::Cos(angle);               // displacement in x-direction
         Double_t dy = dr * TMath::Sin(angle);               // displacement in y-direction
         Double_t dt = gRandom->Gaus(0, sigmaL) / fVelDrift; // displacement in time

         /*

           Int_t index = fElectronArray->GetEntriesFast();
           STDriftedElectron *electron
             = new ((*fElectronArray)[index])
               STDriftedElectron(fMCPoint->GetX()*10, dx,
                                 fMCPoint->GetZ()*10, dz,
                                 fMCPoint->GetY()*10,
                                 fMCPoint->GetTime(), tDrift, dt,
                                 iWire, zWire,
                                 gain);

           electron -> SetIndex(index);*/
      }
   }

   // Int_t nDriftElectrons = fElectronArray->GetEntriesFast();

   /* fLogger->Info(MESSAGE_ORIGIN,
                  Form("Event #%d : MC points (%d) found. Drift electrons (%d) created.",
                       fEventID++, nMCPoints, nDriftElectrons));*/

   return;
}

ClassImp(AtAvalancheTask);
