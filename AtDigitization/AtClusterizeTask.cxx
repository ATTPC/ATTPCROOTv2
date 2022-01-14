#include "AtClusterizeTask.h"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "AtDigiPar.h"
#include "AtGas.h"
#include "AtSimulatedElectron.h"
#include "AtTpcPoint.h"
#include "AtVertexPropagator.h"

// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TClonesArray.h"
#include "TF1.h"
#include "TMath.h"
#include "TRandom.h"

AtClusterizeTask::AtClusterizeTask() : FairTask("AtClusterizeTask"), fEventID(0), fIsPersistent(kFALSE) {}

AtClusterizeTask::~AtClusterizeTask()
{
   LOG(debug) << "Destructor of AtClusterizeTask";
}

void AtClusterizeTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtClusterizeTask";

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));
   if (fPar == nullptr)
      LOG(fatal) << "Could not get the parameter container "
                 << "AtDigiPar";
}

InitStatus AtClusterizeTask::Init()
{
   LOG(debug) << "Initilization of AtClusterizeTask";

   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr) {
      LOG(error) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   fElectronNumberArray = new TClonesArray("AtSimulatedElectron");
   ioman->Register("AtSimulatedPoint", "cbmsim", fElectronNumberArray, fIsPersistent);

   fEIonize = fPar->GetEIonize() / 1000000; // [MeV]
   fFano = fPar->GetFano();
   fVelDrift = fPar->GetDriftVelocity();   // [cm/us]
   fCoefT = fPar->GetCoefDiffusionTrans(); // [cm^2/us]
   fCoefL = fPar->GetCoefDiffusionLong();  // [cm^2/us]
   // TODO: replace with math using drift velcoity, TB entrance, sampling rate, and drift length
   fDetPadPlane = fPar->GetZPadPlane(); //[mm]

   std::cout << "  Ionization energy of gas: " << fEIonize << " MeV" << std::endl;
   std::cout << "  Fano factor of gas: " << fFano << std::endl;
   std::cout << "  Drift velocity: " << fVelDrift << std::endl;
   std::cout << "  Longitudal coefficient of diffusion: " << fCoefT << std::endl;
   std::cout << "  Transverse coefficient of diffusion: " << fCoefL << std::endl;
   std::cout << "  Position of the pad plane (Z): " << fDetPadPlane << std::endl;

   return kSUCCESS;
}

void AtClusterizeTask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtClusterizeTask";
   Int_t nMCPoints = fMCPointArray->GetEntries();

   /**
    * NOTE! that fMCPoint has unit of [cm] for length scale,
    * [GeV] for energy and [ns] for time.
    */
   fElectronNumberArray->Delete();

   Double_t energyLoss_rec = 0.0;
   Double_t x = 0;
   Double_t y = 0;
   Double_t z = 0;
   Int_t nElectrons = 0;
   Int_t eFlux = 0;
   Int_t genElectrons = 0;
   TString VolName;
   Double_t tTime; //, entries;

   Double_t driftLength;
   Double_t driftTime;
   Double_t propX;
   Double_t propY;

   Double_t sigstrtrans, sigstrlong, phi, r;
   Int_t electronNumber = 0;

   Double_t x_post = 0;
   Double_t y_post = 0;
   Double_t z_post = 0;
   Double_t x_pre = 0;
   Double_t y_pre = 0;
   Double_t z_pre = 0;
   Double_t stepX, stepY, stepZ;

   Int_t presentTrackID = -10; // control of the point trackID
   Bool_t readyToProject = kFALSE;

   for (Int_t i = 0; i < nMCPoints; i++) {

      fMCPoint = (AtTpcPoint *)fMCPointArray->At(i);

      VolName = fMCPoint->GetVolName();
      Int_t trackID = fMCPoint->GetTrackID();

      if (VolName != "drift_volume")
         continue;

      // Assume energy is deposited between present Point position to previous Point position

      // a new track is entering the volume or no energy was deposited
      if (fMCPoint->GetEnergyLoss() == 0 || presentTrackID != fMCPoint->GetTrackID()) {
         x_pre = fMCPoint->GetXIn() * 10;                  // mm
         y_pre = fMCPoint->GetYIn() * 10;                  // mm
         z_pre = fDetPadPlane - (fMCPoint->GetZIn() * 10); // mm
         presentTrackID = fMCPoint->GetTrackID();
         if (presentTrackID != fMCPoint->GetTrackID())
            LOG(info) << "Note in NEW DIGI: Energy not zero in first point for a track!";
         continue;
      }

      tTime = fMCPoint->GetTime() / 1000;                  // us
      x = fMCPoint->GetXIn() * 10;                         // mm
      y = fMCPoint->GetYIn() * 10;                         // mm
      z = fDetPadPlane - (fMCPoint->GetZIn() * 10);        // mm
      energyLoss_rec = (fMCPoint->GetEnergyLoss()) * 1000; // MeV

      nElectrons = energyLoss_rec / fEIonize;          // mean electrons generated
      eFlux = pow(fFano * nElectrons, 0.5);            // fluctuation of generated electrons
      genElectrons = gRandom->Gaus(nElectrons, eFlux); // generated electrons
      // std::cout<<" nElectrons "<<nElectrons<<" Gen Electrons "<<genElectrons<<"\n";

      // step in each direction for an homogeneous electron creation position along the track
      // stepX = (x_post-x) / genElectrons;
      // stepY = (y_post-y) / genElectrons;
      // stepZ = (z_post-z) / genElectrons;
      if (genElectrons > 0) {
         stepX = (x - x_pre) / genElectrons;
         stepY = (y - y_pre) / genElectrons;
         stepZ = (z - z_pre) / genElectrons;
      }

      driftLength = abs(z);                                            // mm coarse value of the driftLength
      sigstrtrans = sqrt(10.0 * fCoefT * 2 * driftLength / fVelDrift); // transverse diffusion coefficient in mm
      sigstrlong = sqrt(10.0 * fCoefL * 2 * driftLength / fVelDrift);  // longitudal diffusion coefficient in mm

      for (Int_t charge = 0; charge < genElectrons; charge++) { // for every electron in the cluster
         // r             = trans->GetRandom(); //non-Gaussian cloud
         driftLength = abs(z_pre + stepZ * charge); // fine value of the driftLength
         r = gRandom->Gaus(0, sigstrtrans);         // Gaussian cloud
         phi = gRandom->Uniform(0, TMath::TwoPi());
         propX = x_pre + stepX * charge + r * TMath::Cos(phi);
         propY = y_pre + stepY * charge + r * TMath::Sin(phi);
         driftLength = driftLength + (gRandom->Gaus(0, sigstrlong)); // mm
         driftTime = ((driftLength / 10) / fVelDrift);               // us
         // NB: tTibme in the simulation is 0 for the first simulation point
         // std::cout<<i<<"  "<<charge<<"  "<<" Drift velocity "<<fVelDrift
         // <<" driftTime : "<<driftTime<<" tTime "<<tTime<<"\n";
         // std::cout<<" Position of electron "<<charge<<" : "<<propX<<" : "<<propY<<"\n";
         electronNumber += 1;

         // Fill container AtSimulatedPoint
         Int_t size = fElectronNumberArray->GetEntriesFast();
         AtSimulatedElectron *simpoint =
            new ((*fElectronNumberArray)[size]) AtSimulatedElectron(i, electronNumber, // electron #
                                                                    propX,             // X
                                                                    propY,             // Y
                                                                    driftTime);        // Z
         simpoint->SetMCEventID(fMCPoint->GetEventID());
      } // end producing e- and filling AtSimpoint

      x_pre = x;
      y_pre = y;
      z_pre = z;

   } // end through all interaction points

   return;
}

ClassImp(AtClusterizeTask);
