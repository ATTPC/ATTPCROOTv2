// -------------------------------------------------------------------------
// -----            Based on FairIonGenerator source file              -----
// -----            Created 30/01/15  by Y. Ayyad                      -----
// -------------------------------------------------------------------------
#include "AtTPCIonGenerator.h"

#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairParticle.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TDatabasePDG.h>
#include <TMath.h>
#include <TObjArray.h>
#include <TObject.h> // for TObject
#include <TParticle.h>
#include <TParticlePDG.h>
#include <TRandom.h>
#include <TString.h>

#include <cmath>
#include <iostream>
#include <limits> // for numeric_limits

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

using std::cout;
using std::endl;

// -----   Initialsisation of static variables   --------------------------
Int_t AtTPCIonGenerator::fgNIon = 0;
// ------------------------------------------------------------------------

// -----   Default constructor   ------------------------------------------
AtTPCIonGenerator::AtTPCIonGenerator()
   : fMult(0), fPx(0.), fPy(0.), fPz(0.), fR(0.), fz(0.), fOffsetX(0.), fOffsetY(0.), fVx(0.), fVy(0.), fVz(0.),
     fIon(nullptr), fQ(0)

{
   //  cout << "-W- AtTPCIonGenerator: "
   //      << " Please do not use the default constructor! " << endl;
}
// ------------------------------------------------------------------------

AtTPCIonGenerator::AtTPCIonGenerator(const Char_t *ionName, Int_t mult, Double_t px, Double_t py, Double_t pz)
   : fMult(0), fPx(0.), fPy(0.), fPz(0.), fR(0.), fz(0.), fOffsetX(0.), fOffsetY(0.), fVx(0.), fVy(0.), fVz(0.),
     fIon(nullptr), fQ(0)
{

   FairRunSim *fRun = FairRunSim::Instance();
   TObjArray *UserIons = fRun->GetUserDefIons();
   TObjArray *UserParticles = fRun->GetUserDefParticles();
   FairParticle *part = nullptr;
   fIon = dynamic_cast<FairIon *>(UserIons->FindObject(ionName)); // NOLINT

   if (fIon) {
      fgNIon++;
      fMult = mult;
      fPx = Double_t(fIon->GetA()) * px;
      fPy = Double_t(fIon->GetA()) * py;
      fPz = Double_t(fIon->GetA()) * pz;
      // fVx   = vx;
      // fVy   = vy;
      // fVz   = vz;
      // }

   } else {
      part = dynamic_cast<FairParticle *>(UserParticles->FindObject(ionName));
      if (part) {
         fgNIon++;
         TParticle *particle = part->GetParticle();
         fMult = mult;
         fPx = Double_t(particle->GetMass() / 0.92827231) * px;
         fPy = Double_t(particle->GetMass() / 0.92827231) * py;
         fPz = Double_t(particle->GetMass() / 0.92827231) * pz;
         // fVx   = vx;
         // fVy   = vy;
         // fVz   = vz;
      }
   }
   if (fIon == nullptr && part == nullptr) {
      cout << "-E- AtTPCIonGenerator: Ion or Particle is not defined !" << endl;
      Fatal("AtTPCIonGenerator", "No FairRun instantised!");
   }
}

AtTPCIonGenerator::AtTPCIonGenerator(const char *name, Int_t z, Int_t a, Int_t q, Int_t mult, Double_t px, Double_t py,
                                     Double_t pz, Double_t Ex, Double_t m, Double_t ener, Double_t eLoss)
   : fMult(mult), fPx(Double_t(a) * px), fPy(Double_t(a) * py), fPz(Double_t(a) * pz), fR(0.), fz(0.), fOffsetX(0.),
     fOffsetY(0.), fVx(0.), fVy(0.), fVz(0.), fIon(nullptr), fQ(0), fNomEner(ener), fMaxEnLoss(eLoss < 0 ? ener : eLoss)
{
   fgNIon++;

   fIon = new FairIon(TString::Format("FairIon%d", fgNIon).Data(), z, a, q, Ex, m); // NOLINT
   cout << " Beam Ion mass : " << fIon->GetMass() << endl;
   AtVertexPropagator::Instance()->SetBeamMass(fIon->GetMass());
   AtVertexPropagator::Instance()->SetBeamNomE(ener);
   FairRunSim *run = FairRunSim::Instance();
   if (!run) {
      cout << "-E- FairIonGenerator: No FairRun instantised!" << endl;
      Fatal("FairIonGenerator", "No FairRun instantised!");
      return;
   }
   run->AddNewIon(fIon);
}

void AtTPCIonGenerator::SetExcitationEnergy(Double_t eExc)
{
   fIon->SetExcEnergy(eExc);
}
void AtTPCIonGenerator::SetMass(Double_t mass)
{
   fIon->SetMass(mass);
}
void AtTPCIonGenerator::SetSpotRadius(Double32_t r, Double32_t z, Double32_t offx, Double_t offy)
{
   fR = r;
   fz = z;
   fOffsetX = offx;
   fOffsetY = offy;
}

void AtTPCIonGenerator::SetVertexCoordinates()
{
   auto Phi = gRandom->Uniform(0, 360) * TMath::DegToRad();
   auto SpotR = gRandom->Uniform(0, fR);

   fVx = fOffsetX + SpotR * cos(Phi); // gRandom->Uniform(-fx,fx);
   fVy = fOffsetY + SpotR * sin(Phi); // gRandom->Uniform(-fy,fy);
   fVz = fz;
}

Bool_t AtTPCIonGenerator::ReadEvent(FairPrimaryGenerator *primGen)
{

   // if ( ! fIon ) {
   //   cout << "-W- FairIonGenerator: No ion defined! " << endl;
   //   return kFALSE;
   // }

   TParticlePDG *thisPart = TDatabasePDG::Instance()->GetParticle(fIon->GetName());
   if (!thisPart) {
      cout << "-W- FairIonGenerator: Ion " << fIon->GetName() << " not found in database!" << endl;
      return kFALSE;
   }

   int pdgType = thisPart->PdgCode();
   SetVertexCoordinates();

   AtVertexPropagator::Instance()->IncBeamEvtCnt();

   if (AtVertexPropagator::Instance()->GetBeamEvtCnt() % 2 != 0) {
      if (fDoReact) {
         Double_t Er = gRandom->Uniform(0., fMaxEnLoss);
         AtVertexPropagator::Instance()->SetRndELoss(Er);
         // std::cout << cGREEN << " Random Energy AtTPCIonGenerator : " << Er << cNORMAL << std::endl;
      } else
         AtVertexPropagator::Instance()->SetRndELoss(std::numeric_limits<double>::max());
   }

   // We only want to add a beam track if it is a beam event or it is a reaction event and we are not doing a reaction
   if (AtVertexPropagator::Instance()->GetBeamEvtCnt() % 2 != 0 ||
       (AtVertexPropagator::Instance()->GetBeamEvtCnt() % 2 == 0 && !fDoReact))
      for (Int_t i = 0; i < fMult; i++)
         primGen->AddTrack(pdgType, fPx, fPy, fPz, fVx, fVy, fVz);

   return kTRUE;
}

//_____________________________________________________________________________

ClassImp(AtTPCIonGenerator)
