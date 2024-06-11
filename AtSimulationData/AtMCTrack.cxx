/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                   AtMCTrack source file                   -----
// -----                  M. Al-Turany   June 2014                     -----
// ----- This is a light weight particle class that is saved to disk
// ----- instead of saveing the TParticle class
// ----- IT is also used for filtring the stack
// -------------------------------------------------------------------------

#include "AtMCTrack.h"

#include <FairLogger.h>

#include <Rtypes.h>
#include <TDatabasePDG.h> // for TDatabasePDG
#include <TMath.h>
#include <TObject.h>
#include <TParticle.h>    // for TParticle
#include <TParticlePDG.h> // for TParticlePDG

#include <memory>

// -----   Default constructor   -------------------------------------------
AtMCTrack::AtMCTrack()
   : TObject(), fPdgCode(0), fMotherId(-1), fPx(0.), fPy(0.), fPz(0.), fStartX(0.), fStartY(0.), fStartZ(0.),
     fStartT(0.), fNPoints(0)
{
}
// -------------------------------------------------------------------------

// -----   Standard constructor   ------------------------------------------
AtMCTrack::AtMCTrack(Int_t pdgCode, Int_t motherId, Double_t px, Double_t py, Double_t pz, Double_t x, Double_t y,
                     Double_t z, Double_t t, Int_t nPoints = 0)
   : TObject(), fPdgCode(pdgCode), fMotherId(motherId), fPx(px), fPy(py), fPz(pz), fStartX(x), fStartY(y), fStartZ(z),
     fStartT(t), fNPoints(nPoints)
{
}
// -------------------------------------------------------------------------

// -----   Copy constructor   ----------------------------------------------
AtMCTrack::AtMCTrack(const AtMCTrack &track)

   = default;
// -------------------------------------------------------------------------

// -----   Constructor from TParticle   ------------------------------------
AtMCTrack::AtMCTrack(TParticle *part)
   : TObject(), fPdgCode(part->GetPdgCode()), fMotherId(part->GetMother(0)), fPx(part->Px()), fPy(part->Py()),
     fPz(part->Pz()), fStartX(part->Vx()), fStartY(part->Vy()), fStartZ(part->Vz()), fStartT(part->T() * 1e09),
     fNPoints(0)
{
}
// -------------------------------------------------------------------------

// -----   Destructor   ----------------------------------------------------
AtMCTrack::~AtMCTrack() = default;
// -------------------------------------------------------------------------

// -----   Public method Print   -------------------------------------------
void AtMCTrack::Print(Int_t trackId) const
{
   LOG(debug) << "Track " << trackId << ", mother : " << fMotherId << ", Type " << fPdgCode << ", momentum (" << fPx
              << ", " << fPy << ", " << fPz << ") GeV";
   /* LOG(DEBUG2) << "       Ref " << GetNPoints(kREF)
                << ", TutDet " << GetNPoints(kTutDet)
                << ", Rutherford " << GetNPoints(kFairRutherford)
               ;
  */
}
// -------------------------------------------------------------------------

// -----   Public method GetMass   -----------------------------------------
Double_t AtMCTrack::GetMass() const
{
   if (TDatabasePDG::Instance()) {
      TParticlePDG *particle = TDatabasePDG::Instance()->GetParticle(fPdgCode);
      if (particle) {
         return particle->Mass();
      } else {
         return 0.;
      }
   }
   return 0.;
}
// -------------------------------------------------------------------------

// -----   Public method GetRapidity   -------------------------------------
Double_t AtMCTrack::GetRapidity() const
{
   Double_t e = GetEnergy();
   Double_t y = 0.5 * TMath::Log((e + fPz) / (e - fPz));
   return y;
}
// -------------------------------------------------------------------------

ClassImp(AtMCTrack)
