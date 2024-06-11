/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#include "AtSeGADigitizer.h"

#include "AtMCPoint.h"            // for AtMCPoint
#include "AtSeGACrystalCalData.h" // for AtSeGACrystalCalData

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager
#include <FairRuntimeDb.h>   // for FairRuntimeDb
#include <FairTask.h>        // for FairTask, InitStatus, kFATAL, kSUC...

#include <TClonesArray.h> // for TClonesArray
#include <TObject.h>      // for TObject
#include <TRandom.h>      // for TRandom, gRandom

#include <algorithm> // for max
#include <cmath>     // for sqrt
#include <iostream>  // for cerr, cout, endl
#include <vector>    // for allocator, vector

using std::cerr;
using std::cout;
using std::endl;
ClassImp(AtSeGACrystalCalData);
AtSeGADigitizer::AtSeGADigitizer()
   : FairTask("ATTPC SEGA Digitizer"), fMCPointDataCA(nullptr), fSeGACryCalDataCA("AtSeGACrystalCalData", 5)
{
}

AtSeGADigitizer::~AtSeGADigitizer()
{
   LOG(info) << "AtSeGADigitizer: Delete instance";

   fSeGACryCalDataCA.Delete();
}

void AtSeGADigitizer::SetParContainers()
{

   FairRuntimeDb *rtdb = FairRuntimeDb::instance();
   if (!rtdb) {
      LOG(error) << "AtSeGADigitizer:: FairRuntimeDb not opened!";
   }
}

void AtSeGADigitizer::SetParameter() {}

InitStatus AtSeGADigitizer::Init()
{
   LOG(info) << "AtSeGADigitizer::Init ";

   FairRootManager *rootManager = FairRootManager::Instance();
   if (!rootManager)
      LOG(fatal) << "Init: No FairRootManager";

   fMCPointDataCA = dynamic_cast<TClonesArray *>(rootManager->GetObject("AtMCPoint")); // NOLINT
   if (!fMCPointDataCA) {
      LOG(fatal) << "Init: No AtMCPoint CA";
      return kFATAL;
   }

   rootManager->Register("SeGACrystalCalData", "CALIFA Crystal Cal", &fSeGACryCalDataCA, kTRUE);

   SetParameter();
   return kSUCCESS;
}

void AtSeGADigitizer::Exec(Option_t *option)
{
   // Reset entries in output arrays, local arrays
   Reset();

   // Reading the Input -- Point data --
   Int_t nHits = fMCPointDataCA->GetEntries();
   if (!nHits)
      return;

   std::vector<AtMCPoint *> pointData;
   for (Int_t i = 0; i < nHits; i++)
      pointData.push_back(dynamic_cast<AtMCPoint *>(fMCPointDataCA->At(i)));

   Int_t fDetCopyID;
   Double_t time;
   Double_t energy;

   for (Int_t i = 0; i < nHits; i++) {
      fDetCopyID = pointData[i]->GetDetCopyID();
      time = pointData[i]->GetTime();
      energy = pointData[i]->GetEnergyLoss();

      Int_t nCrystalCals = fSeGACryCalDataCA.GetEntriesFast();
      Bool_t existHit = false;
      if (nCrystalCals == 0)
         AddCrystalCal(fDetCopyID, NUSmearing(energy), time);
      else {
         for (Int_t j = 0; j < nCrystalCals; j++) {
            if ((dynamic_cast<AtSeGACrystalCalData *>(fSeGACryCalDataCA.At(j)))->GetDetCopyID() == fDetCopyID) {
               (dynamic_cast<AtSeGACrystalCalData *>(fSeGACryCalDataCA.At(j)))->AddMoreEnergy(NUSmearing(energy));
               if ((dynamic_cast<AtSeGACrystalCalData *>(fSeGACryCalDataCA.At(j)))->GetTime() > time) {
                  (dynamic_cast<AtSeGACrystalCalData *>(fSeGACryCalDataCA.At(j)))->SetTime(time);
               }
               existHit = true; // to avoid the creation of a new CrystalHit

               break;
            }
         }
         if (!existHit)
            AddCrystalCal(fDetCopyID, NUSmearing(energy), time);
      }
   }

   Int_t nCrystalCals = fSeGACryCalDataCA.GetEntriesFast();
   if (nCrystalCals == 0)
      return;

   Double_t temp = 0;
   Int_t tempCryID, parThres;
   Bool_t inUse;
   Int_t tempIndex;
   Float_t parReso;

   for (Int_t i = 0; i < nCrystalCals; i++) {

      tempCryID = (dynamic_cast<AtSeGACrystalCalData *>(fSeGACryCalDataCA.At(i)))->GetDetCopyID();

      temp = (dynamic_cast<AtSeGACrystalCalData *>(fSeGACryCalDataCA.At(i)))->GetEnergy();
      if (temp < fThreshold) {
         fSeGACryCalDataCA.RemoveAt(i);
         fSeGACryCalDataCA.Compress();
         nCrystalCals--; // remove from CalData those below threshold
         i--;
         continue;
      }

      if (isGe(tempCryID) && fResolutionGe > 0)
         (dynamic_cast<AtSeGACrystalCalData *>(fSeGACryCalDataCA.At(i)))->SetEnergy(ExpResSmearingGe(temp));
   }
}

// -----   Public method EndOfEvent   -----------------------------------------
void AtSeGADigitizer::EndOfEvent()
{
   // fMCPointDataCA->Clear();
   // fSeGACryCalDataCA.Clear();
   ResetParameters();
}

void AtSeGADigitizer::Register()
{
   FairRootManager::Instance()->Register("CrystalCal", GetName(), &fSeGACryCalDataCA, kTRUE);
}

void AtSeGADigitizer::Reset()
{
   // Clear the CA structure
   LOG(debug) << "Clearing SeGACrystalCalData Structure";
   fSeGACryCalDataCA.Clear();

   ResetParameters();
}

void AtSeGADigitizer::FinishEvent() {}

void AtSeGADigitizer::SetDetectionThreshold(Double_t thresholdEne)
{
   fThreshold = thresholdEne;
   LOG(info) << "AtSeGADigitizer::SetDetectionThreshold to " << fThreshold << " GeV.";
}

AtSeGACrystalCalData *AtSeGADigitizer::AddCrystalCal(Int_t ident, Double_t energy, ULong64_t time)
{
   TClonesArray &clref = fSeGACryCalDataCA;
   Int_t size = clref.GetEntriesFast();
   if (fVerbose > 1)
      LOG(info) << "-I- AtSeGADigitizer: Adding CrystalCalData "
                << " with unique identifier " << ident << " entering with " << energy * 1e06 << " keV "
                << " Time=" << time;

   return new (clref[size]) AtSeGACrystalCalData(ident, energy, time); // NOLINT
}

void AtSeGADigitizer::SetExpEnergyRes(Double_t crystalResGe)
{
   fResolutionGe = crystalResGe;

   LOG(info) << "AtSeGADigitizer::SetExpEnergyRes to " << fResolutionGe << "% @ 1 MeV for Ge";
}

Double_t AtSeGADigitizer::NUSmearing(Double_t inputEnergy)
{
   // Very simple preliminary scheme where the NU is introduced as a flat random
   // distribution with limits fNonUniformity (%) of the energy value.
   //
   return gRandom->Uniform(inputEnergy - inputEnergy * fNonUniformity / 100,
                           inputEnergy + inputEnergy * fNonUniformity / 100);
}

void AtSeGADigitizer::SetNonUniformity(Double_t nonU)
{
   fNonUniformity = nonU;
   LOG(info) << "AtSeGADigitizer::SetNonUniformity to " << fNonUniformity << " %";
}

Double_t AtSeGADigitizer::ExpResSmearingGe(Double_t inputEnergy)
{
   // Smears the energy according to some Experimental Resolution distribution
   // Very simple  scheme where the Experimental Resolution
   // is introduced as a gaus random distribution with a width given by the
   // parameter fResolution(in % @ MeV). Scales according to 1/sqrt(E)
   //
   // The formula is   TF1("name","0.058*x/sqrt(x)",0,10) for 3% at 1MeV (3.687 @
   // 662keV)
   //  ( % * energy ) / sqrt( energy )
   // and then the % is given at 1 MeV!!
   //
   if (fResolutionGe == 0)
      return inputEnergy;
   else {
      // Energy in MeV, that is the reason for the factor 1000...
      Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolutionGe * 1000 / (235 * sqrt(inputEnergy * 1000)));
      return inputEnergy + randomIs / 1000;
   }
}

Bool_t AtSeGADigitizer::isGe(Int_t id)
{
   if (id > 0 && id < 16)
      return kTRUE;
   else
      return kFALSE;
}
