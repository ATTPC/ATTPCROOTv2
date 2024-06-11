/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#include "AtApolloDigitizer.h"

#include "AtApolloCrystalCalData.h"
#include "AtApolloPoint.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <TClonesArray.h>
#include <TObject.h>
#include <TRandom.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using std::cerr;
using std::cout;
using std::endl;

AtApolloDigitizer::AtApolloDigitizer()
   : FairTask("ATTPC APOLLO Digitizer"), fApolloPointDataCA(nullptr), fApolloCryCalDataCA("AtApolloCrystalCalData", 5)
{
}

AtApolloDigitizer::~AtApolloDigitizer()
{
   LOG(info) << "AtApolloDigitizer: Delete instance";

   if (fApolloPointDataCA) {
      fApolloPointDataCA->Delete();
      delete fApolloPointDataCA;
   }
   fApolloCryCalDataCA.Delete();
}

void AtApolloDigitizer::SetParContainers()
{

   FairRuntimeDb *rtdb = FairRuntimeDb::instance();
   if (!rtdb) {
      LOG(error) << "AtApolloDigitizer:: FairRuntimeDb not opened!";
   }
}

void AtApolloDigitizer::SetParameter() {}

InitStatus AtApolloDigitizer::Init()
{
   LOG(info) << "AtApolloDigitizer::Init ";

   FairRootManager *rootManager = FairRootManager::Instance();
   if (!rootManager) {
      LOG(fatal) << "Init: No FairRootManager";
      return kFATAL;
   }

   fApolloPointDataCA = dynamic_cast<TClonesArray *>(rootManager->GetObject("AtApolloPoint"));
   if (!fApolloPointDataCA) {
      LOG(fatal) << "Init: No AtApolloPoint CA";
      return kFATAL;
   }

   rootManager->Register("ApolloCrystalCalData", "CALIFA Crystal Cal", &fApolloCryCalDataCA, kTRUE);

   SetParameter();
   return kSUCCESS;
}

void AtApolloDigitizer::Exec(Option_t *option)
{
   // Reset entries in output arrays, local arrays
   Reset();

   // Reading the Input -- Point data --
   Int_t nHits = fApolloPointDataCA->GetEntries();
   if (!nHits)
      return;

   std::vector<AtApolloPoint *> pointData;
   // AtApolloPoint **pointData = nullptr;
   // pointData = new AtApolloPoint *[nHits];
   for (Int_t i = 0; i < nHits; i++)
      pointData.push_back(dynamic_cast<AtApolloPoint *>(fApolloPointDataCA->At(i)));

   Int_t crystalId;
   Double_t time;
   Double_t energy;

   for (Int_t i = 0; i < nHits; i++) {
      crystalId = pointData[i]->GetCrystalID();
      time = pointData[i]->GetTime();
      energy = pointData[i]->GetEnergyLoss();

      Int_t nCrystalCals = fApolloCryCalDataCA.GetEntriesFast();
      Bool_t existHit = false;
      if (nCrystalCals == 0)
         AddCrystalCal(crystalId, NUSmearing(energy), time);
      else {
         for (Int_t j = 0; j < nCrystalCals; j++) {
            if ((dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(j)))->GetCrystalId() == crystalId) {
               (dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(j)))->AddMoreEnergy(NUSmearing(energy));
               if ((dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(j)))->GetTime() > time) {
                  (dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(j)))->SetTime(time);
               }
               existHit = true; // to avoid the creation of a new CrystalHit

               break;
            }
         }
         if (!existHit)
            AddCrystalCal(crystalId, NUSmearing(energy), time);
      }
   }

   Int_t nCrystalCals = fApolloCryCalDataCA.GetEntriesFast();
   if (nCrystalCals == 0)
      return;

   Double_t temp = 0;
   Int_t tempCryID, parThres;
   Bool_t inUse;
   Int_t tempIndex;
   Float_t parReso;

   for (Int_t i = 0; i < nCrystalCals; i++) {

      tempCryID = (dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(i)))->GetCrystalId();

      temp = (dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(i)))->GetEnergy();
      if (temp < fThreshold) {
         fApolloCryCalDataCA.RemoveAt(i);
         fApolloCryCalDataCA.Compress();
         nCrystalCals--; // remove from CalData those below threshold
         i--;
         continue;
      }

      if (isCsI(tempCryID) && fResolutionCsI > 0)
         (dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(i)))->SetEnergy(ExpResSmearingCsI(temp));
      if (isLaBr(tempCryID) && fResolutionLaBr > 0)
         (dynamic_cast<AtApolloCrystalCalData *>(fApolloCryCalDataCA.At(i)))->SetEnergy(ExpResSmearingLaBr(temp));
   }
}

// -----   Public method EndOfEvent   -----------------------------------------
void AtApolloDigitizer::EndOfEvent()
{
   // fApolloPointDataCA->Clear();
   // fApolloCryCalDataCA.Clear();
   ResetParameters();
}

void AtApolloDigitizer::Register()
{
   FairRootManager::Instance()->Register("CrystalCal", GetName(), &fApolloCryCalDataCA, kTRUE);
}

void AtApolloDigitizer::Reset()
{
   // Clear the CA structure
   LOG(debug) << "Clearing ApolloCrystalCalData Structure";
   fApolloCryCalDataCA.Clear();

   ResetParameters();
}

void AtApolloDigitizer::FinishEvent() {}

void AtApolloDigitizer::SetDetectionThreshold(Double_t thresholdEne)
{
   fThreshold = thresholdEne;
   LOG(info) << "AtApolloDigitizer::SetDetectionThreshold to " << fThreshold << " GeV.";
}

AtApolloCrystalCalData *AtApolloDigitizer::AddCrystalCal(Int_t ident, Double_t energy, ULong64_t time)
{
   TClonesArray &clref = fApolloCryCalDataCA;
   Int_t size = clref.GetEntriesFast();
   if (fVerbose > 1)
      LOG(info) << "-I- AtApolloDigitizer: Adding CrystalCalData "
                << " with unique identifier " << ident << " entering with " << energy * 1e06 << " keV "
                << " Time=" << time;

   return new (clref[size]) AtApolloCrystalCalData(ident, energy, time); // NOLINT
}

void AtApolloDigitizer::SetExpEnergyRes(Double_t crystalResCsI, Double_t crystalResLaBr)
{
   fResolutionCsI = crystalResCsI;
   fResolutionLaBr = crystalResLaBr;
   LOG(info) << "AtApolloDigitizer::SetExpEnergyRes to " << fResolutionCsI << "% @ 1 MeV for CsI and "
             << fResolutionLaBr << "% @ 1 MeV for LaBr.";
}

Double_t AtApolloDigitizer::NUSmearing(Double_t inputEnergy)
{
   // Very simple preliminary scheme where the NU is introduced as a flat random
   // distribution with limits fNonUniformity (%) of the energy value.
   //
   return gRandom->Uniform(inputEnergy - inputEnergy * fNonUniformity / 100,
                           inputEnergy + inputEnergy * fNonUniformity / 100);
}

void AtApolloDigitizer::SetNonUniformity(Double_t nonU)
{
   fNonUniformity = nonU;
   LOG(info) << "AtApolloDigitizer::SetNonUniformity to " << fNonUniformity << " %";
}

Double_t AtApolloDigitizer::ExpResSmearingCsI(Double_t inputEnergy)
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
   if (fResolutionCsI == 0)
      return inputEnergy;
   else {
      // Energy in MeV, that is the reason for the factor 1000...
      Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolutionCsI * 1000 / (235 * sqrt(inputEnergy * 1000)));
      return inputEnergy + randomIs / 1000;
   }
}
Double_t AtApolloDigitizer::ExpResSmearingLaBr(Double_t inputEnergy)
{
   if (fResolutionLaBr == 0)
      return inputEnergy;
   else {
      // Energy in MeV, that is the reason for the factor 1000...
      Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolutionLaBr * 1000 / (235 * sqrt(inputEnergy * 1000)));
      return inputEnergy + randomIs / 1000;
   }
}

Bool_t AtApolloDigitizer::isCsI(Int_t id)
{
   if (id > 6 && id < 27)
      return kTRUE;
   else
      return kFALSE;
}

Bool_t AtApolloDigitizer::isLaBr(Int_t id)
{
   if (id > 0 && id < 7)
      return kTRUE;
   else
      return kFALSE;
}

ClassImp(AtApolloDigitizer);
