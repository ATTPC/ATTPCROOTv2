 /********************************************************************************
  *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
  *                                                                              *
  *              This software is distributed under the terms of the             *
  *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
  *                  copied verbatim in the file "LICENSE"                       *
  ********************************************************************************/

#include "AtApolloDigitizer.h"
#include "FairRootManager.h"
#include "AtApollo.h"
#include "AtApolloCrystalCalData.h"
#include "AtApolloPoint.h"
#include "TArrayD.h"
#include "TClonesArray.h"
#include "TMath.h"
#include "TRandom.h"
#include "TVector3.h"
#include <iostream>
#include <stdlib.h>

using std::cerr;
using std::cout;
using std::endl;

AtApolloDigitizer::AtApolloDigitizer()
    : FairTask("ATTPC APOLLO Digitizer")
    , fApolloPointDataCA(NULL)
    , fApolloCryCalDataCA(NULL)
    , fNonUniformity(0)
{
    fNonUniformity = 0.; // perfect crystals
    fResolution = 0.;    // perfect crystals
    fThreshold = 0.;     // no threshold
}


AtApolloDigitizer::~AtApolloDigitizer()
{
    LOG(INFO) << "AtApolloDigitizer: Delete instance";

    if (fApolloPointDataCA)
    {
        fApolloPointDataCA->Delete();
        delete fApolloPointDataCA;
    }
    if (fApolloCryCalDataCA)
    {
        fApolloCryCalDataCA->Delete();
        delete fApolloCryCalDataCA;
    }
}

void AtApolloDigitizer::SetParContainers()
{

        FairRuntimeDb* rtdb = FairRuntimeDb::instance();
        if (!rtdb)
        {
            LOG(ERROR) << "AtApolloDigitizer:: FairRuntimeDb not opened!";
        }

}

void AtApolloDigitizer::SetParameter()
{

}

InitStatus AtApolloDigitizer::Init()
{
    LOG(INFO) << "AtApolloDigitizer::Init ";

    FairRootManager* rootManager = FairRootManager::Instance();
    if (!rootManager)
        LOG(fatal) << "Init: No FairRootManager";

    fApolloPointDataCA = (TClonesArray*)rootManager->GetObject("CrystalPoint");
    if (!fApolloPointDataCA)
    {
        LOG(fatal) << "Init: No CrystalPoint CA";
        return kFATAL;
    }

    fApolloCryCalDataCA = new TClonesArray("AtApolloCrystalCalData", 5);
    rootManager->Register("ApolloCrystalCalData", "CALIFA Crystal Cal", fApolloCryCalDataCA, kTRUE);

    SetParameter();
    return kSUCCESS;
}

void AtApolloDigitizer::Exec(Option_t* option)
{
    // Reset entries in output arrays, local arrays
    Reset();

    // Reading the Input -- Point data --
    Int_t nHits = fApolloPointDataCA->GetEntries();
    if (!nHits)
        return;

    AtApolloPoint** pointData = NULL;
    pointData = new AtApolloPoint*[nHits];
    for (Int_t i = 0; i < nHits; i++)
        pointData[i] = (AtApolloPoint*)(fApolloPointDataCA->At(i));

    Int_t crystalId;
    Double_t time;
    Double_t energy;

    for (Int_t i = 0; i < nHits; i++)
    {
        crystalId = pointData[i]->GetCrystalID();
        time = pointData[i]->GetTime();
        energy = pointData[i]->GetEnergyLoss();

        Int_t nCrystalCals = fApolloCryCalDataCA->GetEntriesFast();
        Bool_t existHit = 0;
        if (nCrystalCals == 0)
            AddCrystalCal(crystalId, NUSmearing(energy), time);
        else
        {
            for (Int_t j = 0; j < nCrystalCals; j++)
            {
                if (((AtApolloCrystalCalData*)(fApolloCryCalDataCA->At(j)))->GetCrystalId() == crystalId)
                {
                    ((AtApolloCrystalCalData*)(fApolloCryCalDataCA->At(j)))->AddMoreEnergy(NUSmearing(energy));
                    if (((AtApolloCrystalCalData*)(fApolloCryCalDataCA->At(j)))->GetTime() > time)
                    {
                        ((AtApolloCrystalCalData*)(fApolloCryCalDataCA->At(j)))->SetTime(time);
                    }
                    existHit = 1; // to avoid the creation of a new CrystalHit

                    break;
                }
            }
            if (!existHit)
                AddCrystalCal(crystalId, NUSmearing(energy), time);
        }
        existHit = 0;
    }

    if (pointData)
        delete[] pointData;

    Int_t nCrystalCals = fApolloCryCalDataCA->GetEntriesFast();
    if (nCrystalCals == 0)
        return;

    Double_t temp = 0;
    Int_t tempCryID, parThres;
    Bool_t inUse;
    Int_t tempIndex;
    Float_t parReso;

    for (Int_t i = 0; i < nCrystalCals; i++)
    {

        tempCryID = ((AtApolloCrystalCalData*)(fApolloCryCalDataCA->At(i)))->GetCrystalId();

        temp = ((AtApolloCrystalCalData*)(fApolloCryCalDataCA->At(i)))->GetEnergy();
        if (temp < fThreshold)
        {
            fApolloCryCalDataCA->RemoveAt(i);
            fApolloCryCalDataCA->Compress();
            nCrystalCals--; // remove from CalData those below threshold
            i--;
            continue;
        }

        if (fResolution > 0)
            ((AtApolloCrystalCalData*)(fApolloCryCalDataCA->At(i)))->SetEnergy(ExpResSmearing(temp));
    }
}

// -----   Public method EndOfEvent   -----------------------------------------
void AtApolloDigitizer::EndOfEvent()
{
    // fApolloPointDataCA->Clear();
    // fApolloCryCalDataCA->Clear();
    ResetParameters();
}

void AtApolloDigitizer::Register()
{
    FairRootManager::Instance()->Register("CrystalCal", GetName(), fApolloCryCalDataCA, kTRUE);
}

void AtApolloDigitizer::Reset()
{
    // Clear the CA structure
    LOG(DEBUG) << "Clearing ApolloCrystalCalData Structure";
    if (fApolloCryCalDataCA)
        fApolloCryCalDataCA->Clear();

    ResetParameters();
}

void AtApolloDigitizer::FinishEvent() {}

void AtApolloDigitizer::SetDetectionThreshold(Double_t thresholdEne)
{
    fThreshold = thresholdEne;
    LOG(INFO) << "AtApolloDigitizer::SetDetectionThreshold to " << fThreshold << " GeV.";
}

AtApolloCrystalCalData* AtApolloDigitizer::AddCrystalCal(Int_t ident,
                                                           Double_t energy,
                                                           ULong64_t time)
{
    TClonesArray& clref = *fApolloCryCalDataCA;
    Int_t size = clref.GetEntriesFast();
    if (fVerbose > 1)
        LOG(INFO) << "-I- AtApolloDigitizer: Adding CrystalCalData "
                  << " with unique identifier " << ident << " entering with " << energy * 1e06 << " keV "<< " Time=" << time;

    return new (clref[size]) AtApolloCrystalCalData(ident, energy, time);
}

void AtApolloDigitizer::SetExpEnergyRes(Double_t crystalRes)
{
    fResolution = crystalRes;
    LOG(INFO) << "AtApolloDigitizer::SetExpEnergyRes to " << fResolution << "% @ 1 MeV.";
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
    LOG(INFO) << "AtApolloDigitizer::SetNonUniformity to " << fNonUniformity << " %";
}

Double_t AtApolloDigitizer::ExpResSmearing(Double_t inputEnergy)
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
    if (fResolution == 0)
        return inputEnergy;
    else
    {
        // Energy in MeV, that is the reason for the factor 1000...
        Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolution * 1000 / (235 * sqrt(inputEnergy * 1000)));
        return inputEnergy + randomIs / 1000;
    }
}
