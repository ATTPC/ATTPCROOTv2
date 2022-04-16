/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#ifndef ATAPOLLODIGITIZER_H
#define ATAPOLLODIGITIZER_H

#include <Rtypes.h>
// Needed for streamer generation
#include <FairTask.h>

#include <TClonesArray.h> // IWYU pragma: keep

class AtApolloCrystalCalData;
class TBuffer;
class TClass;
class TMemberInspector;

class AtApolloDigitizer : public FairTask {

public:
   /** Default constructor **/
   AtApolloDigitizer();

   /** Destructor **/
   ~AtApolloDigitizer();

   /** Virtual method Init **/
   virtual InitStatus Init();

   /** Virtual method Exec **/
   virtual void Exec(Option_t *opt);

   /** Virtual method EndOffEvent **/
   virtual void EndOfEvent();

   /** Virtual method Register **/
   virtual void Register();

   /** Virtual method Reset **/
   virtual void Reset();

   /** Virtual method FinishEvent **/
   virtual void FinishEvent();

   virtual void SetParContainers();

   /** Public method SetRealConfig
    **
    ** Defines the REAL experimental resolution and Thresholds of the CsI(Tl)
    *Crystals
    *@param isRealSet  Bool parameter used to set the experimental Resolution and
    *Thresholds
    **/
   void SetRealConfig(Bool_t isRealSet);

   /** Public method SetExpEnergyRes
    **
    ** Sets the experimental energy resolution of the CsI and LaBr crystals
    **/
   void SetExpEnergyRes(Double_t crystalResCsI, Double_t crystalResLaBr);

   /** Public method SetDetectionThreshold
    **
    ** Defines the minimum energy requested in a crystal to be included as a
    *CrystalCal
    *@param thresholdEne  Double parameter used to set the threshold (in GeV)
    **/
   void SetDetectionThreshold(Double_t thresholdEne);

   /** Public method SetNonUniformity
    **
    ** Defines the fNonUniformity parameter in % deviation from the central value
    *@param nonU  Double parameter setting the maximum non-uniformity allowed
    **/
   void SetNonUniformity(Double_t nonU);

   inline void ResetParameters(){};

   /** Private method AddCrystalCal
    **
    ** Adds a ApolloCrystalCal data
    **/
   AtApolloCrystalCalData *AddCrystalCal(Int_t ident, Double_t energy, ULong64_t time);

private:
   void SetParameter();

   TClonesArray *fApolloPointDataCA; //!  The crystal hit collection
   TClonesArray fApolloCryCalDataCA; /**< Array with CALIFA Cal- output data. >*/

   Double_t fNonUniformity{0.};  // Experimental non-uniformity parameter
   Double_t fResolutionCsI{0.};  // Experimental resolution @ 1 MeV for CsI
   Double_t fResolutionLaBr{0.}; // Experimental resolution @ 1 MeV for LaBr
   Double_t fThreshold{0.};      // Minimum energy requested to create a Cal

   /** Private method NUSmearing
    **
    ** Smears the energy according to some non-uniformity distribution
    ** Very simple scheme where the NU is introduced as a flat random
    ** distribution with limits fNonUniformity (%) of the energy value.
    **/
   Double_t NUSmearing(Double_t inputEnergy);

   /** Private method ExpResSmearingCsI
    **
    ** Smears the energy according to some Experimental Resolution distribution
    ** Very simple scheme where the Experimental Resolution
    ** is introduced as a gaus random distribution with a width given by the
    ** parameter fResolutionCsI (LaBr) (in % @ MeV). Scales according to 1/sqrt(E)
    **/
   Double_t ExpResSmearingCsI(Double_t inputEnergy);

   /** Private method ExpResSmearingLaBr
    **/
   Double_t ExpResSmearingLaBr(Double_t inputEnergy);

   Bool_t isCsI(Int_t id);
   Bool_t isLaBr(Int_t id);

   ClassDef(AtApolloDigitizer, 1);
};

#endif
