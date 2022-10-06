/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#ifndef ATSEGADIGITIZER_H
#define ATSEGADIGITIZER_H

#include <FairTask.h>

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, Bool_t, Int_t
#include <TClonesArray.h>

class AtSeGACrystalCalData;
class TBuffer;
class TClass;
class TMemberInspector;

class AtSeGADigitizer : public FairTask {
private:
   TClonesArray *fMCPointDataCA;   //!  The crystal hit collection (we do not own)
   TClonesArray fSeGACryCalDataCA; /**< Array with CALIFA Cal- output data. >*/

   Double_t fNonUniformity{0}; // Experimental non-uniformity parameter
   Double_t fResolutionGe{0};  // Experimental resolution @ 1 MeV for Ge
   Double_t fThreshold{0};     // Minimum energy requested to create a Cal

public:
   /** Default constructor **/
   AtSeGADigitizer();

   /** Destructor **/
   ~AtSeGADigitizer();

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
    ** Defines the REAL experimental resolution and Thresholds of the Ge
    *Crystals
    *@param isRealSet  Bool parameter used to set the experimental Resolution and
    *Thresholds
    **/
   void SetRealConfig(Bool_t isRealSet);

   /** Public method SetExpEnergyRes
    **
    ** Sets the experimental energy resolution of the Ge crystals
    **/
   void SetExpEnergyRes(Double_t crystalResGe);

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
    ** Adds a SeGACrystalCal data
    **/
   AtSeGACrystalCalData *AddCrystalCal(Int_t ident, Double_t energy, ULong64_t time);

private:
   void SetParameter();
   /** Private method NUSmearing
    **
    ** Smears the energy according to some non-uniformity distribution
    ** Very simple scheme where the NU is introduced as a flat random
    ** distribution with limits fNonUniformity (%) of the energy value.
    **/
   Double_t NUSmearing(Double_t inputEnergy);

   /** Private method ExpResSmearingGe
    **
    ** Smears the energy according to some Experimental Resolution distribution
    ** Very simple scheme where the Experimental Resolution
    ** is introduced as a gause random distribution with a width given by the
    ** parameter fResolutionGe (in % @ MeV). Scales according to 1/sqrt(E)
    **/

   /** Private method ExpResSmearingGe
    **/
   Double_t ExpResSmearingGe(Double_t inputEnergy);

   Bool_t isGe(Int_t id);

   ClassDef(AtSeGADigitizer, 1);
};

#endif
