/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef SEGAPOINT_H
#define SEGAPOINT_H 1

#include "FairMCPoint.h"
#include "TVector3.h"

#include "TObject.h"

class AtSeGAPoint : public FairMCPoint {

public:
   /** Default constructor **/
   AtSeGAPoint();

   /** Constructor with arguments
    *@param trackID   Index of MCTrack
    *@param detID     Detector ID
    *@param pos      Ccoordinates at entrance to active volume [cm]
    *@param mom      Momentum of track at entrance [GeV]
    *@param crystalID Crystal ID
    *@param tof       Time since event start [ns]
    *@param length    Track length since creation [cm]
    *@param eLoss     Energy deposit [GeV]
    **/
   AtSeGAPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Int_t crystalID, Double_t tof, Double_t length,
                 Double_t eLoss, TString VolName);

   /** Destructor **/
   virtual ~AtSeGAPoint();

   /** Accessors **/
   Int_t GetCrystalID() const { return fCrystalID; }
   TString GetVolName() const { return fVolName; }

   /** Modifiers **/
   void SetCrystalID(Int_t id) { fCrystalID = id; }; // added by Marc
   void SetVolName(Int_t Vlna) { fVolName = Vlna; };

   /** Output to screen **/
   virtual void Print(const Option_t *opt) const;

private:
   /** Copy constructor **/
   AtSeGAPoint(const AtSeGAPoint &point);
   AtSeGAPoint operator=(const AtSeGAPoint &point);

protected:
   Int_t fCrystalID;
   TString fVolName

   ClassDef(AtSeGAPoint, 1)
};

#endif
