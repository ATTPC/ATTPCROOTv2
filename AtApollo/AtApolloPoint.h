/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef APOLLOPOINT_H
#define APOLLOPOINT_H 1

#include <Rtypes.h>
#include <RtypesCore.h>

#include "FairMCPoint.h"
#include "TVector3.h"

class TBuffer;
class TClass;
class TMemberInspector;

class AtApolloPoint : public FairMCPoint {

public:
   /** Default constructor **/
   AtApolloPoint();

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
   AtApolloPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Int_t crystalID, Double_t tof, Double_t length,
                 Double_t eLoss);

   /** Destructor **/
   virtual ~AtApolloPoint();

   /** Accessors **/
   Int_t GetCrystalID() const { return fCrystalID; }

   /** Modifiers **/
   void SetCrystalID(Int_t id) { fCrystalID = id; }; // added by Marc

   /** Output to screen **/
   virtual void Print(const Option_t *opt) const;

private:
   /** Copy constructor **/
   AtApolloPoint(const AtApolloPoint &point);
   AtApolloPoint operator=(const AtApolloPoint &point);

protected:
   Int_t fCrystalID;

   ClassDef(AtApolloPoint, 1)
};

#endif
