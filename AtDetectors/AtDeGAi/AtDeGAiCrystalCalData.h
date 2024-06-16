/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#ifndef ATDEGAICRYSTALCALDATA_H
#define ATDEGAICRYSTALCALDATA_H

#include <FairMultiLinkedData.h>

#include <Rtypes.h> // for Int_t, ULong64_t, Double32_t, THash...

class TBuffer;
class TClass;
class TMemberInspector;

class AtDeGAiCrystalCalData : public FairMultiLinkedData {
protected:
   Double32_t fEnergy; // total energy in the crystal
   ULong64_t fTime;    // time of the interaction
   Int_t fDetCopyID;   // crystal unique identifier

public:
   /** Default constructor **/
   AtDeGAiCrystalCalData();

   /** Constructor with arguments
    *@param fDetCopyID   Crystal unique identifier
    *@param fEnergy      Total energy deposited on the crystal ([GeV] in sim)
    *@param fTime        Time since event start [ns]
    **/
   AtDeGAiCrystalCalData(Int_t ident, Double_t energy, ULong64_t time);

   /** Copy constructor **/
   AtDeGAiCrystalCalData(const AtDeGAiCrystalCalData &) = default;

   AtDeGAiCrystalCalData &operator=(const AtDeGAiCrystalCalData &) { return *this; }

   /** Destructor **/
   virtual ~AtDeGAiCrystalCalData() {}

   /** Accessors **/
   inline const Int_t &GetDetCopyID() const { return fDetCopyID; }
   inline const Double_t &GetEnergy() const { return fEnergy; }
   inline const ULong64_t &GetTime() const { return fTime; }

   /** Modifiers **/
   void SetDetCopyID(Int_t ident) { fDetCopyID = ident; }
   void SetEnergy(Double32_t energy) { fEnergy = energy; }
   void SetTime(ULong64_t time) { fTime = time; }
   void AddMoreEnergy(Double32_t moreEnergy) { fEnergy += moreEnergy; }
   /** Output to screen **/
   virtual void Print(const Option_t *opt) const;

   ClassDef(AtDeGAiCrystalCalData, 1)
};

#endif
