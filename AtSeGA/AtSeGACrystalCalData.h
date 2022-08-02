/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

#ifndef ATSEGACRYSTALCALDATA_H
#define ATSEGACRYSTALCALDATA_H

#include "FairMultiLinkedData.h"
#include "TObject.h"

class AtSeGACrystalCalData : public FairMultiLinkedData {
public:
   /** Default constructor **/
   AtSeGACrystalCalData();

   /** Constructor with arguments
    *@param fCrystalId   Crystal unique identifier
    *@param fEnergy      Total energy deposited on the crystal ([GeV] in sim)
    *@param fTime        Time since event start [ns]
    **/
   AtSeGACrystalCalData(Int_t ident, Double_t energy, ULong64_t time);

   /** Copy constructor **/
   AtSeGACrystalCalData(const AtSeGACrystalCalData &);

   AtSeGACrystalCalData &operator=(const AtSeGACrystalCalData &) { return *this; }

   /** Destructor **/
   virtual ~AtSeGACrystalCalData() {}

   /** Accessors **/
   inline const Int_t &GetCrystalId() const { return fCrystalId; }
   inline const Double_t &GetEnergy() const { return fEnergy; }
   inline const ULong64_t &GetTime() const { return fTime; }

   /** Modifiers **/
   void SetCrystalId(Int_t ident) { fCrystalId = ident; }
   void SetEnergy(Double32_t energy) { fEnergy = energy; }
   void SetTime(ULong64_t time) { fTime = time; }
   void AddMoreEnergy(Double32_t moreEnergy) { fEnergy += moreEnergy; }
   /** Output to screen **/
   virtual void Print(const Option_t *opt) const;

protected:
   Double32_t fEnergy; // total energy in the crystal
   ULong64_t fTime;    // time of the interaction
   Int_t fCrystalId;   // crystal unique identifier

public:
   ClassDef(AtSeGACrystalCalData, 1)
};

#endif
