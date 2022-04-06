/*********************************************************************
 *   ATTPC Mapping Class	AtTpcProtoMap.h			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 31-03-2015 11:42 JST					     *
 *								     *
 *********************************************************************/
//#pragma once
#ifndef ATTPCPROTOMAP_H
#define ATTPCPROTOMAP_H

#include <Math/Point2Dfwd.h>
#include <Rtypes.h>
#include <TString.h>
#include <vector>
#include <map>

#include "AtMap.h"

class TBuffer;
class TClass;
class TFile;
class TH2Poly;
class TMemberInspector;

class AtTpcProtoMap : public AtMap {
protected:
   TFile *f; // Raw pointer, owned by gROOT
   Int_t bin;

   Bool_t kIsFileSet;
   Bool_t kIsGenerated;
   Bool_t kIsProtoMapSet;

   std::map<Int_t, std::vector<Float_t>> ProtoGeoMap;
   std::map<Int_t, Int_t> ProtoBinMap;

public:
   AtTpcProtoMap();
   ~AtTpcProtoMap();

   virtual void GeneratePadPlane() override;
   virtual void Dump() override;
   virtual ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) override;
   virtual TH2Poly *GetPadPlane() override;
   virtual Int_t BinToPad(Int_t binval) override;
   TH2Poly *GetAtTpcPlane(TString TH2Poly_name);

   Bool_t SetGeoFile(TString geofile);
   Bool_t SetProtoMap(TString file);

   ClassDefOverride(AtTpcProtoMap, 2);
};

#endif
