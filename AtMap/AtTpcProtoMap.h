/*********************************************************************
 *   ATTPC Mapping Class	AtTpcProtoMap.h			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 31-03-2015 11:42 JST					     *
 *								     *
 *********************************************************************/
//#pragma once
#ifndef ATTPCPROTOMAP_H
#define ATTPCPROTOMAP_H

#include "AtTpcMap.h"
#include "TFile.h"
#include "TH2Poly.h"
#include <vector>
#include <fstream>
#include <map>

class AtTpcProtoMap : public AtMap {
public:
   AtTpcProtoMap();
   ~AtTpcProtoMap();

   virtual void GenerateAtTpc() override;
   virtual void Dump() override;
   virtual std::vector<Float_t> CalcPadCenter(Int_t PadRef) override;

   virtual TH2Poly *GetAtTpcPlane() override;
   TH2Poly *GetAtTpcPlane(TString TH2Poly_name);

   Bool_t SetGeoFile(TString geofile);
   Bool_t SetProtoMap(TString file);
   virtual Int_t BinToPad(Int_t binval) override;

   TFile *f;
   TH2Poly *hProto;
   Int_t bin;

   Bool_t kIsFileSet;
   Bool_t kIsGenerated;
   Bool_t kIsProtoMapSet;

   std::ifstream *InProtoMap;
   std::map<Int_t, std::vector<Float_t>> ProtoGeoMap;
   std::map<Int_t, Int_t> ProtoBinMap;

   ClassDefOverride(AtTpcProtoMap, 1);
};

#endif
