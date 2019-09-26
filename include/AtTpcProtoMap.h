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


class AtTpcProtoMap : public AtTpcMap 
{
   public:
	AtTpcProtoMap();
	~AtTpcProtoMap();

	void Initialize();
        void GenerateATTPC();
        TH2Poly* GetATTPCPlane();
        TH2Poly* GetATTPCPlane(TString TH2Poly_name);
        Bool_t SetGeoFile(TString geofile);
        std::vector<Float_t> CalcPadCenter(Int_t PadRef);
        Bool_t SetProtoMap(TString file);
        Int_t BinToPad(Int_t binval);

        TFile *f;
        TH2Poly *hProto;
        Int_t bin;

        Bool_t kIsFileSet;
        Bool_t kIsGenerated;
        Bool_t kIsProtoMapSet;


        std::ifstream *InProtoMap;
	    std::map<Int_t,std::vector<Float_t>> ProtoGeoMap;
        std::map<Int_t,Int_t> ProtoBinMap;        


   ClassDef(AtTpcProtoMap,1);

};

#endif

