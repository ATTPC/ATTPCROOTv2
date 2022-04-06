/*********************************************************************
 *   ATTPC Mapping Class	AtTpcMap.h			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 13-02-2015 17:16 JST					     *
 *								     *
 *********************************************************************/

#ifndef ATTPCMAP_H
#define ATTPCMAP_H
#include <Math/Point2Dfwd.h>
#include <Rtypes.h>

#include "AtMap.h"

class TBuffer;
class TClass;
class TH2Poly;
class TMemberInspector;

class AtTpcMap : public AtMap {

public:
   AtTpcMap();
   ~AtTpcMap();

   virtual void Dump() override;
   virtual void GeneratePadPlane() override;
   virtual ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) override;
   virtual TH2Poly *GetPadPlane() override;
   virtual Int_t BinToPad(Int_t binval) override { return binval - 1; };

   /*  friend ostream & operator << (ostream& out, const AtTpcMap& p){

       std::vector<int>::iterator it;

       for(auto it=p.PadKey.begin();it!=p.PadKey.end();it++){

       out<<"  This "<<p.PadKey[0]<<std::endl;
       }

       out<<" EN Node ID :"<<p.id<<" , ";
       out<<" EN detector segment name :"<<p.detname<<" , ";
       out<<" EN module ID :"<<p.modid<<" , ";
       out<<" EN detector ID :"<<p.detID<<" , ";
       out<<" EN VME module :"<<p.vme<<" , ";
       out<<" EN Module status :"<<p.stat<<endl;
       }*/

   ClassDefOverride(AtTpcMap, 1);

protected:
   Int_t fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort);
};

#endif
