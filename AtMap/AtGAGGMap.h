/*********************************************************************
 *   ATGAGG Mapping Class	AtGAGGMap.h		  	                         *
 *   Author: T. J. Gray            				                           *
 *   Log: 05-05-2026                                   					     *
 *								                                                   *
 *********************************************************************/

#ifndef ATGAGGMAP_H
#define ATGAGGMAP_H
#include "AtMap.h"

#include <Math/Point2Dfwd.h>
#include <Rtypes.h>
#include <TXMLDocument.h>
#include <TXMLNode.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtGAGGMap : public AtMap {

public:
   AtGAGGMap();
   ~AtGAGGMap();

   virtual void Dump() override;
   virtual void GeneratePadPlane() override;
   virtual ROOT::Math::XYPoint CalcPadCenter(Int_t PadRef) override;
   virtual Int_t BinToPad(Int_t binval) override { return binval - 1; };
   virtual void ParseAtTPCMap(TXMLNode *node) override;

   Int_t InhibitStrips(TString stripsFilePath);

   std::unordered_map<AtPadReference, int> fLayerMap;

   ClassDefOverride(AtGAGGMap, 1);

protected:
   Int_t fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort);
};

#endif
