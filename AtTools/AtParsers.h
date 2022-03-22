/*********************************************************************
 *   Multipurpose parsers                      			     *
 *   Author: Y. Ayyad yassid.ayyad@usc.es            	             *
 *   Log: 03/21/2022 					             *
 *								     *
 *********************************************************************/

#ifndef ATPARSERS_H
#define ATPARSERS_H

#include <iostream>

#include "TROOT.h"
#include "TObject.h"
#include "TDOMParser.h"
#include "TXMLAttr.h"
#include "TXMLNode.h"
#include "TList.h"

namespace AtTools {

struct IonFitInfo {
   std::string _ionName;
   UInt_t _PDG;
   Int_t _mass;
   Int_t _atomicNumber;
   std::string _eLossFile;
};

class AtParsers : public TObject {

public:
   AtParsers();
   ~AtParsers();

   Int_t ParseIonFitXML(TString filename);

private:
   std::vector<IonFitInfo> ionList;

   ClassDef(AtParsers, 1)
};

} // namespace AtTools

#endif
