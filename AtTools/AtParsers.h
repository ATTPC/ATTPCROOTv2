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

#include "AtFormat.h"

namespace AtTools {

struct IonFitInfo {
   std::string _ionName;
   UInt_t _PDG;
   Int_t _mass;
   Int_t _atomicNumber;
   UInt_t _MassNumber;
   std::string _eLossFile;

  friend std::ostream& operator << (std::ostream& out, const AtTools::IonFitInfo& ifi)
   {

     out<<cGREEN<<" Ion name :"<<ifi._ionName<<"\n";
     out<<" PDG : "<<ifi._PDG<<"\n";
     out<<" Mass (AMU) : "<<ifi._mass<<"\n";
     out<<" Atomic Number : "<<ifi._atomicNumber<<"\n";
     out<<" Mass Number : "<<ifi._MassNumber<<"\n";
     out<<" Energy loss file : "<<ifi._eLossFile<<cNORMAL<<"\n"; 
     return out;
   }

};

class AtParsers : public TObject {

public:
   AtParsers();
   ~AtParsers();

   Int_t ParseIonFitXML(TString filename);
   std::vector<IonFitInfo>* GetIonFile() {return &ionList;}
    
private:

  void ParseIonList(TXMLNode *node);
  IonFitInfo ParseIon(TXMLNode *node, Int_t id);
  
  std::vector<IonFitInfo> ionList;
  
  
   ClassDef(AtParsers, 1)
};

} // namespace AtTools

#endif
