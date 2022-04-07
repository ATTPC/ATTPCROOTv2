/*********************************************************************
 *   Multipurpose parsers                      			     *
 *   Author: Y. Ayyad yassid.ayyad@usc.es            	             *
 *   Log: 03/21/2022 					             *
 *								     *
 *********************************************************************/

#ifndef ATPARSERS_H
#define ATPARSERS_H

#include <Rtypes.h>
#include <TString.h>
#include <TObject.h>
#include <string>
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

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
