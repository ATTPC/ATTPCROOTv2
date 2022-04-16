#include "AtParsers.h"

#include <Rtypes.h>
#include <TDOMParser.h>

#include <iostream>
#include <memory>

ClassImp(AtTools::AtParsers)

   AtTools::AtParsers::AtParsers() = default;

AtTools::AtParsers::~AtParsers() = default;

Int_t AtTools::AtParsers::ParseIonFitXML(TString filename)
{

   std::shared_ptr<TDOMParser> parser = std::make_shared<TDOMParser>();
   Int_t parsecode = parser->ParseFile(filename);

   if (parsecode < 0) {
      std::cerr << parser->GetParseCodeMessage(parsecode) << "\n";
      return -1;
   }

   return 0;
}
