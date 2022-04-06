#include "AtParsers.h"

#include <TDOMParser.h>
#include <iostream>
#include <memory>

#include <Rtypes.h>

ClassImp(AtTools::AtParsers)

   AtTools::AtParsers::AtParsers()
{
}

AtTools::AtParsers::~AtParsers() {}

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
