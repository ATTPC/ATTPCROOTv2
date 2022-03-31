#ifndef ATVIRTUALTERMINAL_H
#define ATVIRTUALTERMINAL_H

#include <iostream>
#include <sstream>
#include "TROOT.h"
#include "TObject.h"
#include "AtFormat.h"

namespace AtTools {


  class AtVirtualTerminal : public TObject {

public:
   AtVirtualTerminal();
   ~AtVirtualTerminal();

   Int_t CreateXterm();
   void StreamViaXTerm();

  private:
   std::string xTermID;
   
   ClassDef(AtVirtualTerminal, 1)
};

} // namespace AtTools

#endif
  
