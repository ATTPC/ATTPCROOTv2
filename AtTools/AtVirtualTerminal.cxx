#include "AtVirtualTerminal.h"

#include <Rtypes.h> // for TGenericClassInfo

#include <cstdlib> // for system
#include <sstream> // IWYU pragma: keep

ClassImp(AtTools::AtVirtualTerminal);

Int_t AtTools::AtVirtualTerminal::CreateXterm()
{
   std::ostringstream oss;
   oss << "xterm";
   Int_t val = std::system(oss.str().c_str());
   return val;
}

void AtTools::AtVirtualTerminal::StreamViaXTerm() {}
