#include "AtVirtualTerminal.h"

ClassImp(AtTools::AtVirtualTerminal)

   AtTools::AtVirtualTerminal::AtVirtualTerminal()
{
}
AtTools::AtVirtualTerminal::~AtVirtualTerminal() {}

Int_t AtTools::AtVirtualTerminal::CreateXterm()
{
   std::ostringstream oss;
   oss << "xterm";
   Int_t val = std::system(oss.str().c_str());
   return val;
}

void AtTools::AtVirtualTerminal::StreamViaXTerm() {}
