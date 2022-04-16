#ifndef ATVIRTUALTERMINAL_H
#define ATVIRTUALTERMINAL_H

#include <Rtypes.h> // for THashConsistencyHolder, ClassDef, Int_t

#include "TObject.h" // for TObject

#include <string> // for string
class TBuffer;
class TClass;
class TMemberInspector;

namespace AtTools {

class AtVirtualTerminal : public TObject {

public:
   AtVirtualTerminal() = default;
   ~AtVirtualTerminal() = default;

   Int_t CreateXterm();
   void StreamViaXTerm();

private:
   std::string xTermID;

   ClassDef(AtVirtualTerminal, 1)
};

} // namespace AtTools

#endif
