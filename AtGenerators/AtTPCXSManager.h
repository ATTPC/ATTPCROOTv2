// -------------------------------------------------------------------------
// -----                    AtTPCXSManager header file                 -----
// -----               Created 18/05/2021 by Y. Ayyad                  -----
// -------------------------------------------------------------------------

#ifndef AtTPCXSMANAGER_H
#define AtTPCXSMANAGER_H

#include <Rtypes.h>
#include <TObject.h>
#include <string>
#include <memory>

class TBuffer;
class TClass;
class TH2F;
class TMemberInspector;

class AtTPCXSManager : public TObject {

public:
   AtTPCXSManager();
   ~AtTPCXSManager();

   bool SetExcitationFunction(std::string filename);

   inline std::shared_ptr<TH2F> GetExcitationFunction() { return fExFunction; }

private:
   std::string fExFunctionFile;
   std::shared_ptr<TH2F> fExFunction;
   Bool_t kIsExFunction;

   ClassDef(AtTPCXSManager, 1)
};

extern AtTPCXSManager *gAtXS; // global

#endif
