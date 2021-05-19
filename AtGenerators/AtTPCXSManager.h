// -------------------------------------------------------------------------
// -----                    AtTPCXSManager header file                 -----
// -----               Created 18/05/2021 by Y. Ayyad                  -----
// -------------------------------------------------------------------------

#ifndef AtTPCXSMANAGER_H
#define AtTPCXSMANAGER_H

#include "FairGenerator.h"
#include "FairIon.h"
#include "FairParticle.h"
#include "TH1F.h"
#include "TH2F.h"

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <map>

class AtTPCXSManager;

class AtTPCXSManager : public TObject {

public:
   AtTPCXSManager();
   ~AtTPCXSManager();

   bool SetExcitationFunction(std::string filename);

  
  inline std::shared_ptr<TH2F> GetExcitationFunction() {return fExFunction;}
  
private:
   std::string fExFunctionFile;
   std::shared_ptr<TH2F> fExFunction;
  Bool_t kIsExFunction;
  
   ClassDef(AtTPCXSManager, 1)
};

extern AtTPCXSManager *gAtXS; // global

#endif
