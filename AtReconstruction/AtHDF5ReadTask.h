#ifndef ATHDF4READTASK_H
#define ATHDF4READTASK_H

#include <FairTask.h>

#include <Rtypes.h> // for Bool_t, Int_t, Option_t
#include <TClonesArray.h>
#include <TString.h> // for TString

#include <H5Cpp.h>

#include <memory> // for unique_ptr

class TBuffer;
class TClass;
class TMemberInspector;

class AtHDF5ReadTask : public FairTask {

protected:
   TString fInputFileName;
   TString fOutputBranchName;

   std::unique_ptr<H5::H5File> fFile{nullptr}; //!
   TClonesArray fEventArray;

   Bool_t fIsPersistence{false};
   Int_t fEventNum{0};

public:
   AtHDF5ReadTask(TString fileName, TString outputBranchName = "AtEventH");

   void SetPersistence(bool val) { fIsPersistence = val; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

   ClassDefOverride(AtHDF5ReadTask, 1);
};

#endif //#ifndef ATHDF4READTASK_H
