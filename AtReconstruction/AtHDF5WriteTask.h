#ifndef ATHDF4WRITETASK_H
#define ATHDF4WRITETASK_H

#include <FairTask.h>

#include <Rtypes.h>  // for THashConsistencyHolder, Bool_t, ClassDefOverride
#include <TString.h> // for TString

#include <H5Cpp.h>

#include <memory> // for unique_ptr

class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

class AtHDF5WriteTask : public FairTask {

protected:
   TString fOutputFileName;
   TString fInputBranchName;

   std::unique_ptr<H5::H5File> fFile{nullptr}; //!
   TClonesArray *fEventArray{nullptr};

   Bool_t fIsPersistence{false};
   Int_t fEventNum{0};

public:
   AtHDF5WriteTask(TString fileName, TString branchName = "AtEventH");

   void SetPersistence(bool val) { fIsPersistence = val; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

   ClassDefOverride(AtHDF5WriteTask, 1);
};

#endif //#ifndef ATHDF4WRITETASK_H
