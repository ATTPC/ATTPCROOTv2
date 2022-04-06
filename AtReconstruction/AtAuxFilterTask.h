#ifndef ATAUXFILTERTASK_H
#define ATAUXFILTERTASK_H

/*
 * This task applies the passed AtFilter to any aux pads in the array
 * auxPads. The result is added to the input AtRawEvent as a new aux pad
 *
 */

// FairRoot classes
#include <FairTask.h>
// ROOT classes
#include <TString.h>
#include <Rtypes.h>
#include <string>
#include <vector>

// ATTPCROOT classes;
class AtFilter;
class TClonesArray;

using stringVec = std::vector<std::string>;

class AtAuxFilterTask : public FairTask {

private:
   TClonesArray *fInputEventArray; // AtRawEvent
   TString fInputEventBranchName;

   AtFilter *fFilter;
   stringVec auxPads;

public:
   AtAuxFilterTask(AtFilter *filter);
   ~AtAuxFilterTask();

   void AddAuxPad(std::string pad);
   void SetInputBranchName(TString branchName);
   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
};
#endif //#ifndef ATAUXFILTERTASK_H
