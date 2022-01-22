#ifndef ATFILTERTASK_H
#define ATFILTERTASK_H

// FairRoot classes
#include "FairTask.h"

// ATTPCROOT classes;
class AtFilter;

// ROOT classes
class TClonesArray;

class AtFilterTask : public FairTask {

private:
   TClonesArray *fInputEventArray;  // AtRawEvent
   TClonesArray *fOutputEventArray; // AtRawEvent

   AtFilter *fFilter;
   Bool_t fIsPersistent;
   Bool_t fFilterAux;

public:
   AtFilterTask(AtFilter *filter);
   ~AtFilterTask();

   void SetPersistence(Bool_t value);
   void SetFilterAux(Bool_t value);
   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
};
#endif //#ifndef ATFILTERTASK_H
