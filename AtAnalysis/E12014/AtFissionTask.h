#ifndef ATFSSIONTASK_H
#define ATFSSIONTASK_H

#include <FairTask.h>

#include <Rtypes.h> // for Bool_t, Double_t, Option_t
#include <TClonesArray.h>
#include <TString.h> // for TString

#include <memory> // for unique_ptr
#include <vector> // for vector
class AtHit;

class AtFissionEvent;

/**
 * Class for filling an AtFissionEvent that will be added to the TTree.
 * This event is essentially the composition between a space charge corrected AtPatternEvent and
 * a non space charge corrected AtEvent.
 * That pattern event is cloned into the new AtFissionEvent (which inherits AtPatternEvent), and we copy the hits from
 * uncorrected AtEvent into the new event.
 */
class AtFissionTask : public FairTask {
protected:
   TString fOutBranch{"AtFissionEvent"};
   TString fPatternBranch{"AtPatternEvent"}; //< Pattern with corrected hits
   TString fEventBranch{"AtEventH"};         //< Uncorrected event

   TClonesArray fFissionEventArray;
   TClonesArray *fPatternEventArray{nullptr};
   TClonesArray *fEventArray{nullptr};

   Bool_t fIsPersistant{true};
   Double_t fLambda; //<Value of space charge correction applied to this run

public:
   AtFissionTask(double fLambda = 0);
   virtual ~AtFissionTask() = default;

   virtual InitStatus Init();
   virtual void Exec(Option_t *opt);
   virtual void Finish() {}

   void SetPersistance(bool val) { fIsPersistant = val; }
   void SetOutBranch(TString name) { fOutBranch = name; }
   void SetPatternBranch(TString name) { fPatternBranch = name; }
   void SetUncorrectedEventBranch(TString name) { fEventBranch = name; }

protected:
   using HitVector = std::vector<std::unique_ptr<AtHit>>;
   std::vector<AtHit *> GetSortedBeamHits(AtFissionEvent *event);
   std::vector<AtHit *> GetSortedFragmentHits(AtFissionEvent *event, int fragID);
   static HitVector GetMatchingHits(const std::vector<AtHit *> hitsToFind, const HitVector &hitsToClone);
};

#endif // ATFSSIONTASK_H
