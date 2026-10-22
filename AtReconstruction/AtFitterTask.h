/*********************************************************************
 *   Fitter Task AtFitterTask.hh			             *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 3/10/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATFITTERTASK
#define ATFITTERTASK

#include "AtFormat.h"
#include "AtKinematics.h"
#include "AtParsers.h"

#include <FairTask.h>

#include <Rtypes.h>
#include <TClonesArray.h>

#include "FairLogger.h"
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRunAna.h"

#include <cstddef>
#include <string>
#include <vector>

class AtDigiPar;
class FairLogger;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;
class AtTrack;

namespace AtTools {
class AtTrackTransformer;
} // namespace AtTools
namespace EventFit {
class AtFitter;
} // namespace EventFit

/**
 * Task that takes a certain AtFitter and uses to fit an AtPatternEvent. The AtFitter may need access to the AtRawEvent
 * or AtEvent as well, so pointers to them are also read and passed to the AtFitter. An AtFitMetadata object may also be
 * written, which would contain the fit metadata information for all fits done to all AtTracks.
 * Specific logic of the fitting is contained in AtFitter and derived classes.
 */
class AtFitterTask : public FairTask {
public:
   AtFitterTask(std::unique_ptr<EventFit::AtFitter> fitter);
   ~AtFitterTask() = default;

   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   void SetPersistence(Bool_t value = kTRUE);

   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);
   void SetFitMetadataBranch(TString branchName);

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);

private:
   TString fInputBranchName;
   TString fOutputBranchName;

   Bool_t fIsPersistence; //!< Persistence check variable

   std::unique_ptr<EventFit::AtFitter> fFitter;
   AtDigiPar *fPar{nullptr};
   TClonesArray *fPatternEventArray;
   TClonesArray fTrackingEventArray;

   std::size_t fEventCnt{0};

   // Include the option to input AtRawEvent and AtEvent in case some specific AtFitter needs it.
   TString fRawEventBranchName;
   TString fEventBranchName;
   TClonesArray *fRawEventArray;
   TClonesArray *fEventArray;

   // Include the option to store all the fit metadata in a AtFitResult branch.
   TString fFitMetadataBranchName;
   TClonesArray fFitMetadataArray;
   bool fSaveFitMetadata{false};
};

#endif
