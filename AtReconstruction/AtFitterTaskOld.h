/*********************************************************************
 *   Fitter Task AtFitterTask.hh			             *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 3/10/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATFITTERTASKOLD
#define ATFITTERTASKOLD

#include "AtFormat.h"
#include "AtKinematics.h"
#include "AtParsers.h"

#include <FairTask.h>

#include <Rtypes.h>

#include "EventDisplay.h"
#include "Exception.h"
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
namespace AtFITTER {
class AtFitterOld;
} // namespace AtFITTER
namespace genfit {
class Track;
} // namespace genfit

class [[deprecated]] AtFitterTaskOld : public FairTask {

public:
   // AtFitterTask();
   ~AtFitterTaskOld() = default;
   AtFitterTaskOld(std::unique_ptr<AtFITTER::AtFitterOld> fitter);

   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   void SetPersistence(Bool_t value = kTRUE);

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);

private:
   TString fInputBranchName;
   TString fOutputBranchName;

   Bool_t fIsPersistence; //!< Persistence check variable

   std::unique_ptr<AtFITTER::AtFitterOld> fFitter;
   AtDigiPar *fPar{nullptr};
   TClonesArray *fPatternEventArray;
   TClonesArray fTrackingEventArray;

   std::size_t fEventCnt{0};

   ClassDef(AtFitterTaskOld, 1);
};

#endif
