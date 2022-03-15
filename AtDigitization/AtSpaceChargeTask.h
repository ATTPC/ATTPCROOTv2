/*****************************************************************/
/*    AtSpaceChargeTask: Simulates the effect of space charge on */
/*    the electrons as they're drifted to the pad plane. Based   */
/*    on code by Juan Zamora                                     */
/*    Author: Adam Anthony (NSCL) 				 */
/*    anthonya@frib.msu.edu                                      */
/*****************************************************************/
#ifndef ATSPACECHARGETASK_H
#define ATSPACECHARGETASK_H

#include "FairTask.h"

class TClonesArray;
class AtDigiPar;
class AtMap;
class AtMCPoint;

using AtMapPtr = std::shared_ptr<AtMap>;

class AtSpaceChargeTask : public FairTask {

protected:
   AtMapPtr fMap; //!< AtTPC map

   Int_t fEventID = 0; //!< EventID

   Double_t fVelDrift = 0;    //!< Drift velocity of electron in gas. [mm/ns]
   Double_t fDetPadPlane = 0; //!< Position of the pad plane with respect to the entrance [mm]

   AtDigiPar *fPar; //!< Base parameter container.

   TClonesArray *fMCPointArray;          //!< All AtMCpoints (input)
   TClonesArray *fSpaceChargePointArray; //!< space charge corrected MC Points (output)
   Bool_t fIsPersistent = kTRUE;         //!< If true, save container

   TString fInputBranchName = "AtTpcPoint";
   TString fOutputBranchName = "AtSpaceChargePoint";

   void getParameters();
   void processPoint(AtMCPoint *point);

public:
   AtSpaceChargeTask();
   AtSpaceChargeTask(const char *name);
   ~AtSpaceChargeTask();

   void SetPersistence(Bool_t val) { fIsPersistent = val; }
   void SetMap(AtMapPtr map) { fMap = map; };
   void SetInputBranchName(TString name) { fInputBranchName = name; }
   void SetOutputBranchName(TString name) { fOutputBranchName = name; }

   virtual InitStatus Init() override;        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.
   virtual void SetParContainers() override;  //!< Load the parameter container from the runtime database.

   ClassDefOverride(AtSpaceChargeTask, 1);
};

#endif //#ifndef ATSPACECHARGETASK_H
