/*****************************************************************/
/*    ATTriggerTask: Simulates the ionized electrons which are    */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef ATTriggerTask_H
#define ATTriggerTask_H

#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "FairTask.h"
#include "FairMCPoint.h"

#include "TClonesArray.h"
#include "ATTriggerPar.hh"
#include "ATTrigger.hh"



class ATTriggerTask : public FairTask
{
  public:
     ATTriggerTask();
     ~ATTriggerTask();

     void SetPersistence(Bool_t val) { fIsPersistent = val; }
     void SetAtMap(TString mapPath);

    virtual InitStatus Init();        //!< Initiliazation of task at the beginning of a run.
    virtual void Exec(Option_t* opt); //!< Executed for each event.
    virtual void SetParContainers();  //!< Load the parameter container from the runtime database.

   private:

     ATTriggerPar* fPar;
     ATTrigger* fTrigger;
     TClonesArray* fATRawEventArray;
     TClonesArray* fATEventArray;
     TClonesArray* fATRawEventArray_acc;
     TClonesArray* fATEventArray_acc;
     ATEvent*      fEvent;
     ATRawEvent*   fRawEvent;
     TString       fMapPath;
     

     Bool_t fIsTrigger;
     Bool_t fIsPersistent;


     ClassDef(ATTriggerTask,1);

};

#endif
