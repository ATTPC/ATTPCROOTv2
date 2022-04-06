/*****************************************************************/
/*    AtTriggerTask: Simulates the ionized electrons which are    */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtTriggerTask_H
#define AtTriggerTask_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <TString.h>

#include <FairTask.h>

class AtEvent;
class AtRawEvent;
class AtTrigger;
class AtTriggerPar;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

class AtTriggerTask : public FairTask {
public:
   AtTriggerTask();
   ~AtTriggerTask();

   void SetPersistence(Bool_t val) { fIsPersistent = val; }
   void SetAtMap(TString mapPath);

   virtual InitStatus Init();        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt); //!< Executed for each event.
   virtual void SetParContainers();  //!< Load the parameter container from the runtime database.

private:
   AtTriggerPar *fPar;
   AtTrigger *fTrigger;
   TClonesArray *fAtRawEventArray;
   TClonesArray *fAtEventArray;
   TClonesArray *fAtRawEventArray_acc;
   TClonesArray *fAtEventArray_acc;
   AtEvent *fEvent;
   AtRawEvent *fRawEvent;
   TString fMapPath;

   Bool_t fIsTrigger;
   Bool_t fIsPersistent;

   ClassDef(AtTriggerTask, 1);
};

#endif
