/*****************************************************************/
/*    ATPulseTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef ATPulseTask_H
#define ATPulseTask_H


#include "FairTask.h"
#include "FairMCPoint.h"

#include "TClonesArray.h"
#include "ATDigiPar.hh"
#include "TH2Poly.h"
#include "AtTpcMap.h"
#include "ATRawEvent.hh"



class ATPulseTask : public FairTask
{
  public:
     ATPulseTask();
     ~ATPulseTask();

    void SetPersistence(Bool_t val) { fIsPersistent = val; }
    virtual InitStatus Init();        //!< Initiliazation of task at the beginning of a run.
    virtual void Exec(Option_t* opt); //!< Executed for each event.
    virtual void SetParContainers();  //!< Load the parameter container from the runtime database.

   private:
    Int_t fEventID;                      //!< EventID
    Bool_t fIsPersistent;                //!< If true, save container
    TClonesArray* fDriftedElectronArray; //!< drifted electron array (input)
    TClonesArray* fRawEventArray;        //!< Raw Event array(only one)
    TH2Poly *fPadPlane;                  //!< pad plane


     ClassDef(ATPulseTask,1);

};

#endif
