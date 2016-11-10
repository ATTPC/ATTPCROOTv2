/*****************************************************************/
/*    ATClusterizeTask: Simulates the ionized electrons which are    */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef ATClusterizeTask_H
#define ATClusterizeTask_H


#include "FairTask.h"
#include "FairMCPoint.h"

#include "TClonesArray.h"
#include "ATDigiPar.hh"
#include "ATGas.hh"
#include "AtTpcPoint.h"



class ATClusterizeTask : public FairTask
{
  public:
     ATClusterizeTask();
     ~ATClusterizeTask();

     void SetPersistence(Bool_t val) { fIsPersistent = val; }
     //void SetTestMode()              { fTestMode = kTRUE; };

    virtual InitStatus Init();        //!< Initiliazation of task at the beginning of a run.
    virtual void Exec(Option_t* opt); //!< Executed for each event.
    //virtual void SetParContainers();  //!< Load the parameter container from the runtime database.

   private:
    Int_t fEventID; //!< EventID

    TClonesArray* fMCPointArray;
    AtTpcPoint* fMCPoint;
    TClonesArray* fElectronNumberArray; //!< Primary cluster array
    Bool_t fIsPersistent;               //!< If true, save container


     ClassDef(ATClusterizeTask,1);

};

#endif
