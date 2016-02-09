/*****************************************************************/
/*    ATAvalancheTask: Simulates the ionized electrons which are */ 
/*    drifted and amplified by avalanche when reaching the mesh. */ 
/*    Log: Created in 23-07-2015     				 */
/*    Author: Yassid Ayyad (NSCL)				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef ATAVALANCHETASK_H
#define ATAVALANCHETASK_H


#include "FairTask.h"
#include "FairMCPoint.h"

#include "TClonesArray.h"
#include "ATDigiPar.hh"
#include "ATGas.hh"
#include "AtTpcPoint.h"



class ATAvalancheTask : public FairTask 
{
  public:
     ATAvalancheTask();
     ~ATAvalancheTask();

    virtual InitStatus Init();        //!< Initiliazation of task at the beginning of a run.
    virtual void Exec(Option_t* opt); //!< Executed for each event.
    virtual void SetParContainers();  //!< Load the parameter container from the runtime database.

   private:

    Int_t fEventID; //!< EventID

    TClonesArray* fMCPointArray;     
    AtTpcPoint* fMCPoint;            

    //TClonesArray* fElectronArray;    

    ATDigiPar* fPar; //!< Base parameter container.
    ATGas*     fGas; //!< Gas parameter container.
 
    Double_t fEIonize;  //!< Effective ionization energy of gas. [eV]
    Double_t fVelDrift; //!< Drift velocity of electron in gas. [mm/ns]
    Double_t fCoefT;    //!< Transversal diffusion coefficient. [mm^(-1/2)]
    Double_t fCoefL;    //!< Longitudinal diffusion coefficient. [mm^(-1/2)]
    Double_t fGain;     //!< Gain.


     ClassDef(ATAvalancheTask,1);
    
};

#endif


