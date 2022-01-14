/*****************************************************************/
/*    AtClusterizeTask: Simulates the ionized electrons which are    */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtClusterizeTask_H
#define AtClusterizeTask_H

#include "FairTask.h"

class AtGas;
class AtDigiPar;
class AtTpcPoint;

class TClonesArray;

class AtClusterizeTask : public FairTask {
private:
   Int_t fEventID;        //!< EventID
   Double_t fEIonize;     //!< Effective ionization energy of gas. [eV]
   Double_t fFano;        //!< Fano factor of the gas
   Double_t fVelDrift;    //!< Drift velocity of electron in gas. [mm/ns]
   Double_t fCoefT;       //!< Transversal diffusion coefficient. [mm^(-1/2)]
   Double_t fCoefL;       //!< Longitudinal diffusion coefficient. [mm^(-1/2)]
   Double_t fDetPadPlane; //!< Position of the pad plane with respect to the entrance [mm]

   AtGas *fGas;     //!< Gas parameter container.
   AtDigiPar *fPar; //!< Base parameter container.

   TClonesArray *fMCPointArray;
   AtTpcPoint *fMCPoint;
   TClonesArray *fElectronNumberArray; //!< Primary cluster array
   Bool_t fIsPersistent;               //!< If true, save container

public:
   AtClusterizeTask();
   ~AtClusterizeTask();

   void SetPersistence(Bool_t val) { fIsPersistent = val; }

   virtual InitStatus Init() override;        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.
   virtual void SetParContainers() override;  //!< Load the parameter container from the runtime database.

   ClassDefOverride(AtClusterizeTask, 1);
};

#endif
