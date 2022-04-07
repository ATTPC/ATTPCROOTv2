/*****************************************************************/
/*    AtAvalancheTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 23-07-2015     				 */
/*    Author: Yassid Ayyad (NSCL)				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtAVALANCHETASK_H
#define AtAVALANCHETASK_H

#include <Rtypes.h>
#include <FairTask.h>

class AtDigiPar;
class AtGas;
class AtMCPoint;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

class AtAvalancheTask : public FairTask {
public:
   AtAvalancheTask();
   ~AtAvalancheTask();

   virtual InitStatus Init();        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt); //!< Executed for each event.
   virtual void SetParContainers();  //!< Load the parameter container from the runtime database.

private:
   Int_t fEventID; //!< EventID

   TClonesArray *fMCPointArray;
   AtMCPoint *fMCPoint;

   // TClonesArray* fElectronArray;

   AtDigiPar *fPar; //!< Base parameter container.
   AtGas *fGas;     //!< Gas parameter container.

   Double_t fEIonize;  //!< Effective ionization energy of gas. [eV]
   Double_t fVelDrift; //!< Drift velocity of electron in gas. [mm/ns]
   Double_t fCoefT;    //!< Transversal diffusion coefficient. [mm^(-1/2)]
   Double_t fCoefL;    //!< Longitudinal diffusion coefficient. [mm^(-1/2)]
   Double_t fGain;     //!< Gain.

   ClassDef(AtAvalancheTask, 1);
};

#endif
