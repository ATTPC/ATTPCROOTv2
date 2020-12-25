/*****************************************************************/
/*    ATPulseTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef ATPulseTask_H
#define ATPulseTask_H

#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "FairTask.h"
#include "FairMCPoint.h"

#include "TClonesArray.h"
#include "TH1D.h"
#include "ATDigiPar.hh"
#include "TH2Poly.h"
#include "AtTpcMap.h"
#include "ATRawEvent.hh"
#include "ATGas.hh"

class ATPulseTask : public FairTask {

public:
  ATPulseTask();
  ~ATPulseTask();

  void SetPersistence(Bool_t val) { fIsPersistent = val; }
  void SetSaveMCInfo()            { fIsSaveMCInfo = kTRUE;}
  virtual InitStatus Init();        //!< Initiliazation of task at the beginning of a run.
  virtual void Exec(Option_t* opt); //!< Executed for each event.
  virtual void SetParContainers();  //!< Load the parameter container from the runtime database.
  void SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap);
  inline void IsInhibitMap(Bool_t val) { fIsInhibitMap = val; }


private:
  ATGas*     fGas;                     //!< Gas parameter container.
  ATDigiPar* fPar;                     //!< Base parameter container.
  Int_t fEventID;                      //!< EventID
  Double_t fGain;                      //!< Gain.
  Double_t fGETGain;                   //!< GET Gain.
  Int_t fPeakingTime;                  //!< Electronic peaking time
  Int_t fTBTime;                       //!< Time bucket size
  Int_t fNumTbs;                       //!<
  Bool_t fIsPersistent;                //!< If true, save container
  TClonesArray* fDriftedElectronArray; //!< drifted electron array (input)
  TClonesArray* fRawEventArray;        //!< Raw Event array(only one)
  TClonesArray* fMCPointArray;         //!< MC Point Array 
  ATRawEvent* fRawEvent;               //!< Raw Event Object
  TH2Poly *fPadPlane;                  //!< pad plane
  AtTpcMap *fMap;                      //!<ATTPC map
  Int_t fInternalID;                   //!<Internal ID

  std::map<Int_t, TH1F*> electronsMap;          //!<
  TH1F** eleAccumulated;                        //!<
  std::multimap<Int_t,std::size_t> MCPointsMap; //!< Correspondance between MC Points and pads

  TF1 *gain;                         //!<

  TString fIniMap;
  TString fLowgMap;
  TString fXtalkMap;
  Bool_t fIsInhibitMap;
  Bool_t fIsSaveMCInfo; //!<< Propagates MC information

  ClassDef(ATPulseTask,1);
};

template<typename Iterator>
bool wasAlreadyInTheMap(std::pair<Iterator, bool> const& insertionResult)
{
    return !insertionResult.second;
}

#endif
