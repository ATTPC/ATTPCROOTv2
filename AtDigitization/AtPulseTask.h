/*****************************************************************/
/*    AtPulseTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtPulseTask_H
#define AtPulseTask_H

#include "FairTask.h"

class TClonesArray;
class AtDigiPar;
class AtMap;
class AtRawEvent;
class TF1;
class TH1F;
class TH2Poly;

using AtMapPtr = std::shared_ptr<AtMap>;

class AtPulseTask : public FairTask {

private:
   AtMapPtr fMap; //!< AtTPC map

   AtDigiPar *fPar;      //!< Base parameter container.
   Int_t fEventID;       //!< EventID
   Double_t fGain;       //!< Gain.
   Double_t fGETGain;    //!< GET Gain.
   Int_t fPeakingTime;   //!< Electronic peaking time
   Int_t fTBTime;        //!< Time bucket size
   Int_t fNumTbs;        //!< Number of time buckers
   Bool_t fIsPersistent; //!< If true, save container

   TClonesArray *fDriftedElectronArray; //!< drifted electron array (input)
   TClonesArray *fRawEventArray;        //!< Raw Event array(only one)
   TClonesArray *fMCPointArray;         //!< MC Point Array
   AtRawEvent *fRawEvent;               //!< Raw Event Object
   TH2Poly *fPadPlane;                  //!< pad plane

   Int_t fInternalID; //!< Internal ID

   std::map<Int_t, TH1F *> electronsMap;          //!<
   TH1F **eleAccumulated;                         //!<
   std::multimap<Int_t, std::size_t> MCPointsMap; //!< Correspondance between MC Points and pads

   TF1 *gain; //!<

   Bool_t fIsSaveMCInfo; //!<< Propagates MC information
   void saveMCInfo(int mcPointID, int padNumber, int trackID);

public:
   AtPulseTask();
   ~AtPulseTask();

   void SetPersistence(Bool_t val) { fIsPersistent = val; }
   void SetSaveMCInfo() { fIsSaveMCInfo = kTRUE; }
   void SetMap(AtMapPtr map) { fMap = map; };

   virtual InitStatus Init() override;        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.
   virtual void SetParContainers() override;  //!< Load the parameter container from the runtime database.

   ClassDefOverride(AtPulseTask, 2);
};

template <typename Iterator>
bool wasAlreadyInTheMap(std::pair<Iterator, bool> const &insertionResult)
{
   return !insertionResult.second;
}

#endif
