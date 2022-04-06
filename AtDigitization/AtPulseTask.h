/*****************************************************************/
/*    AtPulseTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtPulseTask_H
#define AtPulseTask_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <cstddef>
#include <iterator>
#include <map>
#include <memory>

#include "FairTask.h"

class TClonesArray;
class AtDigiPar;
class AtMap;
class AtRawEvent;
class TF1;
class TH1F;
class TH2Poly;
class AtSimulatedPoint;
class TBuffer;
class TClass;
class TMemberInspector;

using AtMapPtr = std::shared_ptr<AtMap>;

class AtPulseTask : public FairTask {

protected:
   AtMapPtr fMap; //!< AtTPC map

   AtDigiPar *fPar;             //!< Base parameter container.
   Int_t fEventID = 0;          //!< EventID
   Double_t fGain = 0;          //!< Micromegas gain.
   Double_t fLowGainFactor = 0; //! If pad is AtMap::kLowGain multiply gain by this factor
   Double_t fGETGain = 0;       //!< GET Gain.
   Double_t fPeakingTime = 0;   //!< Electronic peaking time in us
   Double_t fTBTime = 0;        //!< Time bucket size in us
   Int_t fNumTbs = 0;           //!< Number of time buckers
   Int_t fTBEntrance = 0;       //! Window location in timebuckets (from config)
   Int_t fTBPadPlane = 0;       //! Pad plane location in TBs (calculated from DriftVelocity, TBEntrance, ZPadPlane

   Bool_t fIsPersistent = kTRUE;  //!< If true, save container
   Bool_t fIsSaveMCInfo = kFALSE; //!<< Propagates MC information
   Bool_t fUseFastGain = kTRUE;

   TClonesArray *fSimulatedPointArray; //!< drifted electron array (input)
   TClonesArray *fRawEventArray;       //!< Raw Event array(only one)
   TClonesArray *fMCPointArray;        //!< MC Point Array
   AtRawEvent *fRawEvent;              //!< Raw Event Object
   TH2Poly *fPadPlane;                 //!< pad plane

   std::map<Int_t, TH1F *> electronsMap;          //!<
   TH1F **eleAccumulated;                         //!<
   std::multimap<Int_t, std::size_t> MCPointsMap; //!< [padNum] = mcPointID

   TF1 *gain; //!<
   Double_t avgGainDeviation;

   void saveMCInfo(int mcPointID, int padNumber, int trackID);
   void setParameters();
   void getPadPlaneAndCreatePadHist();
   void reset();
   void generateTracesFromGatheredElectrons();
   double getAvgGETgain(Int_t numElectrons);

   // Add all electrons for AtSimulatedPoint to the electronMap
   // Returns if any electrons were added
   virtual bool gatherElectronsFromSimulatedPoint(AtSimulatedPoint *point);

public:
   AtPulseTask();
   AtPulseTask(const char *name);
   ~AtPulseTask();

   void SetLowGainFactor(Double_t factor) { fLowGainFactor = factor; }
   void SetPersistence(Bool_t val) { fIsPersistent = val; }
   void SetSaveMCInfo() { fIsSaveMCInfo = kTRUE; }
   void SetMap(AtMapPtr map) { fMap = map; };
   void UseFastGain(Bool_t val) { fUseFastGain = val; }

   virtual InitStatus Init() override;        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.
   virtual void SetParContainers() override;  //!< Load the parameter container from the runtime database.

   ClassDefOverride(AtPulseTask, 4);
};

template <typename Iterator>
bool wasAlreadyInTheMap(std::pair<Iterator, bool> const &insertionResult)
{
   return !insertionResult.second;
}

#endif
