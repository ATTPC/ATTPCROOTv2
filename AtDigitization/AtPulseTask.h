/*****************************************************************/
/*    AtPulseTask: Simulates the ionized electrons which are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtPulseTask_H
#define AtPulseTask_H

#include <FairTask.h>

#include <Rtypes.h>
#include <TClonesArray.h>
#include <TF1.h> //Needed for unique_ptr<TF1>
#include <TH1.h> //Needed for unique_ptr<TH1F>

#include <cstddef>
#include <iterator>
#include <map>
#include <memory>
#include <vector>

class AtMap;
class AtRawEvent;
class TH2Poly;
class AtSimulatedPoint;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPulseTask : public FairTask {

protected:
   using AtMapPtr = std::shared_ptr<AtMap>;
   using ResponseFunctionType = std::add_pointer_t<double(double)>;

   AtMapPtr fMap; //!< AtTPC map

   Int_t fEventID = 0;          //! EventID
   Double_t fGain = 0;          //! Micromegas gain.
   Double_t fLowGainFactor = 0; //! If pad is AtMap::kLowGain multiply gain by this factor
   Double_t fGETGain = 0;       //! GET Gain.
   Double_t fPeakingTime = 0;   //! Electronic peaking time in us
   Double_t fTBTime = 0;        //! Time bucket size in us
   Int_t fNumTbs{512};          //! Number of time buckers
   Int_t fTBEntrance = 0;       //! Window location in timebuckets (from config)
   Int_t fTBPadPlane = 0;       //! Pad plane location in TBs (calculated from DriftVelocity, TBEntrance, ZPadPlane
   ResponseFunctionType fResponseFunction{&(AtPulseTask::nominalResponseFunction)};

   Bool_t fIsPersistent = true;  //!< If true, save container
   Bool_t fIsSaveMCInfo = false; //!<< Propagates MC information
   Bool_t fUseFastGain = true;

   TClonesArray *fSimulatedPointArray{}; //!< drifted electron array (input)
   TClonesArray fRawEventArray;          //!< Raw Event array(only one)
   TClonesArray *fMCPointArray{};        //!< MC Point Array
   TH2Poly *fPadPlane{};

   AtRawEvent *fRawEvent{}; //!< Raw Event Object

   std::map<Int_t, TH1F *> electronsMap;              //!<
   std::vector<std::unique_ptr<TH1F>> eleAccumulated; //!<
   std::multimap<Int_t, std::size_t> MCPointsMap;     //!< [padNum] = mcPointID

   std::unique_ptr<TF1> gain; //!<
   Double_t avgGainDeviation{};

public:
   AtPulseTask();
   AtPulseTask(const char *name);
   ~AtPulseTask() = default;

   void SetLowGainFactor(Double_t factor) { fLowGainFactor = factor; }
   void SetPersistence(Bool_t val) { fIsPersistent = val; }
   void SetSaveMCInfo() { fIsSaveMCInfo = kTRUE; }
   void SetMap(AtMapPtr map) { fMap = map; };
   void UseFastGain(Bool_t val) { fUseFastGain = val; }

   /**
    * @brief: Set the response funtion of the electronics for a single electron.
    *
    * @param[in] responseFunction Function pointer of the type `double responseFunction(double reducedTime)` where
    * reducedTime is the time since the arrival of the electron divided by the electronics peaking time.
    */
   void SetResponseFunction(ResponseFunctionType responseFunction) { fResponseFunction = responseFunction; }

   virtual InitStatus Init() override;        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.
   virtual void SetParContainers() override;  //!< Load the parameter container from the runtime database.

protected:
   void saveMCInfo(int mcPointID, int padNumber, int trackID);
   void setParameters();
   void getPadPlaneAndCreatePadHist();
   void reset();
   void generateTracesFromGatheredElectrons();
   double getAvgGETgain(Int_t numElectrons);
   static double nominalResponseFunction(double reducedTime);

   // Add all electrons for AtSimulatedPoint to the electronMap
   // Returns if any electrons were added
   virtual bool gatherElectronsFromSimulatedPoint(AtSimulatedPoint *point);

   ClassDefOverride(AtPulseTask, 4);
};

template <typename Iterator>
bool wasAlreadyInTheMap(std::pair<Iterator, bool> const &insertionResult)
{
   return !insertionResult.second;
}

#endif
