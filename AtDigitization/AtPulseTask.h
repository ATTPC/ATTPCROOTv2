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
#include <TString.h>

#include <cstddef>
#include <functional> // for function
#include <map>
#include <memory>
#include <type_traits> // for add_pointer_t

class AtMap;
class AtSimulatedPoint;
class AtDigiPar;
class AtPulse;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPulseTask : public FairTask {

protected:
   using AtMapPtr = std::shared_ptr<AtMap>;
   using ResponseFunctionType = std::add_pointer_t<double(double)>;
   /// Function (or callable object) to use as the response function. Parameters are padNum and time (us)
   using ResponseFunc = std::function<double(int, double)>;

   Int_t fEventID{0};                     //! EventID
   Bool_t fIsPersistent{true};            //!< If true, save container
   Bool_t fIsPersistentAtTpcPoint{false}; //!< If true, save container
   Bool_t fSaveMCInfo{false};             //!<< Propagates MC information (adds AtTpcPoint branch to output

   TString fOutputBranchName{"AtRawEvent"};

   TClonesArray *fSimulatedPointArray{nullptr}; //!< drifted electron array (input)
   TClonesArray *fMCPointArray{nullptr};        //!< MC Point Array (input)
   TClonesArray fRawEventArray;                 //!< Raw Event array (only one)

   std::multimap<Int_t, std::size_t> MCPointsMap; //!< [padNum] = mcPointID

   std::shared_ptr<AtPulse> fPulse; //!
   AtDigiPar *fPar{nullptr};

public:
   AtPulseTask(std::shared_ptr<AtPulse> pulse);
   ~AtPulseTask() = default;

   void SetPersistence(Bool_t val) { fIsPersistent = val; }
   void SetPersistenceAtTpcPoint(Bool_t val) { fIsPersistentAtTpcPoint = val; }
   void SetSaveMCInfo() { fSaveMCInfo = true; }
   void SetOutputBranch(TString branchName) { fOutputBranchName = branchName; }

   virtual InitStatus Init() override;        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.
   virtual void SetParContainers() override;  //!< Load the parameter container from the runtime database.

protected:
   void FillPointsMap(AtSimulatedPoint *point);
   void saveMCInfo(int mcPointID, int padNumber, int trackID);
   void reset();

   ClassDefOverride(AtPulseTask, 5);
};

#endif
