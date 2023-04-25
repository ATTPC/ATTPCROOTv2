#ifndef ATMCFITTER_H
#define ATMCFITTER_H

#include "AtEvent.h"
#include "AtMCResult.h" // for AtMCResult
#include "AtPSA.h"
#include "AtRawEvent.h"

#include <TClonesArray.h> // for TClonesArray

#include <functional> // for function
#include <map>        // for map
#include <memory>     // for shared_ptr
#include <mutex>      // for mutex
#include <set>        // for set
#include <string>     // for string
#include <utility>    // for pair
#include <vector>     // for vector

class AtBaseEvent;        // lines 13-13
class AtClusterize;       // lines 15-15
class AtMap;              // lines 18-18
class AtPatternEvent;     // lines 12-12
class AtPulse;            // lines 16-16
class AtSimpleSimulation; // lines 14-14
class AtDigiPar;

namespace MCFitter {
class AtParameterDistribution;

class AtMCFitter {
protected:
   using ParamPtr = std::shared_ptr<AtParameterDistribution>;
   using SimPtr = std::shared_ptr<AtSimpleSimulation>;

   using ClusterPtr = std::shared_ptr<AtClusterize>;
   using PulsePtr = std::shared_ptr<AtPulse>;
   using MapPtr = std::shared_ptr<AtMap>;
   using PsaPtr = std::shared_ptr<AtPSA>;
   using ObjPair = std::pair<int, double>; //< Iteration number and objective function value

   std::map<std::string, ParamPtr> fParameters;

   MapPtr fMap;
   SimPtr fSim;
   ClusterPtr fClusterize;
   PulsePtr fPulse;
   PsaPtr fPSA{nullptr};

   int fNumIter{1};
   int fNumEventsToSave{10};
   bool fTimeEvent{false};
   int fNumThreads{1};

   // Things used by threads excecuting that are either expensive to create and delete
   // or unaccessable due to FairRoot design choices
   const AtPatternEvent *fCurrentEvent{nullptr};
   std::vector<PulsePtr> fThPulse;             //< Cached because it is expensive to create and delete.
   std::vector<std::unique_ptr<AtPSA>> fThPSA; //< Cached because it is expensive to create and delete.

   const AtDigiPar *fPar{nullptr};

   /** Things below here need to be written to by threads and will be locked using a shared mutex ***/
   /// Store the iteration number sorted by lowest objective funtion
   std::mutex fResultMutex;
   std::set<AtMCResult, std::function<bool(AtMCResult, AtMCResult)>> fResults;
   std::vector<AtRawEvent> fRawEventArray;
   std::vector<AtEvent> fEventArray;

public:
   AtMCFitter(SimPtr sim, ClusterPtr cluster, PulsePtr pulse);
   virtual ~AtMCFitter() = default;

   void Init();
   void SetPSA(PsaPtr psa) { fPSA = psa; }
   void Exec(const AtPatternEvent &event);

   ParamPtr GetParameter(const std::string &name) const;
   void FillResultArrays(TClonesArray &resultArray, TClonesArray &simEvent, TClonesArray &simRawEvent);
   void SetNumIter(int iter) { fNumIter = iter; }
   void SetTimeEvent(bool val) { fTimeEvent = val; }
   void SetNumEventsToSave(int num) { fNumEventsToSave = num; }
   void SetNumThreads(int num) { fNumThreads = num; }

protected:
   void RunIter(int iterNum){};
   void RunIterRange(int startIter, int numIter, AtPulse *pulse, AtPSA *psa);

   /**
    *@brief Create the parameter distributions to use for the fit.
    */
   virtual void CreateParamDistros() = 0;

   /**
    * @brief Set parameter distributions (mean/spread) from the event.
    */
   virtual void SetParamDistributions(const AtPatternEvent &event) = 0;

   /**
    * @brief This is the thing we are minimizing between events (SimEventID is index in TClonesArray)
    */
   virtual double ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID, AtMCResult &definition) = 0;

   /**
    * Simulate an event using the parameters in the passed AtMCResult class and return an array of
    * the AtMCPoints to then digitize.
    */
   virtual TClonesArray SimulateEvent(AtMCResult &definition) = 0;

   /**
    * Sample parameter distributions and constrain the system to simulate an event.
    * The parameters in AtMCResult will be used to then simulate an event.
    * This function calls Sample() on all the parameter distributions and saves them.
    */
   virtual AtMCResult DefineEvent();

   /**
    * Create the AtRawEvent and AtEvent from fSim
    * returns the index of the event in the TClonesArray
    */
   int DigitizeEvent(const TClonesArray &points, int idx, AtPulse *pulse, AtPSA *psa);
};

} // namespace MCFitter

#endif // ATMCFITTER_H
