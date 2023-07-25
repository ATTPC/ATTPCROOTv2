#ifndef ATMCFITTER_H
#define ATMCFITTER_H

#include "AtEvent.h"
#include "AtMCResult.h" // for AtMCResult
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
class AtPSA;

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
   int fNumRounds{1};
   int fNumEventsToSave{10};
   bool fTimeEvent{false};
   int fNumThreads{1};

   // Things used by threads excecuting that are either expensive to create and delete
   // or unaccessable due to FairRoot design choices
   const AtPatternEvent *fCurrentEvent{nullptr};
   std::vector<PulsePtr> fThPulse; //< Cached because it is expensive to create and delete.
   const AtDigiPar *fPar{nullptr}; //<Tracked sepretly because FairRun::Instance is thread local.

   // These are not locked by the mutex since we ensure no realloc of the vector is happening and
   // each thread is accessing a discrete subset of the elements of these vectors
   std::vector<AtRawEvent> fRawEventArray;
   std::vector<AtEvent> fEventArray;

   /** Things below here need to be written to by threads and will be locked using a shared mutex ***/
   /// Store the iteration number sorted by lowest objective funtion
   std::mutex fResultMutex;
   std::set<AtMCResult, std::function<bool(AtMCResult, AtMCResult)>> fResults;

public:
   AtMCFitter(SimPtr sim, ClusterPtr cluster, PulsePtr pulse);
   virtual ~AtMCFitter() = default;

   void Init();
   void SetPSA(PsaPtr psa) { fPSA = psa; }
   void Exec(const AtPatternEvent &event);

   ParamPtr GetParameter(const std::string &name) const;
   void FillResultArrays(TClonesArray &resultArray, TClonesArray &simEvent, TClonesArray &simRawEvent);
   void SetNumIter(int iter) { fNumIter = iter; }

   /// Set number of times to run fNumIter iterations and then re-center and truncate the parameter space.
   void SetNumRounds(int rounds) { fNumRounds = rounds; }
   void SetTimeEvent(bool val) { fTimeEvent = val; }
   void SetNumEventsToSave(int num) { fNumEventsToSave = num; }
   void SetNumThreads(int num);

protected:
   void RunRound();
   void RunIterRange(int startIter, int numIter, AtPulse *pulse);

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
    * Recenter the parameter distributions around the best result and truncate the parameter space.
    */
   virtual void RecenterParamDistributions();

   /**
    * Create the AtRawEvent and AtEvent from fSim
    * returns the index of the event in the TClonesArray
    */
   int DigitizeEvent(const TClonesArray &points, int idx, AtPulse *pulse);
};

} // namespace MCFitter

#endif // ATMCFITTER_H
