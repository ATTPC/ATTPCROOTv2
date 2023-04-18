#ifndef ATMCFITTER_H
#define ATMCFITTER_H

#include "AtMCResult.h" // for AtMCResult

#include <TClonesArray.h> // for TClonesArray

#include <functional>     // for function
#include <map>            // for map
#include <memory>         // for shared_ptr
#include <set>            // for set
#include <string>         // for string
#include <utility>        // for pair
class AtBaseEvent;        // lines 13-13
class AtClusterize;       // lines 15-15
class AtMap;              // lines 18-18
class AtPSA;              // lines 19-19
class AtPatternEvent;     // lines 12-12
class AtPulse;            // lines 16-16
class AtSimpleSimulation; // lines 14-14

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

   /// Store the iteration number sorted by lowest objective funtion
   std::set<AtMCResult, std::function<bool(AtMCResult, AtMCResult)>> fResults;
   TClonesArray fRawEventArray;
   TClonesArray fEventArray;

public:
   AtMCFitter(SimPtr sim, ClusterPtr cluster, PulsePtr pulse);
   virtual ~AtMCFitter() = default;

   void Init();
   void SetPSA(PsaPtr psa) { fPSA = psa; }
   void Exec(const AtPatternEvent &event);

   ParamPtr GetParameter(const std::string &name) const;
   TClonesArray &GetEventArray() { return fEventArray; }
   TClonesArray &GetRawEventArray() { return fRawEventArray; }
   void FillResultArray(TClonesArray &resultArray) const;
   void SetNumIter(int iter) { fNumIter = iter; }

protected:
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
   virtual double ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID) = 0;

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
   int DigitizeEvent(const TClonesArray &points);
};

} // namespace MCFitter

#endif // ATMCFITTER_H
