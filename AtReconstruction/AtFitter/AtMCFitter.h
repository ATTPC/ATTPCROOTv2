#ifndef ATMCFITTER_H
#define ATMCFITTER_H

#include "AtParameterDistribution.h"

#include <map>
#include <memory>
#include <string>

class AtPatternEvent;
class AtBaseEvent;
class AtSimpleSimulation;
class AtClusterizeTask;
class AtPulseTask;
class FairTask;

namespace MCFitter {

class AtMCFitter {
protected:
   using ParamPtr = std::shared_ptr<AtParameterDistribution>;
   using SimPtr = std::shared_ptr<AtSimpleSimulation>;
   using ClusterPtr = std::shared_ptr<AtClusterizeTask>;
   using PulsePtr = std::shared_ptr<AtPulseTask>;

   std::map<std::string, ParamPtr> fParameters;

   SimPtr fSim{nullptr};
   ClusterPtr *fClusterize;
   PulsePtr *fPulse;
   std::vector<FairTask *> fTasks;

public:
   AtMCFitter();
   AtMCFitter(AtClusterizeTask *cluster, AtPulseTask *pulse);
   virtual ~AtMCFitter() = default;

   void Init();
   // void AddParameter(const std::string &name, ParamPtr parameter);
   ParamPtr GetParameter(const std::string &name) const;

protected:
   /**
    *@brief Create the parameter distributions to use for the fit.
    */
   virtual void CreateParamDistros() = 0;

   /**
    * @brief Set parameter distributions (mean/spread) from the event.
    */
   virtual void SetParamsFromEvent(const AtPatternEvent &event) = 0;

   /**
    * @brief This is the thing we are minimizing between events
    */
   virtual double ObjectiveFunction(const AtBaseEvent &expEvent, const AtBaseEvent &simEvent) = 0;

   /**
    * Simulate an event and save it in the
    */
   virtual void SimulateEvent() = 0;
};

} // namespace MCFitter

#endif // ATMCFITTER_H
