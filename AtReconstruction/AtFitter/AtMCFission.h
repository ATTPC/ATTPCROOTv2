#ifndef ATMCFISSION_H
#define ATMCFISSION_H

#include "AtMCFitter.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector

#include <vector>
namespace MCFitter {

class AtMCFission : public AtMCFitter {
protected:
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;

   int fZcn{85};  /// Z of the compound nucleous
   int fAcn{204}; /// A of the compound nucleous

public:
   AtMCFission(SimPtr sim, ClusterPtr cluster, PulsePtr pulse) : AtMCFitter(sim, cluster, pulse) {}
   virtual ~AtMCFission() = default;
   void SetCN(int Z, int A);

protected:
   virtual void CreateParamDistros() override;
   virtual void SetParamsFromEvent(const AtPatternEvent &event) override;
   virtual double ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID) override;
   virtual void SimulateEvent() override;

   XYZPoint calcualteVetrex(const std::vector<XYZVector> &lineStart, const std::vector<XYZVector> &lineStep);
};

} // namespace MCFitter

#endif // ATMCFISSION_H
