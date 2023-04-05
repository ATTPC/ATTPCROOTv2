#include "AtTestSimulation.h"

#include <Math/Point3D.h>
#include <Math/Vector3D.h>

using namespace ROOT::Math;

AtTestSimulation::AtTestSimulation() {}

InitStatus AtTestSimulation::Init()
{
   fSimulation = std::make_unique<AtSimpleSimulation>();
   return kSUCCESS;
}

void AtTestSimulation::Exec(Option_t *)
{
   fSimulation->NewEvent();

   XYZPoint pos(0, 0, 500);
   XYZVector mom(0, 0, 1);

   fSimulation->SimulateParticle(82, 208, pos, mom);
}

ClassImp(AtTestSimulation);
