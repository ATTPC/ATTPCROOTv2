#include "AtTestSimulation.h"

#include "AtSimpleSimulation.h"

#include <Math/Point3D.h>
#include <Math/Vector3D.h>
using namespace ROOT::Math;

InitStatus AtTestSimulation::Init()
{
   fSimulation->Init();

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
