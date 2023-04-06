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
   XYZVector momDir = XYZVector(0, 0, 1).Unit();
   // Assume Pb208 (mass is 207.93 amu)
   double m = 207.93 * 931.4936; // Mev
   // Assume initial KE is 35 MeV/u
   double E = m + 35 * 207.93;
   // Grab the momentum
   double p = sqrt(E * E - m * m);
   PxPyPzEVector mom(0, 0, p, E);
   fSimulation->SimulateParticle(82, 208, pos, mom);
}

ClassImp(AtTestSimulation);
