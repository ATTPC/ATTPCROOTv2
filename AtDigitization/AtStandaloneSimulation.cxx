#include "AtStandaloneSimulation.h"

#include "AtMCPoint.h"
#include "AtSimpleSimulation.h"
#include "AtSpaceChargeModel.h"

#include <FairLogger.h>
#include <FairRootManager.h>

#include <TClonesArray.h>

#include <stdexcept>

thread_local TClonesArray AtStandaloneSimulation::fMCPoints("AtMCPoint");
thread_local int AtStandaloneSimulation::fTrackID = 0;

using XYZPoint = ROOT::Math::XYZPoint;
using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

AtStandaloneSimulation::AtStandaloneSimulation(std::unique_ptr<AtSimpleSimulation> engine)
   : fEngine(std::move(engine))
{
}

void AtStandaloneSimulation::NewEvent()
{
   fMCPoints.Clear();
   fTrackID = 0;
}

void AtStandaloneSimulation::RegisterBranch(std::string branchName, bool pers)
{
   auto ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "The IO manager was not instantiated before attempting to simulate an event.";
      return;
   }

   ioMan->Register(branchName.c_str(), "AtTPC", &fMCPoints, pers);
}

std::pair<XYZPoint, PxPyPzEVector>
AtStandaloneSimulation::SimulateParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                         std::function<bool(XYZPoint, PxPyPzEVector)> func)
{
   if (fEngine->GetVolumeNameAt(iniPos) != fVolumeName)
      throw std::invalid_argument("Position of particle is not in active volume but is in " +
                                  fEngine->GetVolumeNameAt(iniPos));

   ++fTrackID;
   const auto &volName = fVolumeName;

   // The callback records hits while inside the volume, forwards to the user's callback,
   // and stops transport when the particle exits. All checks use postPosition so hits
   // are recorded at the step endpoint (matching the Geant4 convention).
   return fEngine->TransportParticle(
      Z, A, iniPos, iniMom, [this, &func, &volName](const AtSimpleSimulation::TransportStep &step) {
         if (fEngine->GetVolumeNameAt(step.postPosition) == volName)
            AddHit(step.energyLoss, step.postPosition, step.postMomentum, step.length);

         // Call user's callback
         if (!func(step.postPosition, step.postMomentum))
            return false;

         // Stop transport if the particle has exited the configured volume
         if (fEngine->GetVolumeNameAt(step.postPosition) != volName)
            return false;

         return true;
      });
}

void AtStandaloneSimulation::AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length)
{
   LOG(debug) << "Adding a hit at element " << fMCPoints.GetEntriesFast() << " in TClonesArray.";

   auto *mcPoint = dynamic_cast<AtMCPoint *>(fMCPoints.ConstructedAt(fMCPoints.GetEntriesFast(), "C"));

   mcPoint->SetTrackID(fTrackID);
   mcPoint->SetLength(length / 10.);      // Convert to cm
   mcPoint->SetEnergyLoss(ELoss / 1000.); // Convert to GeV
   mcPoint->SetVolName(fVolumeName.c_str());

   if (fSCModel) {
      // In the simulation z = 0 is the window and z=1000 is the pad plane.
      // In the data analysis that is flipped, so we must adjust the z value, apply SC and move back
      auto posExpCoord = pos;
      posExpCoord.SetZ(1000 - pos.Z());
      auto corrExpCoord = fSCModel->ApplySpaceCharge(posExpCoord);
      corrExpCoord.SetZ(1000 + corrExpCoord.Z());
      mcPoint->SetPosition(corrExpCoord / 10.);
   } else
      mcPoint->SetPosition(pos / 10.);       // Convert to cm
   mcPoint->SetMomentum(mom.Vect() / 1000.); // Convert to GeV/c
}
