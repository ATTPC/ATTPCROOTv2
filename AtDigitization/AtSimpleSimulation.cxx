
#include "AtSimpleSimulation.h"

#include "AtMCPoint.h"

#include <FairLogger.h>
#include <FairRootManager.h>

#include <TGeoManager.h>
#include <TGeoNode.h>
#include <TGeoVolume.h>

AtSimpleSimulation::AtSimpleSimulation(std::string geoFile)
{
   TGeoManager *geo = TGeoManager::Import(geoFile.c_str());

   if (gGeoManager == nullptr)
      LOG(fatal) << "Failed to load geometry file " << geoFile << " " << geo;
}
bool AtSimpleSimulation::ParticleID::operator<(const ParticleID &other) const
{
   if (A < other.A) {
      return true;
   } else if (A > other.A) {
      return false;
   } else {
      return Z < other.Z;
   }
}

/// Takes position in mm
TGeoVolume *AtSimpleSimulation::GetVolume(const XYZPoint &point)
{
   auto pointCm = point / 10.;
   TGeoNode *node = gGeoManager->FindNode(pointCm.X(), pointCm.Y(), pointCm.Z());
   if (node == nullptr) {
      return nullptr;
   }
   return node->GetVolume();
}

bool AtSimpleSimulation::IsInVolume(const std::string &volName, const XYZPoint &point)
{

   TGeoVolume *volume = GetVolume(point);
   if (volume == nullptr || volName != std::string(volume->GetName())) {
      return false;
   }

   return true;
}

std::string AtSimpleSimulation::GetVolumeName(const XYZPoint &point)
{
   TGeoVolume *volume = GetVolume(point);
   if (volume == nullptr) {
      return "";
   }
   return volume->GetName();
}

void AtSimpleSimulation::AddModel(int Z, int A, ModelPtr model)
{
   ParticleID id = {
      .A = A,
      .Z = Z,
   };

   fModels[id] = model;
}

void AtSimpleSimulation::SimulateParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom)
{
   auto modelIt = fModels.find({A, Z});
   if (modelIt == fModels.end())
      throw std::invalid_argument("Missing energy loss model for Z:" + std::to_string(Z) + " A:" + std::to_string(A));
   if (!IsInVolume("drift_volume", iniPos))
      throw std::invalid_argument("Position of particle is not in active volume but is in " + GetVolumeName(iniPos));

   SimulateParticle(modelIt->second, iniPos, iniMom);
}

void AtSimpleSimulation::SimulateParticle(ModelPtr model, const XYZPoint &iniPos, const PxPyPzEVector &iniMom)
{
   // This is a new track
   fTrackID++;

   auto pos = iniPos;
   auto mom = iniMom;
   auto m = iniMom.M();

   double length = 0;

   while (IsInVolume("drift_volume", pos)) {

      // Direction particle is traveling
      auto dir = iniMom.Vect().Unit();

      // Get the energy loss from the model
      double eLoss = 1;

      // Update the momentum from the energy loss model. Assume the energy loss does not change
      // the direction of the particle.
      // newMom (x/y/z) =
      auto E = mom.E() - eLoss;
      double p = sqrt(E * E - mom.M2());
      mom.SetPxPyPzE(dir.X() * p, dir.Y() * p, dir.Z() * p, E);

      LOG(debug) << mom << " " << mom.M() << " " << iniMom.M();

      pos += iniMom.Vect().Unit() * fDistStep;
      length += fDistStep;
      AddHit(eLoss, pos, mom, length);
   }
}

void AtSimpleSimulation::NewEvent()
{
   fMCPoints->Clear();
   fTrackID = 0;
}

/**
 * Units are mm, Mev, and Mev/c.
 */
void AtSimpleSimulation::AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length)
{
   LOG(info) << "Adding a hit at element " << fMCPoints->GetEntriesFast() << " in TClonesArray.";

   auto *mcPoint = dynamic_cast<AtMCPoint *>(fMCPoints->ConstructedAt(fMCPoints->GetEntriesFast(), "C"));

   mcPoint->SetTrackID(fTrackID);
   mcPoint->SetLength(length / 10.);      // Convert to cm
   mcPoint->SetEnergyLoss(ELoss / 1000.); // Convert to GeV
   mcPoint->SetVolName("drift_volume");

   // mcPoint->fEnergyIni = Eini;
   // mcPoint->fAiso = A;
   // mcPoint->fZiso = Z;
   mcPoint->SetPosition(pos / 10.);          // Convert to cm
   mcPoint->SetMomentum(mom.Vect() / 1000.); // Convert to GeV/c
   mcPoint->Print(nullptr);
}

void AtSimpleSimulation::Init(std::string branchName)
{
   auto ioMan = FairRootManager::Instance();
   if (ioMan == nullptr)
      LOG(fatal) << "The IO manager was not instatiated before attempting to simulate an event.";

   fMCPoints = ioMan->Register(branchName, "AtMCPoint", "AtTPC", true);
}
