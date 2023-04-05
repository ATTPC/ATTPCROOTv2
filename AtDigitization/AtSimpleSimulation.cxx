
#include "AtSimpleSimulation.h"

#include "AtMCPoint.h"

#include <FairLogger.h>
#include <FairRootManager.h>

#include <TGeoManager.h>
#include <TGeoNode.h>
#include <TGeoVolume.h>

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

bool AtSimpleSimulation::IsInVolume(const std::string &volName, const XYZPoint &point)
{
   TGeoNode *node = gGeoManager->FindNode(point.X(), point.Y(), point.Z());
   if (node == nullptr) {
      return false;
   }

   TGeoVolume *volume = node->GetVolume();
   if (volume == nullptr || volName != std::string(volume->GetName())) {
      return false;
   }

   return true;
}

std::string AtSimpleSimulation::GetVolumeName(const XYZPoint &point)
{
   TGeoNode *node = gGeoManager->FindNode(point.X(), point.Y(), point.Z());
   if (node == nullptr) {
      return "";
   }
   TGeoVolume *volume = node->GetVolume();
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

void AtSimpleSimulation::SimulateParticle(int Z, int A, const XYZPoint &iniPos, const XYZVector &iniMom)
{
   auto modelIt = fModels.find({Z, A});
   if (modelIt == fModels.end())
      throw std::invalid_argument("Missing energy loss model for Z:" + std::to_string(Z) + " A:" + std::to_string(A));
   if (!IsInVolume("drift_vol", iniPos))
      throw std::invalid_argument("Position of particle is not in active volume");

   SimulateParticle(modelIt->second, iniPos, iniMom);
}

void AtSimpleSimulation::SimulateParticle(ModelPtr model, const XYZPoint &iniPos, const XYZVector &iniMom)
{
   // This is a new track
   fTrackID++;
   auto pos = iniPos;
   auto mom = iniMom;
   double length = 0;

   while (IsInVolume("drift_vol", pos)) {
      double eLoss;
      // Update the momentum from the energy loss model

      pos += iniMom.Unit() * fDistStep;
      length += fDistStep;
      AddHit(eLoss, pos, mom, length);
   }
}

AtSimpleSimulation::AtSimpleSimulation(bool isPersistant, std::string branchName)
{
   auto ioMan = FairRootManager::Instance();
   if (ioMan == nullptr)
      LOG(error) << "The IO manager was not instatiated before AtSimpleSimulation.";
   else
      ioMan->Register(branchName.c_str(), "AtTpc", &fMCPoints, isPersistant);
}

void AtSimpleSimulation::NewEvent()
{
   fMCPoints.Clear();
   fTrackID = 0;
}

void AtSimpleSimulation::AddHit(double ELoss, const XYZPoint &pos, const XYZVector &mom, double length)
{
   auto *mcPoint = dynamic_cast<AtMCPoint *>(fMCPoints.ConstructedAt(fMCPoints.GetEntriesFast(), "C"));

   mcPoint->SetTrackID(fTrackID);
   mcPoint->SetLength(length);
   mcPoint->SetEnergyLoss(ELoss);

   // mcPoint->fEnergyIni = Eini;
   // mcPoint->fAiso = A;
   // mcPoint->fZiso = Z;

   mcPoint->SetPosition(pos);
   mcPoint->SetMomentum(mom);
}
