
#include "AtSimpleSimulation.h"

#include "AtELossModel.h"
#include "AtMCPoint.h"
#include "AtSpaceChargeModel.h" // for AtSpaceChargeModel

#include <FairLogger.h>
#include <FairRootManager.h>

#include <TClonesArray.h> // for TClonesArray
#include <TGeoManager.h>
#include <TGeoNode.h>
#include <TGeoVolume.h>
#include <TObject.h> // for TObject

#include <cmath>     // for sqrt
#include <stdexcept> // for invalid_argument
#include <utility>   // for pair

thread_local TClonesArray AtSimpleSimulation::fMCPoints("AtMCPoint");
thread_local int AtSimpleSimulation::fTrackID = 0;

using SpaceChargeModel = std::shared_ptr<AtSpaceChargeModel>;
using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

AtSimpleSimulation::AtSimpleSimulation(std::string geoFile)
{
   TGeoManager *geo = TGeoManager::Import(geoFile.c_str());

   if (gGeoManager == nullptr)
      LOG(fatal) << "Failed to load geometry file " << geoFile << " " << geo;
}
AtSimpleSimulation::AtSimpleSimulation()
{
   if (gGeoManager == nullptr)
      LOG(fatal) << "No geometry file loaded!";
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
   {
      std::lock_guard<std::mutex> lock(fGeoMutex);
      TGeoNode *node = gGeoManager->FindNode(pointCm.X(), pointCm.Y(), pointCm.Z());
      if (node == nullptr) {
         return nullptr;
      }
      return node->GetVolume();
   }
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

std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::SimulateParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                     std::function<bool(XYZPoint, PxPyPzEVector)> func)
{
   auto modelIt = fModels.find({A, Z});
   if (modelIt == fModels.end())
      throw std::invalid_argument("Missing energy loss model for Z:" + std::to_string(Z) + " A:" + std::to_string(A));
   if (!IsInVolume("drift_volume", iniPos))
      throw std::invalid_argument("Position of particle is not in active volume but is in " + GetVolumeName(iniPos));

   return SimulateParticle(modelIt->second, iniPos, iniMom, func);
}

std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::SimulateParticle(ModelPtr model, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                     std::function<bool(XYZPoint, PxPyPzEVector)> func)
{
   // This is a new track
   fTrackID++;

   auto pos = iniPos;
   auto mom = iniMom;
   double length = 0;

   // Go until we exit the volume or the KE is less than 1keV
   while (IsInVolume("drift_volume", pos) && mom.E() - mom.M() > 1e-3 && func(pos, mom)) {

      if (isnan(pos.X()) || isnan(mom.X())) {
         LOG(error) << "Failed to simulate a point with nan!";
         return {{0, 0, 0}, {0, 0, 0, 0}};
      }
      // Direction particle is traveling
      auto dir = mom.Vect().Unit();

      // Get the energy loss from the model
      double KE = mom.E() - mom.M();
      double eLoss = model->GetEnergyLoss(KE, fDistStep);

      // Update the momentum from the energy loss model. Assume the energy loss does not change
      // the direction of the particle.
      // newMom (x/y/z) =
      auto E = mom.E() - eLoss;
      double p = sqrt(E * E - mom.M2());
      mom.SetPxPyPzE(dir.X() * p, dir.Y() * p, dir.Z() * p, E);

      LOG(debug) << mom << " " << mom.M() << " " << iniMom.M();

      pos += dir * fDistStep;
      length += fDistStep;
      AddHit(eLoss, pos, mom, length);
   }

   return {pos, mom};
}

void AtSimpleSimulation::NewEvent()
{
   fMCPoints.Clear();
   fTrackID = 0;
}

/**
 * Units are mm, Mev, and Mev/c.
 */
void AtSimpleSimulation::AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length)
{
   LOG(debug) << "Adding a hit at element " << fMCPoints.GetEntriesFast() << " in TClonesArray.";

   auto *mcPoint = dynamic_cast<AtMCPoint *>(fMCPoints.ConstructedAt(fMCPoints.GetEntriesFast(), "C"));

   mcPoint->SetTrackID(fTrackID);
   mcPoint->SetLength(length / 10.);      // Convert to cm
   mcPoint->SetEnergyLoss(ELoss / 1000.); // Convert to GeV
   mcPoint->SetVolName("drift_volume");

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
   // mcPoint->Print(nullptr);
}

void AtSimpleSimulation::RegisterBranch(std::string branchName, bool perc)
{
   auto ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "The IO manager was not instatiated before attempting to simulate an event.";
      return;
   }

   ioMan->Register(branchName.c_str(), "AtTPC", &fMCPoints, perc);
}
