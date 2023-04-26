#include "AtClusterizeLine.h"

#include "AtDigiPar.h"
#include "AtMCPoint.h"
#include "AtSimulatedLine.h"
#include "AtSimulatedPoint.h" // for AtSimulatedPoint

#include <FairLogger.h>

#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <TClonesArray.h>
#include <TString.h> // for operator!=, TString

#include <algorithm> // for max
#include <memory>
#include <utility> // for move

void AtClusterizeLine::GetParameters(const AtDigiPar *fPar)
{
   AtClusterize::GetParameters(fPar);
   fTBTime = fPar->GetTBTime() / 1000.; // in us
   LOG(info) << "  TB width: " << fTBTime;
}

void AtClusterizeLine::FillTClonesArray(TClonesArray &array, std::vector<SimPointPtr> &vec)
{
   for (auto &point : vec) {
      auto size = array.GetEntriesFast();
      auto *line = dynamic_cast<AtSimulatedLine *>(point.get());
      new (array[size]) AtSimulatedLine(std::move(*line)); // NO LINT
   }
}

std::vector<AtClusterize::SimPointPtr> AtClusterizeLine::processPoint(AtMCPoint &mcPoint, int pointID)
{
   if (mcPoint.GetVolName() != "drift_volume") {
      LOG(info) << "Skipping point " << pointID << ". Not in drift volume.";
      return {};
   }

   auto trackID = mcPoint.GetTrackID();
   XYZPoint currentPoint = getCurrentPointLocation(mcPoint); // [mm, mm, us]

   // If it is a new track entering the volume or no energy was deposited
   // record its location and the new track ID
   if (mcPoint.GetEnergyLoss() == 0 || fTrackID != trackID) {
      fPrevPoint = currentPoint;
      fTrackID = mcPoint.GetTrackID();
      return {};
   }

   std::vector<SimPointPtr> ret;
   ret.push_back(std::make_unique<AtSimulatedLine>());

   auto simLine = dynamic_cast<AtSimulatedLine *>(ret.back().get());

   simLine->SetMCPointID(pointID);
   simLine->SetMCEventID(mcPoint.GetEventID());
   simLine->SetClusterID(0);

   simLine->SetInitialPosition(fPrevPoint.x(), fPrevPoint.y(), fPrevPoint.z());
   simLine->SetFinalPosition(currentPoint.x(), currentPoint.y(), currentPoint.z());

   simLine->SetCharge(getNumberOfElectronsGenerated(mcPoint));

   simLine->SetTransverseDiffusion(getTransverseDiffusion(simLine->GetPosition().z()));
   simLine->SetLongitudinalDiffusion(getLongitudinalDiffusion(simLine->GetPosition().z()));

   fPrevPoint = currentPoint;
   return ret;
}
