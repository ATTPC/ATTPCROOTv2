/* Copyright 2008-2010, Technische Universitaet Muenchen,
   Authors: Christian Hoeppner & Sebastian Neubert & Johannes Rauch
   This file is part of GENFIT.
   GENFIT is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   GENFIT is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public License
   along with GENFIT.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
Rearranged by: Genie Jhang (geniejhang@nuclear.korea.ac.kr, Korea University)
Adapted to AtTPCROOTv2 by Yassid Ayyad ayyadlim@frib.msu.edu
*/

#include "AtSpacePointMeasurement.h"

#include "AtHitCluster.h"

#include <Math/Point3D.h>
#include <TMatrixDSymfwd.h>
#include <TMatrixDfwd.h>
#include <TMatrixT.h>
#include <TMatrixTSym.h>
#include <TVectorDfwd.h>
#include <TVectorT.h>
#include <TrackCandHit.h>

#include <SpacepointMeasurement.h>

namespace genfit {
class AbsMeasurement;
} // namespace genfit

ClassImp(genfit::AtSpacepointMeasurement)

   namespace genfit
{

   AtSpacepointMeasurement::AtSpacepointMeasurement() : SpacepointMeasurement() {}

   AtSpacepointMeasurement::AtSpacepointMeasurement(const AtHitCluster *detHit, const TrackCandHit *hit)
      : SpacepointMeasurement(), fCharge(detHit->GetCharge())
   {
      auto pos = detHit->GetPosition();
      TMatrixD mat = detHit->GetCovMatrix();

      rawHitCoords_(0) = pos.X() / 10.;
      rawHitCoords_(1) = pos.Y() / 10.;
      rawHitCoords_(2) = pos.Z() / 10.;

      TMatrixDSym cov(3);

      /*cov(0,0) = detHit -> GetDx();
      cov(1,1) = detHit -> GetDy();
      cov(2,2) = detHit -> GetDz();*/  //TODO: Compute position variance

      cov(0, 1) = 0.0;
      cov(1, 2) = 0.0;
      cov(2, 0) = 0.0;

      // Forced covariance matrix to be constant. Need to study later.
      cov(0, 0) = 1.0 / 1.0;  // 0.2
      cov(1, 1) = 1.0 / 1.0;  // 0.2
      cov(2, 2) = 1.28 / 1.0; // 1.28

      rawHitCov_ = cov;
      detId_ = hit->getDetId();
      hitId_ = hit->getHitId();

      // std::cout<<" AtSpacepointMeasurement::AtSpacepointMeasurement "<<"\n";
      // std::cout<<rawHitCoords_(0)<<"	"<<rawHitCoords_(1)<<"	  "<<rawHitCoords_(2)<<"	"<<fCharge<<" "<<detId_<<"
      // "<<hitId_<<"\n";

      this->initG();
   }

   AbsMeasurement *AtSpacepointMeasurement::clone() const
   {
      return new AtSpacepointMeasurement(*this);
   }

   Double_t AtSpacepointMeasurement::GetCharge()
   {
      return fCharge;
   }

} /* End of namespace genfit */
