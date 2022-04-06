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

// Modified for SpiRITROOT by Genie Jhang (2015/06/15)
// Adapted to ATTPCROOTv2 Yassid Ayyad (03/17/2021) ayyadlim@frib.msu.edu

#ifndef ATSPACEPOINTMEASUREMENT
#define ATSPACEPOINTMEASUREMENT

#include <Rtypes.h>

#include "SpacepointMeasurement.h"

class AtHitCluster;
class TBuffer;
class TClass;
class TMemberInspector;

namespace genfit {
class AbsMeasurement;
class TrackCandHit;

/** @brief Class for a spacepoint measurement which can be created
 *         from STHitCluster via the MeasurementFactory.
 *
 *  @author Johannes Rauch  (Technische Universit&auml;t M&uuml;nchen, original author)
 *  @author Genie Jhang (Korea University, modifier for S\piRITTPC)
 *  @author Yassid Ayyad (FRIB, Adapted to AtTPCROOTv2)
 */
class AtSpacepointMeasurement : public SpacepointMeasurement {
public:
   AtSpacepointMeasurement();
   AtSpacepointMeasurement(const AtHitCluster *detHit, const TrackCandHit *hit);

   virtual AbsMeasurement *clone() const;

   Double_t GetCharge();

private:
   Double_t fCharge;

   ClassDef(AtSpacepointMeasurement, 1)
};

} /* End of namespace genfit */

#endif
