
/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/

// -------------------------------------------------------------------------
// -----                    AtCave  file                               -----
// -----                Created 26/03/14  by M. Al-Turany              -----
// -------------------------------------------------------------------------
#include "AtCave.h"

#include <FairModule.h>
#include <Rtypes.h>

#include "AtGeoCave.h"        // for AtGeoCave
#include <FairGeoInterface.h> // for FairGeoInterface
#include <FairGeoLoader.h>    // for FairGeoLoader
#include <TString.h>          // for TString

ClassImp(AtCave);

void AtCave::ConstructGeometry()
{
   FairGeoLoader *loader = FairGeoLoader::Instance();
   FairGeoInterface *GeoInterface = loader->getGeoInterface();
   auto *MGeo = new AtGeoCave(); // NOLINT I have no idea who owns this...
   MGeo->setGeomFile(GetGeometryFileName());
   GeoInterface->addGeoModule(MGeo);
   Bool_t rc = GeoInterface->readSet(MGeo);
   if (rc) {
      MGeo->create(loader->getGeoBuilder());
   }
}
AtCave::AtCave() : FairModule() {}

AtCave::~AtCave() = default;
AtCave::AtCave(const char *name, const char *Title) : FairModule(name, Title)
{
   world[0] = 0;
   world[1] = 0;
   world[2] = 0;
}
