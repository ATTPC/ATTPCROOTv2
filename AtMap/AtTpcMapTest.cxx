#include "AtTpcMap.h"

#include <FairLogger.h>

#include <TString.h>

#include <gtest/gtest.h>

TEST(AtTpcMapTest, TEST_InhibitBeamPads_RCNP)
{
   // Construct an AtTpcMap object, parse the RCNP mapping and parse the beam pad list.
   TString dir = getenv("VMCWORKDIR");
   TString scriptFile = "rcnp_map_size.xml";
   TString mapDir = dir + "/scripts/" + scriptFile;
   TString beamPadsFile = "BeamPads_RCNP.csv";
   TString beamPadsDir = dir + "/scripts/" + beamPadsFile;
   AtTpcMap *map = new AtTpcMap();
   map->ParseXMLMap(mapDir.Data());
   map->GeneratePadPlane();
   Int_t inhibitedPadsCount = map->InhibitBeamPads(beamPadsDir);

   // Check all pads and count how many are actually inhibited.
   Int_t actuallyInhibitedPadsCount{};
   for (int i = 0; i < map->GetNumPads(); i++)
      if (map->IsInhibited(i) == AtMap::InhibitType::kXTalk)
         actuallyInhibitedPadsCount++;

   EXPECT_EQ(inhibitedPadsCount, actuallyInhibitedPadsCount);
   ASSERT_TRUE(inhibitedPadsCount > 0);
}

TEST(AtTpcMapTest, TEST_InhibitBeamPads_e23031)
{
   // Construct an AtTpcMap object, parse the e23031 mapping and parse the beam pad list.
   TString dir = getenv("VMCWORKDIR");
   TString scriptFile = "e23031_pad_map_size.xml";
   TString mapDir = dir + "/scripts/" + scriptFile;
   TString beamPadsFile = "BeamPads_e23031.csv";
   TString beamPadsDir = dir + "/scripts/" + beamPadsFile;
   AtTpcMap *map = new AtTpcMap();
   map->ParseXMLMap(mapDir.Data());
   map->GeneratePadPlane();
   Int_t inhibitedPadsCount = map->InhibitBeamPads(beamPadsDir);

   // Check all pads and count how many are actually inhibited.
   Int_t actuallyInhibitedPadsCount{};
   for (int i = 0; i < map->GetNumPads(); i++)
      if (map->IsInhibited(i) == AtMap::InhibitType::kXTalk)
         actuallyInhibitedPadsCount++;

   EXPECT_EQ(inhibitedPadsCount, actuallyInhibitedPadsCount);
   ASSERT_TRUE(inhibitedPadsCount > 0);
}
