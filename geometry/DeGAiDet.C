/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
// Note: In root all sizes are given in cm

#include "TFile.h"
#include "TGeoCompositeShape.h"
#include "TGeoManager.h"
#include "TGeoMaterial.h"
#include "TGeoMatrix.h"
#include "TGeoMedium.h"
#include "TGeoPgon.h"
#include "TGeoVolume.h"
#include "TList.h"
#include "TROOT.h"
#include "TMath.h"
#include "TString.h"
#include "TSystem.h"

#include <iostream>

// Name of geometry version and output file
const TString geoVersion = "DeGAiDet";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "GADGET_5IsoAr_800";
const TString CylinderVolumeMedium = "steel";
const TString MediumVacuum = "vacuum4";
const TString WindowMedium = "kapton";
const TString MicromegasMedium = "G10";
const TString DeGAiMatter = "germanium";
const TString ShellMatter = "aluminium";
// Detector Dimensions (cm)
const Float_t tpc_diameter_in = 15.4051;
const Float_t tpc_diameter_out = 16.51;
const Float_t drift_length = 40.;
const Float_t upstream_ring_dia_in = 11.684;
const Float_t upstream_ring_dia_out = tpc_diameter_out;
const Float_t upstream_ring_length = 2.6162 + 0.9398;
const Float_t downstream_ring_dia_in = 15.4051;
const Float_t downstream_ring_dia_out = 22.86;
const Float_t downstream_ring_length = 1.5748;
const Float_t upstream_cap_dia_in = 5.08;
const Float_t upstream_cap_dia_out = tpc_diameter_out;
const Float_t upstream_cap_length = 1.7272;
const Float_t downstream_cap_dia_in = 18.161;
const Float_t downstream_cap_dia_out = 22.86;
const Float_t downstream_cap_length = 2.6162;
const Float_t window_dia_in = 5.08;
const Float_t window_dia_out = 8.4074;
const Float_t window_length = 0.8636;
const Float_t micromegas_dia_in = 0;
const Float_t micromegas_dia_out = 22.86;
const Float_t micromegas_length = 0.2794;
const Float_t micromegas_ring_dia_in = 0;
const Float_t micromegas_ring_dia_out = 22.86;
const Float_t micromegas_ring_length = 0.635;
const Float_t cathode_dia_in = 5.5118;
const Float_t cathode_dia_out = 12.1844;
const Float_t cathode_length = 1.1016;
const Float_t cathode_mount_dia_in = 6.096;
const Float_t cathode_mount_dia_out = 14.8844;
const Float_t cathode_mount_length = 0.9525;
const Float_t cage_dia_in = 12.2074;
const Float_t cage_dia_out = (12.2074 + (12.2174 - 12.2074) * 0.8);
const Float_t cage_length = 38.0956;
const Float_t cage_support_shift = 2.71655;
const Float_t cage_support_dia_in = 12.2174;
const Float_t cage_support_dia_out = 13.8684;
const Float_t cage_support_length = 38.6867;


// DeGAi
const Float_t Box_Width = 10.1;
const Float_t Box_length = 7.0;
const Float_t Crystal_Diameter = 5.0;
const Float_t Crystal_Length = 7.0;
const Float_t Hole_Diameter = 1.0;
const Float_t Hole_Length = 4.0;
// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void DeGAiDet()
{
   // Load the necessary FairRoot libraries
   gSystem->Load("libGeoBase");
   gSystem->Load("libParBase");
   gSystem->Load("libBase");

   // Load needed material definition from media.geo file
   create_materials_from_media_file();

   // Get the GeoManager for later usage
   gGeoMan = (TGeoManager *)gROOT->FindObject("FAIRGeom");
   gGeoMan->SetVisLevel(7);

   // Create the top volume
   TGeoVolume *top = new TGeoVolumeAssembly("TOP");
   gGeoMan->SetTopVolume(top);

   TGeoMedium *gas = gGeoMan->GetMedium(MediumVacuum);
   TGeoVolume *tpcvac = new TGeoVolumeAssembly(geoVersion);
   tpcvac->SetMedium(gas);
   top->AddNode(tpcvac, 1);

   gModules = create_detector();

   cout << "Voxelizing." << endl;
   top->Voxelize("");
   gGeoMan->CloseGeometry();

   gGeoMan->CheckOverlaps(0.001);
   gGeoMan->PrintOverlaps();
   gGeoMan->Test();

   TFile *outfile = new TFile(FileName, "RECREATE");
   top->Write();
   outfile->Close();

   TFile *outfile1 = new TFile(FileName1, "RECREATE");
   gGeoMan->Write();
   outfile1->Close();

   top->Draw("ogl");
}

void create_materials_from_media_file()
{

   // Use the FairRoot geometry interface to load the media which are already defined
   FairGeoLoader *geoLoad = new FairGeoLoader("TGeo", "FairGeoLoader");
   FairGeoInterface *geoFace = geoLoad->getGeoInterface();
   TString geoPath = gSystem->Getenv("VMCWORKDIR");
   TString geoFile = geoPath + "/geometry/media.geo";
   geoFace->setMediaFile(geoFile);
   geoFace->readMedia();

   // Read the required media and create them in the GeoManager
   FairGeoMedia *geoMedia = geoFace->getMedia();
   FairGeoBuilder *geoBuild = geoLoad->getGeoBuilder();

   FairGeoMedium *GADGET_5IsoAr_800 = geoMedia->getMedium("GADGET_5IsoAr_800");
   FairGeoMedium *steel = geoMedia->getMedium("steel");
   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *kapton = geoMedia->getMedium("kapton");
   FairGeoMedium *G10 = geoMedia->getMedium("G10");

   FairGeoMedium *germanium = geoMedia->getMedium("germanium");
   FairGeoMedium *aluminium = geoMedia->getMedium("aluminium");

   // include check if all media are found

   geoBuild->createMedium(GADGET_5IsoAr_800);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(kapton);
   geoBuild->createMedium(G10);
   geoBuild->createMedium(germanium);
   geoBuild->createMedium(aluminium);


}

TGeoVolume *create_detector()
{

   // needed materials
   TGeoMedium *OuterCylinder = gGeoMan->GetMedium(CylinderVolumeMedium);
   TGeoMedium *gas = gGeoMan->GetMedium(MediumGas);
   TGeoMedium *windowmat = gGeoMan->GetMedium(WindowMedium);
   TGeoMedium *micromegasmat = gGeoMan->GetMedium(MicromegasMedium);

   TGeoMedium *shellmatter = gGeoMan->GetMedium(ShellMatter);
   TGeoMedium *degaimatter = gGeoMan->GetMedium(DeGAiMatter);
   TGeoMedium *mediumvacuum4 = gGeoMan->GetMedium(MediumVacuum);

   TGeoVolume *dummy = gGeoManager->MakeTube("dummy", mediumvacuum4, 0, 0, 0);
   gGeoMan->GetVolume(geoVersion)->AddNode(dummy, 1, new TGeoCombiTrans(20.0, 0, 0, new TGeoRotation("dummy", 0, 0, 0)));
   dummy->SetTransparency(100);
/*
   //Shell with holes
   TGeoTube *entrance = new TGeoTube("entrance", 0, Hull_Outer_Radii/2, Hull_Outer_Radii +3 );

   TGeoSphere *body = new TGeoSphere("body", Hull_Inner_Radii, Hull_Outer_Radii);
   TGeoCompositeShape *shell = new TGeoCompositeShape("shell", "(body - entrance)");

   TGeoVolume *shelly = new TGeoVolume("shelly", shell,shellmatter);
   shelly->SetLineColor(kCyan);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(shelly, 1,
                new TGeoCombiTrans(0.0, 0,39/2, new TGeoRotation("shelly", 0, 0, 0)));
   shelly->SetTransparency(70);

 */
   //detector
   TGeoVolume **Cry_vol;
   Cry_vol = new TGeoVolume *[40];

   TString CrystalName = "Crystal_";
   TString name_cry[40] = {"01", "02", "03", "04", "05", "06", "07", "08", "09","10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40"};

   // CLover

   TGeoTube *Crystal = new TGeoTube("Det1", 0, Crystal_Diameter/2, Crystal_Length/2);
   TGeoTube *Hole = new TGeoTube("Hole", 0, Hole_Diameter/2, Hole_Length/2);
   TGeoCombiTrans *Holetrans = new TGeoCombiTrans("Holetrans", 0, 0, -2.0, new TGeoRotation("Holetrans", 0, 0, 0));
   Holetrans->RegisterYourself();
   TGeoBBox *FlattenCrystalV = new TGeoBBox("FlattenCrystalV", 0.38, Crystal_Diameter/2, Crystal_Length/2+1);
   TGeoCombiTrans *FlattenCrystaltrans1 = new TGeoCombiTrans("FlattenCrystaltrans1", 2.5, 0, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   TGeoCombiTrans *FlattenCrystaltrans2 = new TGeoCombiTrans("FlattenCrystaltrans2", -2.5, 0, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   TGeoBBox *FlattenCrystalH = new TGeoBBox("FlattenCrystalH",Crystal_Diameter/2, 0.38, Crystal_Length/2+1);
   TGeoCombiTrans *FlattenCrystaltrans3 = new TGeoCombiTrans("FlattenCrystaltrans3", 0,2.5, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   TGeoCombiTrans *FlattenCrystaltrans4 = new TGeoCombiTrans("FlattenCrystaltrans4", 0,-2.5, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   FlattenCrystaltrans1->RegisterYourself();
   FlattenCrystaltrans2->RegisterYourself();
   FlattenCrystaltrans3->RegisterYourself();
   FlattenCrystaltrans4->RegisterYourself();

 // Top Right clover
   TGeoCompositeShape *Clover1 = new TGeoCompositeShape("Clover1", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans4)");
   TGeoVolume *Clover1_vol = new TGeoVolume("Clover1_vol", Clover1, degaimatter);   
   Clover1_vol->SetLineColor(kBlue);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(Clover1_vol, 1,
                new TGeoCombiTrans(-2.5+0.38, 2.5-0.38, 0, new TGeoRotation("Clover1_vol", 0, 0, 0)));
   Clover1_vol->SetTransparency(0);

// Top Left clover
   TGeoCompositeShape *Clover2 = new TGeoCompositeShape("Clover2", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans4)");
   TGeoVolume *Clover2_vol = new TGeoVolume("Clover2_vol", Clover2, degaimatter);   
   Clover2_vol->SetLineColor(kYellow);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(Clover2_vol, 1,
                new TGeoCombiTrans(2.5-0.38, 2.5-0.38, 0, new TGeoRotation("Clover2_vol", 0, 0, 0)));
   Clover2_vol->SetTransparency(0);

// Bottom Right clover
   TGeoCompositeShape *Clover3 = new TGeoCompositeShape("Clover3", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans3)");
   TGeoVolume *Clover3_vol = new TGeoVolume("Clover3_vol", Clover3, degaimatter);
   Clover3_vol->SetLineColor(kGreen);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(Clover3_vol, 1,
                new TGeoCombiTrans(-2.5+0.38, -2.5+0.38, 0, new TGeoRotation("Clover3_vol", 0, 0, 0)));
   Clover3_vol->SetTransparency(0);

// Bottom Left clover
   TGeoCompositeShape *Clover4 = new TGeoCompositeShape("Clover4", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans3)");
   TGeoVolume *Clover4_vol = new TGeoVolume("Clover4_vol", Clover4, degaimatter);
   Clover4_vol->SetLineColor(kRed);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(Clover4_vol, 1,
                new TGeoCombiTrans(2.5-0.38,-2.5+0.38, 0, new TGeoRotation("Clover4_vol", 0, 0, 0)));
   Clover4_vol->SetTransparency(0);

// Cover
   TGeoCombiTrans *TRtrans = new TGeoCombiTrans("TRtrans",-2.5+0.38, 2.5-0.38, 0, new TGeoRotation("TRtrans", 0, 0, 0));
   TRtrans->RegisterYourself();
   TGeoCombiTrans *TLtrans = new TGeoCombiTrans("TLtrans",2.5-0.38, 2.5-0.38, 0, new TGeoRotation("TLtrans", 0, 0, 0));
   TLtrans->RegisterYourself();
   TGeoCombiTrans *BRtrans = new TGeoCombiTrans("BRtrans",-2.5+0.38, -2.5+0.38, 0, new TGeoRotation("BRtrans", 0, 0, 0));
   BRtrans->RegisterYourself();
   TGeoCombiTrans *BLtrans = new TGeoCombiTrans("BLtrans",2.5-0.38,-2.5+0.38, 0, new TGeoRotation("BLtrans", 0, 0, 0));
   BLtrans->RegisterYourself();
   
   TGeoCompositeShape *Clover1hole = new TGeoCompositeShape("Clover1hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans4)");
   TGeoCompositeShape *Clover2hole = new TGeoCompositeShape("Clover2hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans4)");
   TGeoCompositeShape *Clover3hole = new TGeoCompositeShape("Clover3hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans3)");
   TGeoCompositeShape *Clover4hole = new TGeoCompositeShape("Clover4hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans3)");




   TGeoBBox *Box = new TGeoBBox("Box", Box_Width/2, Box_Width/2, Box_length/2-0.01);
   TGeoCompositeShape *Cover = new TGeoCompositeShape("Cover", "Box - Clover1hole:TRtrans - Clover2hole:TLtrans - Clover3hole:BRtrans - Clover4hole:BLtrans ");
   TGeoVolume *Cover_vol = new TGeoVolume("Cover_vol", Cover, shellmatter);
   Cover_vol->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(Cover_vol, 1,
                new TGeoCombiTrans(0, 0, 0, new TGeoRotation("Cover_vol", 0, 0, 0)));
   Cover_vol->SetTransparency(0);

/*
   
   // GADGET Main drift volume
   double tpc_rot = 0;
   TGeoVolume *drift_volume = gGeoManager->MakeTube("drift_volume", gas, 0, tpc_diameter_in / 2, drift_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(drift_volume, 1,
                new TGeoCombiTrans(0.0, 0, drift_length / 2.0, new TGeoRotation("drift_volume", 0, tpc_rot, 0)));
   drift_volume->SetTransparency(80);
   // GADGET Outer steel chamber
   TGeoVolume *steel_chamber = gGeoManager->MakeTube("steel_chamber", OuterCylinder, tpc_diameter_in / 2,
                                                     tpc_diameter_out / 2, drift_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(steel_chamber, 1,
                new TGeoCombiTrans(0.0, 0, drift_length / 2.0, new TGeoRotation("steel_chamber", 0, tpc_rot, 0)));
   steel_chamber->SetTransparency(80);
   // GADGET Upstream  Ring
   TGeoVolume *upstream_ring = gGeoManager->MakeTube("upstream_ring", OuterCylinder, upstream_ring_dia_in / 2,
                                                     upstream_ring_dia_out / 2, upstream_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(
         upstream_ring, 1,
         new TGeoCombiTrans(0.0, 0, -upstream_ring_length / 2.0, new TGeoRotation("upstream_ring", 0, tpc_rot, 0)));
   upstream_ring->SetTransparency(80);
   // GADGET Upstream Cap
   TGeoVolume *upstream_cap = gGeoManager->MakeTube("upstream_cap", OuterCylinder, upstream_cap_dia_in / 2,
                                                    upstream_cap_dia_out / 2, upstream_cap_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(upstream_cap, 1,
                new TGeoCombiTrans(0.0, 0, -upstream_cap_length / 2.0 - upstream_ring_length,
                                   new TGeoRotation("upstream_cap", 0, tpc_rot, 0)));
   upstream_cap->SetTransparency(80);
   // GADGET Downstream  Ring
   TGeoVolume *downstream_ring = gGeoManager->MakeTube("downstream_ring", OuterCylinder, downstream_ring_dia_in / 2,
                                                       downstream_ring_dia_out / 2, downstream_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(downstream_ring, 1,
                new TGeoCombiTrans(0.0, 0, drift_length + downstream_ring_length / 2.0,
                                   new TGeoRotation("downstream_ring", 0, tpc_rot, 0)));
   downstream_ring->SetTransparency(80);
   // GADGET Downstream Cap
   TGeoVolume *downstream_cap = gGeoManager->MakeTube("downstream_cap", OuterCylinder, downstream_cap_dia_in / 2,
                                                      downstream_cap_dia_out / 2, downstream_cap_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(downstream_cap, 1,
                new TGeoCombiTrans(0.0, 0, downstream_cap_length / 2.0 + downstream_ring_length + drift_length,
                                   new TGeoRotation("downstream_cap", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(80);
   // GADGET Entrance Window
   TGeoVolume *tpc_window =
      gGeoManager->MakeTube("tpc_window", OuterCylinder, window_dia_in / 2, window_dia_out / 2, window_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(tpc_window, 1,
                new TGeoCombiTrans(0.0, 0, -upstream_ring_length - upstream_cap_length - window_length / 2,
                                   new TGeoRotation("tpc_window", 0, tpc_rot, 0)));
   tpc_window->SetTransparency(80);
   // GADGET Kapton Window
   TGeoVolume *kapton_window = gGeoManager->MakeTube("kapton_window", windowmat, 0, 5.08 / 2.0, 0.0127 / 2.0);
   kapton_window->SetLineColor(kBlue);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(kapton_window, 1,
                new TGeoCombiTrans(0.0, 0.0, -upstream_ring_length - upstream_cap_length,
                                   new TGeoRotation("kapton_window", 0, tpc_rot, 0)));
   kapton_window->SetTransparency(50);
   // GADGET Micromeags
   TGeoVolume *micromegas =
      gGeoManager->MakeTube("micromegas", micromegasmat, 0, micromegas_dia_out / 2, micromegas_length / 2);
   micromegas->SetLineColor(kRed);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(micromegas, 1,
                new TGeoCombiTrans(
                   0.0, 0.0, drift_length + downstream_ring_length + downstream_cap_length + micromegas_length / 2,
                   new TGeoRotation("micromegas", 0, tpc_rot, 0)));
   micromegas->SetTransparency(50);
   // GADGET Micromegas Ring
   TGeoVolume *micromegas_ring = gGeoManager->MakeTube("micromegas_ring", OuterCylinder, micromegas_ring_dia_in / 2,
                                                       micromegas_ring_dia_out / 2, micromegas_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(micromegas_ring, 1,
                new TGeoCombiTrans(0.0, 0,
                                   micromegas_ring_length / 2.0 + drift_length + micromegas_length +
                                      downstream_ring_length + downstream_cap_length,
                                   new TGeoRotation("micromegas_ring", 0, tpc_rot, 0)));
   micromegas_ring->SetTransparency(80);
   // GADGET Cathode
   TGeoVolume *cathode =
      gGeoManager->MakeTube("cathode", OuterCylinder, cathode_dia_in / 2, cathode_dia_out / 2, cathode_length / 2);
   cathode->SetLineColor(kGreen);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(cathode, 1, new TGeoCombiTrans(0.0, 0, cathode_length / 2, new TGeoRotation("cathode", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(80);
   // GADGET Cathode Mount
   TGeoVolume *cathode_mount = gGeoManager->MakeTube("cathode_mount", OuterCylinder, cathode_mount_dia_in / 2,
                                                     cathode_mount_dia_out / 2, cathode_mount_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(cathode_mount, 1,
                new TGeoCombiTrans(0.0, 0, cathode_mount_length / 2, new TGeoRotation("cathode_mount", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(80);
*/
   return dummy;
}
