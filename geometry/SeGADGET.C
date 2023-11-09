

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
#include "TMath.h"
#include "TROOT.h"
#include "TString.h"
#include "TSystem.h"

#include <iostream>

// Name of geometry version and output file
const TString geoVersion = "SeGADGET";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "GADGET_P10_800";

const TString MediumVacuum = "vacuum4";

const TString SegaMatter = "germanium";
const TString SegaCryoMatter = "aluminium";
// tpc

const TString CylinderVolumeMedium = "steel";

const TString WindowMedium = "kapton";
const TString MicromegasMedium = "G10";

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

const Float_t sega_cryo_length = 8.56;
const Float_t sega_cryo_thickness = 0.32;
const Float_t sega_cryo_rad_out = 4.4;
const Float_t sega_vac2_length = 8.5;
const Float_t sega_vac2_thickness = 0.445;
const Float_t sega_vac2_rad_out = sega_cryo_rad_out - sega_cryo_thickness;
const Float_t sega_det_cup_length = 8.1;
const Float_t sega_det_cup_thickness = 0.098;
const Float_t sega_det_cup_rad_out = sega_vac2_rad_out - sega_vac2_thickness;
const Float_t sega_vac1_length = 8.0;
const Float_t sega_vac1_thickness = 0.152;
const Float_t sega_vac1_rad_out = sega_det_cup_rad_out - sega_det_cup_thickness;
const Float_t sega_Ge_crystal_length = 8.0;
const Float_t sega_Ge_crystal_thickness = 0.0195;
const Float_t sega_Ge_crystal_rad_out = sega_vac1_rad_out - sega_vac1_thickness;
const Float_t sega_active_length = 7.9;
const Float_t sega_active_thickness = 3.185;
const Float_t sega_active_rad_out = sega_Ge_crystal_rad_out - sega_Ge_crystal_thickness;
const Float_t sega_inner_dead_layer_length = 7.8;
const Float_t sega_inner_dead_layer_thickness = 0.08;
const Float_t sega_inner_dead_layer_rad_out = sega_active_rad_out - sega_active_thickness;
const Float_t sega_central_contact_length = 7.0;
const Float_t sega_central_contact_thickness = 0.01;
const Float_t sega_central_contact_rad_out = sega_inner_dead_layer_rad_out - sega_inner_dead_layer_thickness;

// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void SeGADGET()
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

   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *GADGET_5IsoAr_800 = geoMedia->getMedium("GADGET_5IsoAr_800");
   FairGeoMedium *steel = geoMedia->getMedium("steel");

   FairGeoMedium *kapton = geoMedia->getMedium("kapton");
   FairGeoMedium *G10 = geoMedia->getMedium("G10");
   FairGeoMedium *germanium = geoMedia->getMedium("germanium");
   FairGeoMedium *aluminium = geoMedia->getMedium("aluminium");

   // include check if all media are found
   geoBuild->createMedium(GADGET_5IsoAr_800);
   geoBuild->createMedium(steel);

   geoBuild->createMedium(kapton);
   geoBuild->createMedium(G10);
   ;
   geoBuild->createMedium(vacuum4);

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
   TGeoMedium *segamatter = gGeoMan->GetMedium(SegaMatter);
   TGeoMedium *segacryomatter = gGeoMan->GetMedium(SegaCryoMatter);
   TGeoMedium *mediumvacuum4 = gGeoMan->GetMedium(MediumVacuum);
   // dummy// need this to work for some reason
   /*

      TGeoVolume *dummy = gGeoManager->MakeTube("dummy", mediumvacuum4, 0, 0, 0);
      gGeoMan->GetVolume(geoVersion)->AddNode(dummy, 1, new TGeoCombiTrans(0.0, 0, 0, new TGeoRotation("dummy", 0, 0,
      0))); dummy->SetTransparency(60);*/

   TGeoVolume **Cry_vol;
   Cry_vol = new TGeoVolume *[17];

   TString CrystalName = "Crystal_";
   TString name_cry[17] = {"01", "02", "03", "04", "05", "06", "07", "08", "09",
                           "10", "11", "12", "13", "14", "15", "16", "17"};

   // upstream ring
   Float_t radian_arraycos[8] = {1, 0.707106781187, 0, -0.707106781187, -1, -0.707106781187,
                                 0, 0.707106781187}; // putting cos and sin values for the polar cordinates
   Float_t radian_arraysin[8] = {0, 0.707106781187, 1, 0.707106781187, 0, -0.707106781187, -1, -0.707106781187};
   Float_t phirot[8] = {90.0, -45.0, 0.0, 45.0, 90.0, -45.0, 0.0, 45.0}; // puting the rotation angles from the
                                                                         // detector.
   Float_t thetarot[8] = {-45.0, 45.0, 45.0, 45.0, 45.0, -45.0, -45.0, -45.0};
   for (int nora = 0; nora < 8; nora++) {

      Float_t x_posSega = (17.3 / 2 + 3.5) * radian_arraycos[nora];
      Float_t y_posSega = (17.3 / 2 + 3.5) * radian_arraysin[nora];
      Float_t z_posSega = 20 - 8.56 / 2 - 1.3746;
      Float_t updown = 0;

      // GADGET SeGA central contact
      TGeoVolume *cencon = gGeoManager->MakeTube("cencon", mediumvacuum4, 0, sega_central_contact_rad_out,
                                                 sega_central_contact_length / 2);
      cencon->SetLineColor(kRed);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cencon, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cencon", phirot[nora], updown * thetarot[nora], 0.0)));
      cencon->SetTransparency(60);

      // GADGET SeGA inner dead layer
      TGeoVolume *indeadL = gGeoManager->MakeTube("indeadL", segamatter, sega_central_contact_rad_out,
                                                  sega_inner_dead_layer_rad_out, sega_inner_dead_layer_length / 2);
      indeadL->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(indeadL, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("indeadL", phirot[nora], updown * thetarot[nora], 0.0)));
      indeadL->SetTransparency(40);
      // GADGET SeGA active layer
      Cry_vol[nora] = gGeoManager->MakeTube(CrystalName + name_cry[nora], segamatter, sega_inner_dead_layer_rad_out,
                                            sega_active_rad_out, sega_active_length / 2);
      Cry_vol[nora]->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[nora], nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("activeL", phirot[nora], updown * thetarot[nora], 0)));
      Cry_vol[nora]->SetTransparency(40);

      // GADGET SeGA germanium crystal

      TGeoVolume *GeCrystal = gGeoManager->MakeTube("GeCrystal", segamatter, sega_active_rad_out,
                                                    sega_Ge_crystal_rad_out, sega_Ge_crystal_length / 2);
      GeCrystal->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(GeCrystal, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("GeCrystal", phirot[nora], updown * thetarot[nora], 0)));
      GeCrystal->SetTransparency(40);
      // GADGET SeGA vacuum 1
      TGeoVolume *vac1 =
         gGeoManager->MakeTube("vac1", mediumvacuum4, sega_Ge_crystal_rad_out, sega_vac1_rad_out, sega_vac1_length / 2);
      vac1->SetLineColor(kRed);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac1, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac1", phirot[nora], updown * thetarot[nora], 0)));
      vac1->SetTransparency(60);
      // GADGET SeGA detector cup
      TGeoVolume *segadetcup = gGeoManager->MakeTube("segadetcup", segacryomatter, sega_vac1_rad_out,
                                                     sega_det_cup_rad_out, sega_det_cup_length / 2);
      segadetcup->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(segadetcup, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("segadetcup", phirot[nora], updown * thetarot[nora], 0)));
      segadetcup->SetTransparency(40);

      // GADGET SeGA vacuum 2
      TGeoVolume *vac2 =
         gGeoManager->MakeTube("vac2", mediumvacuum4, sega_det_cup_rad_out, sega_vac2_rad_out, sega_vac2_length / 2);
      vac2->SetLineColor(kRed);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac2, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac2", phirot[nora], updown * thetarot[nora], 0)));
      vac2->SetTransparency(30);

      // GADGET SeGA Cryostat
      TGeoVolume *cryostat =
         gGeoManager->MakeTube("cryostat", segacryomatter, sega_vac2_rad_out, sega_cryo_rad_out, sega_cryo_length / 2);
      cryostat->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cryostat, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cryostat", phirot[nora], updown * thetarot[nora], 0)));
      cryostat->SetTransparency(40);
   };
   // radius = (tpc_diameter_in /2+sega_cryo_rad_out+2)
   // downstream ring

   for (int nora = 0; nora < 8; nora++) {

      Float_t x_posSega = (17.3 / 2 + 3.5) * radian_arraycos[nora];
      Float_t y_posSega = (17.3 / 2 + 3.5) * radian_arraysin[nora];
      Float_t z_posSega = 20 + 8.56 / 2 + 1.3746;
      Float_t updown = 0;

      // GADGET SeGA central contact
      TGeoVolume *cencon = gGeoManager->MakeTube("cencon", mediumvacuum4, 0, sega_central_contact_rad_out,
                                                 sega_central_contact_length / 2);
      cencon->SetLineColor(kRed);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cencon, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cencon", phirot[nora], updown * thetarot[nora], 0.0)));
      cencon->SetTransparency(60);
      cout << "sega_Ge_crystal_rad_out" << sega_Ge_crystal_rad_out << endl;
      // GADGET SeGA inner dead layer
      TGeoVolume *indeadL = gGeoManager->MakeTube("indeadL", segamatter, sega_central_contact_rad_out,
                                                  sega_inner_dead_layer_rad_out, sega_inner_dead_layer_length / 2);
      indeadL->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(indeadL, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("indeadL", phirot[nora], updown * thetarot[nora], 0.0)));
      indeadL->SetTransparency(40);
      // GADGET SeGA active layer
      Cry_vol[nora + 8] =
         gGeoManager->MakeTube(CrystalName + name_cry[nora + 8], segamatter, sega_inner_dead_layer_rad_out,
                               sega_active_rad_out, sega_active_length / 2);
      Cry_vol[nora]->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[nora + 8], nora + 9,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("activeL", phirot[nora], updown * thetarot[nora], 0)));
      Cry_vol[nora]->SetTransparency(40);

      // GADGET SeGA germanium crystal

      TGeoVolume *GeCrystal = gGeoManager->MakeTube("GeCrystal", segamatter, sega_active_rad_out,
                                                    sega_Ge_crystal_rad_out, sega_Ge_crystal_length / 2);
      GeCrystal->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(GeCrystal, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("GeCrystal", phirot[nora], updown * thetarot[nora], 0)));
      GeCrystal->SetTransparency(40);
      // GADGET SeGA vacuum 1
      TGeoVolume *vac1 =
         gGeoManager->MakeTube("vac1", mediumvacuum4, sega_Ge_crystal_rad_out, sega_vac1_rad_out, sega_vac1_length / 2);
      vac1->SetLineColor(kRed);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac1, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac1", phirot[nora], updown * thetarot[nora], 0)));
      vac1->SetTransparency(60);
      // GADGET SeGA detector cup
      TGeoVolume *segadetcup = gGeoManager->MakeTube("segadetcup", segacryomatter, sega_vac1_rad_out,
                                                     sega_det_cup_rad_out, sega_det_cup_length / 2);
      segadetcup->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(segadetcup, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("segadetcup", phirot[nora], updown * thetarot[nora], 0)));
      segadetcup->SetTransparency(40);

      // GADGET SeGA vacuum 2
      TGeoVolume *vac2 =
         gGeoManager->MakeTube("vac2", mediumvacuum4, sega_det_cup_rad_out, sega_vac2_rad_out, sega_vac2_length / 2);
      vac2->SetLineColor(kRed);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac2, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac2", phirot[nora], updown * thetarot[nora], 0)));
      vac2->SetTransparency(30);

      // GADGET SeGA Cryostat
      TGeoVolume *cryostat =
         gGeoManager->MakeTube("cryostat", segacryomatter, sega_vac2_rad_out, sega_cryo_rad_out, sega_cryo_length / 2);
      cryostat->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cryostat, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cryostat", phirot[nora], updown * thetarot[nora], 0)));
      cryostat->SetTransparency(40);
   };
   // GADGET Main drift volume
   double tpc_rot = 0;
   TGeoVolume *drift_volume = gGeoManager->MakeTube("drift_volume", gas, 0, tpc_diameter_in / 2, drift_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(drift_volume, 1,
                new TGeoCombiTrans(0.0, 0, drift_length / 2.0, new TGeoRotation("drift_volume", 0, tpc_rot, 0)));
   drift_volume->SetTransparency(40);

   // GADGET Outer steel chamber
   TGeoVolume *steel_chamber = gGeoManager->MakeTube("steel_chamber", OuterCylinder, tpc_diameter_in / 2,
                                                     tpc_diameter_out / 2, drift_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(steel_chamber, 1,
                new TGeoCombiTrans(0.0, 0, drift_length / 2.0, new TGeoRotation("steel_chamber", 0, tpc_rot, 0)));
   steel_chamber->SetTransparency(40);

   // GADGET Upstream  Ring
   TGeoVolume *upstream_ring = gGeoManager->MakeTube("upstream_ring", OuterCylinder, upstream_ring_dia_in / 2,
                                                     upstream_ring_dia_out / 2, upstream_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(
         upstream_ring, 1,
         new TGeoCombiTrans(0.0, 0, -upstream_ring_length / 2.0, new TGeoRotation("upstream_ring", 0, tpc_rot, 0)));
   upstream_ring->SetTransparency(40);

   // GADGET Upstream Cap
   TGeoVolume *upstream_cap = gGeoManager->MakeTube("upstream_cap", OuterCylinder, upstream_cap_dia_in / 2,
                                                    upstream_cap_dia_out / 2, upstream_cap_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(upstream_cap, 1,
                new TGeoCombiTrans(0.0, 0, -upstream_cap_length / 2.0 - upstream_ring_length,
                                   new TGeoRotation("upstream_cap", 0, tpc_rot, 0)));
   upstream_cap->SetTransparency(40);

   // GADGET Downstream  Ring
   TGeoVolume *downstream_ring = gGeoManager->MakeTube("downstream_ring", OuterCylinder, downstream_ring_dia_in / 2,
                                                       downstream_ring_dia_out / 2, downstream_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(downstream_ring, 1,
                new TGeoCombiTrans(0.0, 0, drift_length + downstream_ring_length / 2.0,
                                   new TGeoRotation("downstream_ring", 0, tpc_rot, 0)));
   downstream_ring->SetTransparency(40);

   // GADGET Downstream Cap
   TGeoVolume *downstream_cap = gGeoManager->MakeTube("downstream_cap", OuterCylinder, downstream_cap_dia_in / 2,
                                                      downstream_cap_dia_out / 2, downstream_cap_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(downstream_cap, 1,
                new TGeoCombiTrans(0.0, 0, downstream_cap_length / 2.0 + downstream_ring_length + drift_length,
                                   new TGeoRotation("downstream_cap", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(40);

   // GADGET Entrance Window
   TGeoVolume *tpc_window =
      gGeoManager->MakeTube("tpc_window", OuterCylinder, window_dia_in / 2, window_dia_out / 2, window_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(tpc_window, 1,
                new TGeoCombiTrans(0.0, 0, -upstream_ring_length - upstream_cap_length - window_length / 2,
                                   new TGeoRotation("tpc_window", 0, tpc_rot, 0)));
   tpc_window->SetTransparency(40);

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
   micromegas_ring->SetTransparency(40);

   // GADGET Cathode
   TGeoVolume *cathode =
      gGeoManager->MakeTube("cathode", OuterCylinder, cathode_dia_in / 2, cathode_dia_out / 2, cathode_length / 2);
   cathode->SetLineColor(kGreen);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(cathode, 1, new TGeoCombiTrans(0.0, 0, cathode_length / 2, new TGeoRotation("cathode", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(40);

   // GADGET Cathode Mount
   TGeoVolume *cathode_mount = gGeoManager->MakeTube("cathode_mount", OuterCylinder, cathode_mount_dia_in / 2,
                                                     cathode_mount_dia_out / 2, cathode_mount_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(cathode_mount, 1,
                new TGeoCombiTrans(0.0, 0, cathode_mount_length / 2, new TGeoRotation("cathode_mount", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(40);
   // BALL
   /*
        Cry_vol[16]= gGeoManager->MakeSphere(CrystalName + name_cry[16], segamatter, 0,  25 );
        Cry_vol[16]->SetLineColor(kMagenta);
        gGeoMan->GetVolume(geoVersion)
           ->AddNode(Cry_vol[16],17, new TGeoCombiTrans(0.0, 0.0 , 20 - 8.56 / 2 - 1.3746,  new
     TGeoRotation("GeCrystal", 0 ,0, 0))); Cry_vol[16]->SetTransparency(40);
  */
   return drift_volume;
}
