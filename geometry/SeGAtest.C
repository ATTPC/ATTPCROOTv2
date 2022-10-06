

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
const TString geoVersion = "SeGAtest";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "GADGET_P10_800";

const TString MediumVacuum = "vacuum4";

const TString SegaMatter = "germanium";
const TString SegaCryoMatter = "aluminium";

// Detector Dimensions (cm)
const Float_t tpc_diameter_in = 15.4051;
const Float_t tpc_diameter_out = 16.51;
const Float_t drift_length = 40.;

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

void SeGAtest()
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

   FairGeoMedium *germanium = geoMedia->getMedium("germanium");
   FairGeoMedium *aluminium = geoMedia->getMedium("aluminium");

   // include check if all media are found

   ;
   geoBuild->createMedium(vacuum4);

   geoBuild->createMedium(germanium);
   geoBuild->createMedium(aluminium);
}

TGeoVolume *create_detector()
{

   // needed materials

   TGeoMedium *segamatter = gGeoMan->GetMedium(SegaMatter);
   TGeoMedium *segacryomatter = gGeoMan->GetMedium(SegaCryoMatter);
   TGeoMedium *mediumvacuum4 = gGeoMan->GetMedium(MediumVacuum);
   // dummy// need this to work for some reason

   TGeoVolume *dummy = gGeoManager->MakeTube("dummy", mediumvacuum4, 0, 0, 0);
   gGeoMan->GetVolume(geoVersion)->AddNode(dummy, 1, new TGeoCombiTrans(0.0, 0, 0, new TGeoRotation("dummy", 0, 0, 0)));
   dummy->SetTransparency(100);

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
      cencon->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cencon, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cencon", phirot[nora], updown * thetarot[nora], 0.0)));
      cencon->SetTransparency(80);

      // GADGET SeGA inner dead layer
      TGeoVolume *indeadL = gGeoManager->MakeTube("indeadL", segamatter, sega_central_contact_rad_out,
                                                  sega_inner_dead_layer_rad_out, sega_inner_dead_layer_length / 2);
      indeadL->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(indeadL, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("indeadL", phirot[nora], updown * thetarot[nora], 0.0)));
      indeadL->SetTransparency(80);
      // GADGET SeGA active layer
      TGeoVolume *activeL = gGeoManager->MakeTube("activeL", segamatter, sega_inner_dead_layer_rad_out,
                                                  sega_active_rad_out, sega_active_length / 2);
      activeL->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(activeL, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("activeL", phirot[nora], updown * thetarot[nora], 0)));
      activeL->SetTransparency(80);

      // GADGET SeGA germanium crystal

      Cry_vol[nora] = gGeoManager->MakeTube(CrystalName + name_cry[nora], segamatter, sega_active_rad_out,
                                            sega_Ge_crystal_rad_out, sega_Ge_crystal_length / 2);
      Cry_vol[nora]->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[nora], nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("GeCrystal", phirot[nora], updown * thetarot[nora], 0)));
      Cry_vol[nora]->SetTransparency(80);
      // GADGET SeGA vacuum 1
      TGeoVolume *vac1 =
         gGeoManager->MakeTube("vac1", mediumvacuum4, sega_Ge_crystal_rad_out, sega_vac1_rad_out, sega_vac1_length / 2);
      vac1->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac1, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac1", phirot[nora], updown * thetarot[nora], 0)));
      vac1->SetTransparency(80);
      // GADGET SeGA detector cup
      TGeoVolume *segadetcup = gGeoManager->MakeTube("segadetcup", segacryomatter, sega_vac1_rad_out,
                                                     sega_det_cup_rad_out, sega_det_cup_length / 2);
      segadetcup->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(segadetcup, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("segadetcup", phirot[nora], updown * thetarot[nora], 0)));
      segadetcup->SetTransparency(80);

      // GADGET SeGA vacuum 2
      TGeoVolume *vac2 =
         gGeoManager->MakeTube("vac2", mediumvacuum4, sega_det_cup_rad_out, sega_vac2_rad_out, sega_vac2_length / 2);
      vac2->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac2, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac2", phirot[nora], updown * thetarot[nora], 0)));
      vac2->SetTransparency(80);

      // GADGET SeGA Cryostat
      TGeoVolume *cryostat =
         gGeoManager->MakeTube("cryostat", segacryomatter, sega_vac2_rad_out, sega_cryo_rad_out, sega_cryo_length / 2);
      cryostat->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cryostat, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cryostat", phirot[nora], updown * thetarot[nora], 0)));
      cryostat->SetTransparency(80);
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
      cencon->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cencon, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cencon", phirot[nora], updown * thetarot[nora], 0.0)));
      cencon->SetTransparency(80);
      cout << "sega_Ge_crystal_rad_out" << sega_Ge_crystal_rad_out << endl;
      // GADGET SeGA inner dead layer
      TGeoVolume *indeadL = gGeoManager->MakeTube("indeadL", segamatter, sega_central_contact_rad_out,
                                                  sega_inner_dead_layer_rad_out, sega_inner_dead_layer_length / 2);
      indeadL->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(indeadL, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("indeadL", phirot[nora], updown * thetarot[nora], 0.0)));
      indeadL->SetTransparency(80);
      // GADGET SeGA active layer
      TGeoVolume *activeL = gGeoManager->MakeTube("activeL", segamatter, sega_inner_dead_layer_rad_out,
                                                  sega_active_rad_out, sega_active_length / 2);
      activeL->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(activeL, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("activeL", phirot[nora], updown * thetarot[nora], 0)));
      activeL->SetTransparency(80);

      // GADGET SeGA germanium crystal

      Cry_vol[nora + 8] = gGeoManager->MakeTube(CrystalName + name_cry[nora + 8], segamatter, sega_active_rad_out,
                                                sega_Ge_crystal_rad_out, sega_Ge_crystal_length / 2);
      Cry_vol[nora + 8]->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[nora + 8], nora + 9,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("GeCrystal", phirot[nora], updown * thetarot[nora], 0)));
      Cry_vol[nora + 8]->SetTransparency(80);
      // GADGET SeGA vacuum 1
      TGeoVolume *vac1 =
         gGeoManager->MakeTube("vac1", mediumvacuum4, sega_Ge_crystal_rad_out, sega_vac1_rad_out, sega_vac1_length / 2);
      vac1->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac1, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac1", phirot[nora], updown * thetarot[nora], 0)));
      vac1->SetTransparency(80);
      // GADGET SeGA detector cup
      TGeoVolume *segadetcup = gGeoManager->MakeTube("segadetcup", segacryomatter, sega_vac1_rad_out,
                                                     sega_det_cup_rad_out, sega_det_cup_length / 2);
      segadetcup->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(segadetcup, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("segadetcup", phirot[nora], updown * thetarot[nora], 0)));
      segadetcup->SetTransparency(80);

      // GADGET SeGA vacuum 2
      TGeoVolume *vac2 =
         gGeoManager->MakeTube("vac2", mediumvacuum4, sega_det_cup_rad_out, sega_vac2_rad_out, sega_vac2_length / 2);
      vac2->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(vac2, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("vac2", phirot[nora], updown * thetarot[nora], 0)));
      vac2->SetTransparency(80);

      // GADGET SeGA Cryostat
      TGeoVolume *cryostat =
         gGeoManager->MakeTube("cryostat", segacryomatter, sega_vac2_rad_out, sega_cryo_rad_out, sega_cryo_length / 2);
      cryostat->SetLineColor(kCyan);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(cryostat, nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("cryostat", phirot[nora], updown * thetarot[nora], 0)));
      cryostat->SetTransparency(80);
   };

   // BALL

   Cry_vol[16] = gGeoManager->MakeSphere(CrystalName + name_cry[16], segamatter, 17, 27);
   Cry_vol[16]->SetLineColor(kMagenta);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(Cry_vol[16], 17, new TGeoCombiTrans(0.0, 0.0, 20, new TGeoRotation("GeCrystal", 0, 0, 0)));
   Cry_vol[16]->SetTransparency(80);

   return dummy;
}
