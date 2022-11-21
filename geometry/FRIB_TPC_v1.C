/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
// in root all sizes are given in cm

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
#include "TString.h"
#include "TSystem.h"

#include <iostream>

// Name of geometry version and output file
const TString geoVersion = "FRIB_TPC_v1";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "Ar80N20_2bar";
const TString MediumVacuum = "vacuum4";
const TString MediumWindow = "kapton";
const TString CylinderVolumeMedium = "steel";
const TString TubeGas = "He_2bar";
const TString FCPlateMedium = "Aluminum";
const TString TubeWindowMedium = "Tedlar";

// some global variables
TGeoManager *gGeoMan = new TGeoManager("FRIBTPC", "FRIBTPC"); // Pointer to TGeoManager instance
TGeoVolume *gModules;                                         // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void FRIB_TPC_v1()
{
   // Load the necessary FairRoot libraries
   // gROOT->LoadMacro("$VMCWORKDIR/gconfig/basiclibs.C");
   // basiclibs();
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

   // Build the detector
   gModules = create_detector();

   // position_detector();

   cout << "Voxelizing." << endl;
   top->Voxelize("");
   gGeoMan->CloseGeometry();

   // add_alignable_volumes();

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
   // top->Raytrace();
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

   FairGeoMedium *driftGas = geoMedia->getMedium(MediumGas);
   FairGeoMedium *targetGas = geoMedia->getMedium(TubeGas);
   FairGeoMedium *steel = geoMedia->getMedium("steel");
   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *kapton = geoMedia->getMedium("kapton");
   FairGeoMedium *aluminum = geoMedia->getMedium("Aluminum");
   FairGeoMedium *tedlar = geoMedia->getMedium("Tedlar");

   // include check if all media are found
   geoBuild->createMedium(driftGas);
   geoBuild->createMedium(targetGas);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(kapton);
   geoBuild->createMedium(aluminum);
   geoBuild->createMedium(tedlar);
}

TGeoVolume *create_detector()
{

   // needed materials
   TGeoMedium *OuterCylinder = gGeoMan->GetMedium(CylinderVolumeMedium);
   TGeoMedium *gas = gGeoMan->GetMedium(MediumGas);
   TGeoMedium *tgas = gGeoMan->GetMedium(TubeGas);
   TGeoMedium *windowmat = gGeoMan->GetMedium(MediumWindow);
   TGeoMedium *fcmedium = gGeoMan->GetMedium(FCPlateMedium);
   TGeoMedium *tubewindowmat = gGeoMan->GetMedium(TubeWindowMedium);

   // FRIB-TPC gas volume
   // NB: If overlap, first volume that it is introduced is kept.

   double tpc_rot = 0;

   TGeoVolume *target_window =
      gGeoManager->MakeBox("target_window", tubewindowmat, 0.00060 / 2.0, 1.40 / 2.0, 30.0 / 2.0);
   target_window->SetLineColor(kRed);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(target_window, 3, new TGeoCombiTrans(8.6, 0.45, 0.0, new TGeoRotation("target_window", 0, 0, 0)));
   target_window->SetTransparency(60);

   TGeoVolume *target_volume = gGeoManager->MakeBox("target_drift_volume", tgas, 1.4 / 2.0, 1.40 / 2.0, 30.0 / 2.0);
   target_volume->SetLineColor(kBlue);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(target_volume, 2, new TGeoCombiTrans(9.3, 0.45, 0.0, new TGeoRotation("target_drift_volume", 0, 0, 0)));
   target_volume->SetTransparency(30);

   TGeoVolume *drift_volume = gGeoManager->MakeBox("drift_volume", gas, 20.0 / 2., 20.0 / 2., 30.0 / 2.);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(drift_volume, 1, new TGeoCombiTrans(0.0, 0.0, 0.0, new TGeoRotation("drift_volume", 0, 0, 0)));
   drift_volume->SetTransparency(90);

   TGeoVolume *tpc_window = gGeoManager->MakeTube("tpc_window", windowmat, 0, 1.40 / 2.0, 0.00100 / 2.0);
   tpc_window->SetLineColor(kOrange);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(tpc_window, 4, new TGeoCombiTrans(9.3, 0.45, -80.0 / 2.0, new TGeoRotation("tpc_window", 0, 0, 0)));
   tpc_window->SetTransparency(50);

   TGeoBBox *fcplate_in = new TGeoBBox("fcplate_in", 20.0 / 2.0, 0.2 / 2., 30.0 / 2.);
   TGeoBBox *fcplate_out = new TGeoBBox("fcplate_out", 17.2 / 2., 0.20001 / 2., 27.2 / 2.);

   Double_t fcOffset = 10.0; // cm

   for (auto i = 0; i < 14; ++i) {
      TGeoCompositeShape *fcplate = new TGeoCompositeShape("fcplate", "(fcplate_in - fcplate_out)");
      TGeoVolume *fcplatecomp = new TGeoVolume("fcplatecomp", fcplate, fcmedium);
      fcplatecomp->SetTransparency(70);
      fcplatecomp->SetLineColor(kGray);
      gGeoMan->GetVolume(geoVersion)->AddNode(fcplatecomp, 4, new TGeoCombiTrans(0.0, i * 1.6 - fcOffset, 0, 0));
   }

   // gGeoMan->GetVolume(geoVersion)->AddNode(fcplate_in, 1,new TGeoCombiTrans(0.0, 0.0, 0.0, new
   // TGeoRotation("fcplate_in", 0,0, 0))); fcplate_in->SetLineColor(kGrey); drift_volume->SetTransparency(90);

   return drift_volume;
}
