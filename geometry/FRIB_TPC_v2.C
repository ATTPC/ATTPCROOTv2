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
const TString geoVersion = "FRIB_TPC_v2";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "H_1bar";
const TString MediumVacuum = "vacuum4";
const TString MediumWindow = "kapton";
const TString CylinderVolumeMedium = "steel";
const TString TubeGas = "He_2bar";
const TString FCPlateMedium = "Aluminum";
const TString TubeWindowMedium = "Tedlar";
const TString MediumSi = "silicon";

// some global variables
TGeoManager *gGeoMan = new TGeoManager("FRIBTPC", "FRIBTPC"); // Pointer to TGeoManager instance
TGeoVolume *gModules;                                         // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void FRIB_TPC_v2()
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
   FairGeoMedium *silicon = geoMedia->getMedium("silicon");

   // include check if all media are found
   geoBuild->createMedium(driftGas);
   geoBuild->createMedium(targetGas);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(kapton);
   geoBuild->createMedium(aluminum);
   geoBuild->createMedium(tedlar);
   geoBuild->createMedium(silicon);
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
   TGeoMedium *silicon = gGeoMan->GetMedium(MediumSi);

   // FRIB-TPC gas volume
   // NB: If overlap, first volume that it is introduced is kept.

   double tpc_rot = 0;

   TGeoVolume *drift_volume = gGeoManager->MakeBox("drift_volume", gas, 20.0 / 2., 20.0 / 2., 30.0 / 2.);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(drift_volume, 1, new TGeoCombiTrans(0.0, 0.0, 0.0, new TGeoRotation("drift_volume", 0, 0, 0)));
   drift_volume->SetTransparency(90);

   TGeoVolume *tpc_window = gGeoManager->MakeTube("tpc_window", windowmat, 0, 1.40 / 2.0, 0.00100 / 2.0);
   tpc_window->SetLineColor(kOrange);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(tpc_window, 2, new TGeoCombiTrans(0.0, 0.0, -30.0 / 2.0, new TGeoRotation("tpc_window", 0, 0, 0)));
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

   // Left

   TGeoVolume *siliconL_1 = gGeoManager->MakeBox("siliconL_1", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_1, 3, new TGeoTranslation(-10.5, 8.0, 10.0));
   siliconL_1->SetLineColor(kGreen);

   TGeoVolume *siliconL_2 = gGeoManager->MakeBox("siliconL_2", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_2, 4, new TGeoTranslation(-10.5, 8.0, 4.9));
   siliconL_2->SetLineColor(kGreen);

   TGeoVolume *siliconL_3 = gGeoManager->MakeBox("siliconL_3", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_3, 5, new TGeoTranslation(-10.5, 8.0, -0.2));
   siliconL_3->SetLineColor(kGreen);

   TGeoVolume *siliconL_4 = gGeoManager->MakeBox("siliconL_4", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_4, 6, new TGeoTranslation(-10.5, 8.0, -5.3));
   siliconL_4->SetLineColor(kGreen);

   TGeoVolume *siliconL_5 = gGeoManager->MakeBox("siliconL_5", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_5, 7, new TGeoTranslation(-10.5, 8.0, -10.4));
   siliconL_5->SetLineColor(kGreen);

   //

   TGeoVolume *siliconL_6 = gGeoManager->MakeBox("siliconL_6", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_6, 8, new TGeoTranslation(-10.5, 2.9, 10.0));
   siliconL_6->SetLineColor(kGreen);

   TGeoVolume *siliconL_7 = gGeoManager->MakeBox("siliconL_7", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_7, 9, new TGeoTranslation(-10.5, 2.9, 4.9));
   siliconL_7->SetLineColor(kGreen);

   TGeoVolume *siliconL_8 = gGeoManager->MakeBox("siliconL_8", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_8, 10, new TGeoTranslation(-10.5, 2.9, -0.2));
   siliconL_8->SetLineColor(kGreen);

   TGeoVolume *siliconL_9 = gGeoManager->MakeBox("siliconL_9", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_9, 11, new TGeoTranslation(-10.5, 2.9, -5.3));
   siliconL_9->SetLineColor(kGreen);

   TGeoVolume *siliconL_10 = gGeoManager->MakeBox("siliconL_10", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_5, 12, new TGeoTranslation(-10.5, 2.9, -10.4));
   siliconL_10->SetLineColor(kGreen);

   //

   TGeoVolume *siliconL_11 = gGeoManager->MakeBox("siliconL_11", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_11, 13, new TGeoTranslation(-10.5, -2.2, 10.0));
   siliconL_11->SetLineColor(kGreen);

   TGeoVolume *siliconL_12 = gGeoManager->MakeBox("siliconL_12", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_12, 14, new TGeoTranslation(-10.5, -2.2, 4.9));
   siliconL_12->SetLineColor(kGreen);

   TGeoVolume *siliconL_13 = gGeoManager->MakeBox("siliconL_13", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_13, 15, new TGeoTranslation(-10.5, -2.2, -0.2));
   siliconL_13->SetLineColor(kGreen);

   TGeoVolume *siliconL_14 = gGeoManager->MakeBox("siliconL_14", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_14, 16, new TGeoTranslation(-10.5, -2.2, -5.3));
   siliconL_14->SetLineColor(kGreen);

   TGeoVolume *siliconL_15 = gGeoManager->MakeBox("siliconL_15", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_15, 17, new TGeoTranslation(-10.5, -2.2, -10.4));
   siliconL_15->SetLineColor(kGreen);

   //

   TGeoVolume *siliconL_16 = gGeoManager->MakeBox("siliconL_16", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_16, 18, new TGeoTranslation(-10.5, -7.3, 10.0));
   siliconL_16->SetLineColor(kGreen);

   TGeoVolume *siliconL_17 = gGeoManager->MakeBox("siliconL_17", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_17, 43, new TGeoTranslation(-10.5, -7.3, 4.9));
   siliconL_17->SetLineColor(kGreen);

   TGeoVolume *siliconL_18 = gGeoManager->MakeBox("siliconL_18", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_18, 20, new TGeoTranslation(-10.5, -7.3, -0.2));
   siliconL_18->SetLineColor(kGreen);

   TGeoVolume *siliconL_19 = gGeoManager->MakeBox("siliconL_19", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_19, 21, new TGeoTranslation(-10.5, -7.3, -5.3));
   siliconL_19->SetLineColor(kGreen);

   TGeoVolume *siliconL_20 = gGeoManager->MakeBox("siliconL_20", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconL_20, 22, new TGeoTranslation(-10.5, -7.3, -10.4));
   siliconL_20->SetLineColor(kGreen);

   // Right
   TGeoVolume *siliconR_1 = gGeoManager->MakeBox("siliconR_1", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_1, 23, new TGeoTranslation(10.5, 8.0, 10.0));
   siliconR_1->SetLineColor(kGreen);

   TGeoVolume *siliconR_2 = gGeoManager->MakeBox("siliconR_2", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_2, 24, new TGeoTranslation(10.5, 8.0, 4.9));
   siliconR_2->SetLineColor(kGreen);

   TGeoVolume *siliconR_3 = gGeoManager->MakeBox("siliconR_3", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_3, 25, new TGeoTranslation(10.5, 8.0, -0.2));
   siliconR_3->SetLineColor(kGreen);

   TGeoVolume *siliconR_4 = gGeoManager->MakeBox("siliconR_4", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_4, 26, new TGeoTranslation(10.5, 8.0, -5.3));
   siliconR_4->SetLineColor(kGreen);

   TGeoVolume *siliconR_5 = gGeoManager->MakeBox("siliconR_5", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_5, 27, new TGeoTranslation(10.5, 8.0, -10.4));
   siliconR_5->SetLineColor(kGreen);

   //

   TGeoVolume *siliconR_6 = gGeoManager->MakeBox("siliconR_6", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_6, 28, new TGeoTranslation(10.5, 2.9, 10.0));
   siliconR_6->SetLineColor(kGreen);

   TGeoVolume *siliconR_7 = gGeoManager->MakeBox("siliconR_7", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_7, 29, new TGeoTranslation(10.5, 2.9, 4.9));
   siliconR_7->SetLineColor(kGreen);

   TGeoVolume *siliconR_8 = gGeoManager->MakeBox("siliconR_8", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_8, 30, new TGeoTranslation(10.5, 2.9, -0.2));
   siliconR_8->SetLineColor(kGreen);

   TGeoVolume *siliconR_9 = gGeoManager->MakeBox("siliconR_9", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_9, 31, new TGeoTranslation(10.5, 2.9, -5.3));
   siliconR_9->SetLineColor(kGreen);

   TGeoVolume *siliconR_10 = gGeoManager->MakeBox("siliconR_10", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_5, 32, new TGeoTranslation(10.5, 2.9, -10.4));
   siliconR_10->SetLineColor(kGreen);

   //

   TGeoVolume *siliconR_11 = gGeoManager->MakeBox("siliconR_11", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_11, 33, new TGeoTranslation(10.5, -2.2, 10.0));
   siliconR_11->SetLineColor(kGreen);

   TGeoVolume *siliconR_12 = gGeoManager->MakeBox("siliconR_12", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_12, 34, new TGeoTranslation(10.5, -2.2, 4.9));
   siliconR_12->SetLineColor(kGreen);

   TGeoVolume *siliconR_13 = gGeoManager->MakeBox("siliconR_13", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_13, 35, new TGeoTranslation(10.5, -2.2, -0.2));
   siliconR_13->SetLineColor(kGreen);

   TGeoVolume *siliconR_14 = gGeoManager->MakeBox("siliconR_14", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_14, 36, new TGeoTranslation(10.5, -2.2, -5.3));
   siliconR_14->SetLineColor(kGreen);

   TGeoVolume *siliconR_15 = gGeoManager->MakeBox("siliconR_15", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_15, 37, new TGeoTranslation(10.5, -2.2, -10.4));
   siliconR_15->SetLineColor(kGreen);

   //

   TGeoVolume *siliconR_16 = gGeoManager->MakeBox("siliconR_16", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_16, 38, new TGeoTranslation(10.5, -7.3, 10.0));
   siliconR_16->SetLineColor(kGreen);

   TGeoVolume *siliconR_17 = gGeoManager->MakeBox("siliconR_17", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_17, 39, new TGeoTranslation(10.5, -7.3, 4.9));
   siliconR_17->SetLineColor(kGreen);

   TGeoVolume *siliconR_18 = gGeoManager->MakeBox("siliconR_18", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_18, 40, new TGeoTranslation(10.5, -7.3, -0.2));
   siliconR_18->SetLineColor(kGreen);

   TGeoVolume *siliconR_19 = gGeoManager->MakeBox("siliconR_19", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_19, 41, new TGeoTranslation(10.5, -7.3, -5.3));
   siliconR_19->SetLineColor(kGreen);

   TGeoVolume *siliconR_20 = gGeoManager->MakeBox("siliconR_20", silicon, 0.07 / 2, 5.0 / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(siliconR_20, 42, new TGeoTranslation(10.5, -7.3, -10.4));
   siliconR_20->SetLineColor(kGreen);
   siliconR_20->SetTransparency(50);

   // gGeoMan->GetVolume(geoVersion)->AddNode(fcplate_in, 1,new TGeoCombiTrans(0.0, 0.0, 0.0, new
   // TGeoRotation("fcplate_in", 0,0, 0))); fcplate_in->SetLineColor(kGrey); drift_volume->SetTransparency(90);

   return drift_volume;
}
