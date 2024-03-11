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
const TString geoVersion = "rcnp_130matm";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "rcnpATTPCIsoButane";
const TString MediumVacuum = "vacuum4";
const TString MediumWindow = "aramid";
const TString CylinderVolumeMedium = "steel";

/*** Unused parameters for IC ***
const TString ICWindowMedium = "ICpolypropylene";
const TString ICAlWindowMedium = "Aluminum";
const TString ICMediumGas     = "ICIsoButane";
*/

// Distance of the center of the first detector layer [cm];
const Float_t First_Z_Position = 10;
const Float_t Z_Distance = 10;

// Silicon box for both module types
const Float_t tpc_diameter = 50.;
const Float_t drift_length = 100.;

// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void rcnp_130matm()
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

   FairGeoMedium *ATTPCGas = geoMedia->getMedium(MediumGas);
   FairGeoMedium *steel = geoMedia->getMedium("steel");
   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *aramid = geoMedia->getMedium("aramid");

   /***** Unused IC media *****
   FairGeoMedium* ICpolypropylene   = geoMedia->getMedium("ICpolypropylene");
   FairGeoMedium* ICIsoButane       = geoMedia->getMedium("ICIsoButane");
   FairGeoMedium* Aluminium         = geoMedia->getMedium("Aluminum");
   geoBuild->createMedium(ICpolypropylene);
   geoBuild->createMedium(ICIsoButane);
   geoBuild->createMedium(Aluminium);
   */

   // include check if all media are found
   geoBuild->createMedium(ATTPCGas);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(aramid);
}

TGeoVolume *create_detector()
{

   // needed materials
   TGeoMedium *OuterCylinder = gGeoMan->GetMedium(CylinderVolumeMedium);
   TGeoMedium *gas = gGeoMan->GetMedium(MediumGas);
   TGeoMedium *windowmat = gGeoMan->GetMedium(MediumWindow);

   /*** Unused IC media ***
   TGeoMedium* ICgas         = gGeoMan->GetMedium(ICMediumGas);
   TGeoMedium* ICwindowmat   = gGeoMan->GetMedium(ICWindowMedium);
   TGeoMedium* ICAlwindowmat = gGeoMan->GetMedium(ICAlWindowMedium);
   */

   // ATTPC Main drift volume

   double tpc_rot = 0;

   TGeoVolume *drift_volume = gGeoManager->MakeTube("drift_volume", gas, 0, tpc_diameter / 2., drift_length / 2.);
   // TGeoVolume *drift_volume = gGeoManager->MakeBox("drift_volume", gas,  100./2, 100./2, 100./2);
   // gGeoMan->GetVolume(geoVersion)->AddNode(drift_volume,1, new TGeoTranslation(0,0,drift_length/2));

   gGeoMan->GetVolume(geoVersion)
      ->AddNode(drift_volume, 1,
                new TGeoCombiTrans(0.0, 0.0, drift_length / 2.0, new TGeoRotation("drift_volume", 0, tpc_rot, 0)));
   drift_volume->SetTransparency(80);

   // ATTPC Window
   TGeoVolume *tpc_window = gGeoManager->MakeTube("tpc_window", windowmat, 0, 1.00 / 2.0, 0.00036 / 2.0);
   tpc_window->SetLineColor(kBlue);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(tpc_window, 1, new TGeoCombiTrans(0.0, 0.0, 0.0, new TGeoRotation("tpc_window", 0, tpc_rot, 0)));
   tpc_window->SetTransparency(50);

   // ATTPC Vessel
   TGeoVolume *vessel_volume = gGeoManager->MakeTube("vessel_volume", OuterCylinder, tpc_diameter / 2.,
                                                     (tpc_diameter + 2.0) / 2., drift_length / 2.);

   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vessel_volume, 1,
                new TGeoCombiTrans(0.0, 0.0, drift_length / 2.0, new TGeoRotation("vessel_volume", 0, tpc_rot, 0)));
   vessel_volume->SetTransparency(90);

   /*** Unused IC Construction ***
   // IC Drift Volumes

   TGeoVolume *IC_drift_volume_1 = gGeoManager->MakeTube("IC_drift_volume_Out",ICgas,0,3.81/2.0,2.75/2.0);//
   IC_drift_volume_1->SetLineColor(kRed);
   gGeoMan->GetVolume(geoVersion)->AddNode(IC_drift_volume_1,1,new
   TGeoCombiTrans(0.0,0.0,-50.0+1.375+(0.5000005/2.0),new TGeoRotation("IC_drift_volume_Out",0,0,0)));
   IC_drift_volume_1->SetTransparency(80);

   TGeoVolume *IC_drift_volume_2 = gGeoManager->MakeTube("IC_drift_volume_In",ICgas,0,3.81/2.0,2.75/2.0);//
   IC_drift_volume_2->SetLineColor(kRed);
   gGeoMan->GetVolume(geoVersion)->AddNode(IC_drift_volume_2,1,new
   TGeoCombiTrans(0.0,0.0,-50.0-1.375-(0.5000005/2.0),new TGeoRotation("IC_drift_volume_In",0,0,0)));
   IC_drift_volume_2->SetTransparency(80);

   // IC polypropylene windows

     TGeoVolume *IC_window_1 = gGeoManager->MakeTube("IC_window_Mid",ICwindowmat,0,3.81/2.0,0.00005/2.0);
     IC_window_1->SetLineColor(kPink);
     IC_window_1->SetTransparency(80);
     gGeoMan->GetVolume(geoVersion)->AddNode(IC_window_1,1,new TGeoCombiTrans(0.0,0.0,-50.0,new
   TGeoRotation("IC_window_Mid",0,0,0)));

     TGeoVolume *IC_window_2 = gGeoManager->MakeTube("IC_window_Out",ICwindowmat,0,3.81/2.0,0.00005/2.0);
     IC_window_2->SetLineColor(kPink);
     IC_window_2->SetTransparency(80);
     gGeoMan->GetVolume(geoVersion)->AddNode(IC_window_2,1,new TGeoCombiTrans(0.0,0.0,-50.0+6.0/2.0,new
   TGeoRotation("IC_window_Out",0,0,0)));

     TGeoVolume *IC_window_3 = gGeoManager->MakeTube("IC_window_In",ICwindowmat,0,3.81/2.0,0.00005/2.0);
     IC_window_3->SetLineColor(kPink);
     IC_window_3->SetTransparency(80);
     gGeoMan->GetVolume(geoVersion)->AddNode(IC_window_3,1,new TGeoCombiTrans(0.0,0.0,-50.0-6.0/2.0,new
   TGeoRotation("IC_window_In",0,0,0)));

    // IC Aluminum layers

     TGeoVolume *IC_Al_window_1 = gGeoManager->MakeTube("IC_window_1_Mid_Al",ICAlwindowmat,0,3.81/2.0,0.000005/2.0);
     IC_Al_window_1->SetLineColor(kBlue+3);
     IC_Al_window_1->SetTransparency(50);
     gGeoMan->GetVolume(geoVersion)->AddNode(IC_Al_window_1,1,new TGeoCombiTrans(0.0,0.0,-50.0+0.050005,new
   TGeoRotation("IC_window_1_Mid_Al",0,0,0)));

     TGeoVolume *IC_Al_window_2 = gGeoManager->MakeTube("IC_window_2_Mid_Al",ICAlwindowmat,0,3.81/2.0,0.000005/2.0);
     IC_Al_window_2->SetLineColor(kBlue+3);
     IC_Al_window_2->SetTransparency(50);
     gGeoMan->GetVolume(geoVersion)->AddNode(IC_Al_window_2,1,new TGeoCombiTrans(0.0,0.0,-50.0-0.050005,new
   TGeoRotation("IC_window_2_Mid_Al",0,0,0)));

    TGeoVolume *IC_Al_window_3 = gGeoManager->MakeTube("IC_Al_window_Out",ICAlwindowmat,0,3.81/2.0,0.000005/2.0);
     IC_Al_window_3->SetLineColor(kBlue+3);
     IC_Al_window_3->SetTransparency(50);
     gGeoMan->GetVolume(geoVersion)->AddNode(IC_Al_window_3,1,new TGeoCombiTrans(0.0,0.0,-50.0+6.0/2.0+0.050005/2.0,new
   TGeoRotation("IC_Al_window_Out",0,0,0)));

     TGeoVolume *IC_Al_window_4 = gGeoManager->MakeTube("IC_Al_window_In",ICAlwindowmat,0,3.81/2.0,0.000005/2.0);
     IC_Al_window_4->SetLineColor(kBlue+3);
     IC_Al_window_4->SetTransparency(50);
     gGeoMan->GetVolume(geoVersion)->AddNode(IC_Al_window_4,1,new TGeoCombiTrans(0.0,0.0,-50.0-6.0/2.0-0.050005/2.0,new
   TGeoRotation("IC_Al_window_In",0,0,0)));
   */

   return drift_volume;
}

void position_detector()
{

   /* TGeoTranslation* det_trans=NULL;

    Int_t numDets=0;
    for (Int_t detectorPlanes = 0; detectorPlanes < 40; detectorPlanes++) {
      det_trans
        = new TGeoTranslation("", 0., 0., First_Z_Position+(numDets*Z_Distance));
      gGeoMan->GetVolume(geoVersion)->AddNode(gModules, numDets, det_trans);
      numDets++;

    }*/
}

void add_alignable_volumes()
{

   /* TString volPath;
    TString symName;
    TString detStr   = "Tutorial4/det";
    TString volStr   = "/TOP_1/tutorial4_1/tut4_det_";

    for (Int_t detectorPlanes = 0; detectorPlanes < 40; detectorPlanes++) {

      volPath  = volStr;
      volPath += detectorPlanes;

      symName  = detStr;
      symName += Form("%02d",detectorPlanes);

      cout<<"Path: "<<volPath<<", "<<symName<<endl;
  //    gGeoMan->cd(volPath);

      gGeoMan->SetAlignableEntry(symName.Data(),volPath.Data());

    }
      cout<<"Nr of alignable objects: "<<gGeoMan->GetNAlignable()<<endl;*/
}
