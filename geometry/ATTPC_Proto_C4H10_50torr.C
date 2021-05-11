/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
// in root all sizes are given in cm

#include "TSystem.h"
#include "TGeoManager.h"
#include "TGeoVolume.h"
#include "TGeoMaterial.h"
#include "TGeoMedium.h"
#include "TGeoPgon.h"
#include "TGeoMatrix.h"
#include "TGeoCompositeShape.h"
#include "TFile.h"
#include "TString.h"
#include "TList.h"
#include "TROOT.h"

#include <iostream>

// Name of geometry version and output file
const TString geoVersion = "ATTPC_Proto_C4H10_50torr";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "ATTPCIsoButane_50";
const TString CylinderVolumeMedium = "steel";
const TString MediumVacuum = "vacuum4";
const TString FieldCageCyl = "G10";
const TString fc_rings = "Aluminum";

// Distance of the center of the first detector layer [cm];
const Float_t First_Z_Position = 10;
const Float_t Z_Distance = 10;

// Field cage
const Float_t tpc_diameter = 28.;
const Float_t drift_length = 50.;

// Field cage container G10
const Float_t fc_inner_diameter = 28.;
const Float_t fc_outer_diameter = 28.5;
const Float_t fc_drift_length = 50.;

// Chamber
const Float_t chamber_diameter = 40.;
const Float_t chamber_length = 70.;
const Float_t chamber_thickness = 39.; // 40 cm - 1 cm of thickness

// FC rings
const Float_t fc_inner_ring_idiam = 0.0;
const Float_t fc_inner_ring_odiam = 0.5;
const Float_t fc_inner_ring_diam = 26.0;

const Float_t fc_outer_ring_idiam = 0.0;
const Float_t fc_outer_ring_odiam = 0.5;
const Float_t fc_outer_ring_diam = 29.5;

// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC_Proto", "ATTPC_Proto");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void ATTPC_Proto_C4H10_50torr()
{
   // Load the necessary FairRoot libraries
   // gROOT->LoadMacro("$VMCWORKDIR/gconfig/basiclibs.C");
   // basiclibs();
   // gSystem->Load("libGeoBase");
   // gSystem->Load("libParBase");
   // gSystem->Load("libBase");

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

   FairGeoMedium *ATTPCIsoButane_50 = geoMedia->getMedium("ATTPCIsoButane_50");
   FairGeoMedium *isobutan = geoMedia->getMedium("isobutan");
   FairGeoMedium *steel = geoMedia->getMedium("steel");
   FairGeoMedium *heco2 = geoMedia->getMedium("heco2");
   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *G10 = geoMedia->getMedium("G10");
   FairGeoMedium *Aluminum = geoMedia->getMedium("Aluminum");

   // include check if all media are found

   geoBuild->createMedium(ATTPCIsoButane_50);
   geoBuild->createMedium(isobutan);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(heco2);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(G10);
   geoBuild->createMedium(Aluminum);
}

TGeoVolume *create_detector()
{

   // needed materials
   TGeoMedium *OuterCylinder = gGeoMan->GetMedium(CylinderVolumeMedium);
   TGeoMedium *gas = gGeoMan->GetMedium(MediumGas);
   TGeoMedium *fc = gGeoMan->GetMedium(FieldCageCyl);
   TGeoMedium *fc_rings_med = gGeoMan->GetMedium(fc_rings);

   TGeoVolume *drift_volume = gGeoManager->MakeTube("drift_volume", gas, 0, tpc_diameter / 2, drift_length / 2);
   // TGeoVolume *drift_volume = gGeoManager->MakeBox("drift_volume", gas,  100./2, 100./2, 100./2);
   gGeoMan->GetVolume(geoVersion)->AddNode(drift_volume, 1, new TGeoTranslation(0, 0, drift_length / 2));
   drift_volume->SetTransparency(80);

   TGeoVolume *fc_volume =
      gGeoManager->MakeTube("fc_volume", fc, fc_inner_diameter / 2, fc_outer_diameter / 2, fc_drift_length / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(fc_volume, 1, new TGeoTranslation(0, 0, fc_drift_length / 2));
   fc_volume->SetTransparency(50);
   fc_volume->SetLineColor(kYellow - 10);

   TGeoVolume *chamber_volume = gGeoManager->MakeTube("chamber_volume", OuterCylinder, chamber_thickness / 2,
                                                      chamber_diameter / 2, chamber_length / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(chamber_volume, 1, new TGeoTranslation(0, 0, chamber_length / 4));

   TGeoVolume *fc_inner_ring = gGeoManager->MakeTorus("fc_inner_ring", fc_rings_med, fc_inner_ring_diam / 2.0,
                                                      fc_inner_ring_idiam / 2.0, fc_inner_ring_odiam / 2.0);
   fc_inner_ring->SetLineColor(kMagenta + 2);

   TGeoVolume *fc_outer_ring = gGeoManager->MakeTorus("fc_outer_ring", fc_rings_med, fc_outer_ring_diam / 2.0,
                                                      fc_outer_ring_idiam / 2.0, fc_outer_ring_odiam / 2.0);
   fc_outer_ring->SetLineColor(kRed + 2);

   /*  for(Int_t i=0;i<50;i++){
         gGeoMan->GetVolume(geoVersion)->AddNode(fc_inner_ring,i, new TGeoTranslation(0,0,0.5+i));
         fc_inner_ring->SetTransparency(50);
         gGeoMan->GetVolume(geoVersion)->AddNode(fc_outer_ring,i, new TGeoTranslation(0,0,0.5+i));
         fc_outer_ring->SetTransparency(50);

     }*/

   // Single detector_layer
   /* TGeoBBox* det_plane = new TGeoBBox("", Module_Size_X/2., Module_Size_Y/2., Module_Size_Z/2.);
    TGeoVolume* det_plane_vol =
      new TGeoVolume("tut4_det", det_plane, SiliconVolMed);
    det_plane_vol->SetLineColor(kBlue); // set line color
    det_plane_vol->SetTransparency(70); // set transparency
    TGeoTranslation* det_plane_trans
      = new TGeoTranslation("", 0., 0., 0.);

    return det_plane_vol;*/

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
