// APOLLO DETECTOR GEOMETRY - Oct 2020
// Questions to: hector.alvarez@usc.es
// in root all sizes are given in cm
// size of first layer GAGG 18x18x20   25 GAGGs
// size of second layer GAGG 20x20x25  16 GAGGs
// gap between gaggs .4 mm
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

// SETUP Option
Bool_t crystalAlongBeamLine = kFALSE;

// Name of geometry version and output file
const TString geoVersion = "RCNP_GAGG";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumHalfSphere = "Aluminum";
const TString MediumVacuum = "vacuum4";
const TString MediumSciLaBr = "LaBr";
const TString MediumSciCsI = "CsI";

////// Parameters for the half sphere holding the scintillators
// Distance of the center of the first detector layer [cm];
const Float_t gagg1_width = 1.8; 
const Float_t gagg1_length = 2.0; 
const Float_t gagg1_z = 123.5; // z position of the first layer
const Float_t distance_between_centers = 1.894;

const Float_t gagg2_width = 2.0;
const Float_t gagg2_length = 2.5;
const Float_t gagg2_z = gagg1_z + gagg1_length/2 + gagg2_length/2 + 3.3;
const Float_t distance_between_centers_2 = 2.11;  // z position of the second layer



// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();

TGeoRotation *fRefRot = NULL;

void RCNP_GAGG()
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

   TGeoMedium *medium = gGeoMan->GetMedium(MediumVacuum);

   TGeoVolume *apollovac = new TGeoVolumeAssembly(geoVersion);
   apollovac->SetMedium(medium);
   top->AddNode(apollovac, 1);

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

   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *CsI = geoMedia->getMedium("CsI");
   FairGeoMedium *LaBr = geoMedia->getMedium("LaBr");
   FairGeoMedium *Aluminum = geoMedia->getMedium("Aluminum");

   // include check if all media are found
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(CsI);
   geoBuild->createMedium(LaBr);
   geoBuild->createMedium(Aluminum);
}

TGeoVolume *create_detector()
{
   // needed materials
   TGeoMedium *medium = gGeoMan->GetMedium(MediumVacuum);
   TGeoMedium *Aluminum = gGeoMan->GetMedium(MediumHalfSphere);
   TGeoMedium *CsI = gGeoMan->GetMedium(MediumSciCsI);
   TGeoMedium *LaBr = gGeoMan->GetMedium(MediumSciLaBr);

   // APOLLO HALF SPHERE (WITH HOLES)
   TGeoRotation *rBottom = new TGeoRotation("rTop", 0, 0,45);
   rBottom->RegisterYourself();
   TGeoVolume *gagg1_1 = gGeoManager->MakeBox("Crystal_1_1", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_1, 20, new TGeoCombiTrans(0, 0, gagg1_z , rBottom));
   gagg1_1->SetLineColor(kGray);

   TGeoVolume *gagg1_2 = gGeoManager->MakeBox("Crystal_1_2", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_2, 20, new TGeoCombiTrans(distance_between_centers/TMath::Sqrt(2), distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_2->SetLineColor(kGray);

   TGeoVolume *gagg1_3 = gGeoManager->MakeBox("Crystal_1_3", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_3, 20, new TGeoCombiTrans(2*distance_between_centers/TMath::Sqrt(2), 2*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_3->SetLineColor(kGray);

   TGeoVolume *gagg1_4 = gGeoManager->MakeBox("Crystal_1_4", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_4, 20, new TGeoCombiTrans( - distance_between_centers/TMath::Sqrt(2), - distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_4->SetLineColor(kGray);

   TGeoVolume *gagg1_5 = gGeoManager->MakeBox("Crystal_1_5", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_5, 20, new TGeoCombiTrans( - 2*distance_between_centers/TMath::Sqrt(2), - 2*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_5->SetLineColor(kGray);

   TGeoVolume *gagg1_6 = gGeoManager->MakeBox("Crystal_1_6", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_6, 20, new TGeoCombiTrans( -distance_between_centers/TMath::Sqrt(2), distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_6->SetLineColor(kGray);

   TGeoVolume *gagg1_7 = gGeoManager->MakeBox("Crystal_1_7", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_7, 20, new TGeoCombiTrans( - 2*distance_between_centers/TMath::Sqrt(2), 2* distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_7->SetLineColor(kGray);

   TGeoVolume *gagg1_8 = gGeoManager->MakeBox("Crystal_1_8", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_8, 20, new TGeoCombiTrans(distance_between_centers/TMath::Sqrt(2), - distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_8->SetLineColor(kGray);

   TGeoVolume *gagg1_9 = gGeoManager->MakeBox("Crystal_1_9", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_9, 20, new TGeoCombiTrans( 2*distance_between_centers/TMath::Sqrt(2), - 2*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_9->SetLineColor(kGray);

   TGeoVolume *gagg1_10 = gGeoManager->MakeBox("Crystal_1_10", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_10, 20, new TGeoCombiTrans(0 , 2 * distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_10->SetLineColor(kGray);

   TGeoVolume *gagg1_11 = gGeoManager->MakeBox("Crystal_1_11", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_11, 20, new TGeoCombiTrans(-distance_between_centers/TMath::Sqrt(2) , 3 * distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_11->SetLineColor(kGray);

   TGeoVolume *gagg1_12 = gGeoManager->MakeBox("Crystal_1_12", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_12, 20, new TGeoCombiTrans(2 * distance_between_centers/TMath::Sqrt(2) , 0 , gagg1_z , rBottom));
   gagg1_12->SetLineColor(kGray);

   TGeoVolume *gagg1_13 = gGeoManager->MakeBox("Crystal_1_13", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_13, 20, new TGeoCombiTrans(3 * distance_between_centers/TMath::Sqrt(2) , -distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_13->SetLineColor(kGray);

   TGeoVolume *gagg1_14 = gGeoManager->MakeBox("Crystal_1_14", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_14, 20, new TGeoCombiTrans(distance_between_centers/TMath::Sqrt(2), 3*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_14->SetLineColor(kGray);

   TGeoVolume *gagg1_15 = gGeoManager->MakeBox("Crystal_1_15", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_15, 20, new TGeoCombiTrans(0, 4*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_15->SetLineColor(kGray);

   TGeoVolume *gagg1_16 = gGeoManager->MakeBox("Crystal_1_16", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_16, 20, new TGeoCombiTrans(3*distance_between_centers/TMath::Sqrt(2), distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_16->SetLineColor(kGray);

   TGeoVolume *gagg1_17 = gGeoManager->MakeBox("Crystal_1_17", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_17, 20, new TGeoCombiTrans(4*distance_between_centers/TMath::Sqrt(2), 0, gagg1_z , rBottom));
   gagg1_17->SetLineColor(kGray);

   TGeoVolume *gagg1_18 = gGeoManager->MakeBox("Crystal_1_18", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_18, 20, new TGeoCombiTrans( - 2*distance_between_centers/TMath::Sqrt(2), 0, gagg1_z , rBottom));
   gagg1_18->SetLineColor(kGray);

   TGeoVolume *gagg1_19 = gGeoManager->MakeBox("Crystal_1_19", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_19, 20, new TGeoCombiTrans( - 3*distance_between_centers/TMath::Sqrt(2), distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_19->SetLineColor(kGray);

   TGeoVolume *gagg1_20 = gGeoManager->MakeBox("Crystal_1_20", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_20, 20, new TGeoCombiTrans( 0, - 2*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_20->SetLineColor(kGray);

   TGeoVolume *gagg1_21 = gGeoManager->MakeBox("Crystal_1_21", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_21, 20, new TGeoCombiTrans(distance_between_centers/TMath::Sqrt(2), - 3*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_21->SetLineColor(kGray);

   TGeoVolume *gagg1_22 = gGeoManager->MakeBox("Crystal_1_22", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_22, 20, new TGeoCombiTrans( - 3*distance_between_centers/TMath::Sqrt(2), - distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_22->SetLineColor(kGray);

   TGeoVolume *gagg1_23 = gGeoManager->MakeBox("Crystal_1_23", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_23, 20, new TGeoCombiTrans( - 4*distance_between_centers/TMath::Sqrt(2), 0, gagg1_z , rBottom));
   gagg1_23->SetLineColor(kGray);

   TGeoVolume *gagg1_24 = gGeoManager->MakeBox("Crystal_1_24", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_24, 20, new TGeoCombiTrans( - distance_between_centers/TMath::Sqrt(2), - 3*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_24->SetLineColor(kGray);

   TGeoVolume *gagg1_25 = gGeoManager->MakeBox("Crystal_1_25", CsI, gagg1_width/2, gagg1_width/2, gagg1_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg1_25, 20, new TGeoCombiTrans( 0, - 4*distance_between_centers/TMath::Sqrt(2), gagg1_z , rBottom));
   gagg1_25->SetLineColor(kGray);


   TGeoVolume *gagg2_1 = gGeoManager->MakeBox("Crystal_2_1", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_1, 20, new TGeoCombiTrans(0,distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_1->SetLineColor(kGray);

   TGeoVolume *gagg2_2 = gGeoManager->MakeBox("Crystal_2_2", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_2, 20, new TGeoCombiTrans(distance_between_centers_2/TMath::Sqrt(2),0, gagg2_z , rBottom));
   gagg2_2->SetLineColor(kGray);

   TGeoVolume *gagg2_3 = gGeoManager->MakeBox("Crystal_2_3", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_3, 20, new TGeoCombiTrans(2*distance_between_centers_2/TMath::Sqrt(2),-distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_3->SetLineColor(kGray);

   TGeoVolume *gagg2_4 = gGeoManager->MakeBox("Crystal_2_4", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_4, 20, new TGeoCombiTrans(-distance_between_centers_2/TMath::Sqrt(2),2*distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_4->SetLineColor(kGray);

   TGeoVolume *gagg2_5 = gGeoManager->MakeBox("Crystal_2_5", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_5, 20, new TGeoCombiTrans(-2*distance_between_centers_2/TMath::Sqrt(2),distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_5->SetLineColor(kGray);

   TGeoVolume *gagg2_6 = gGeoManager->MakeBox("Crystal_2_6", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_6, 20, new TGeoCombiTrans(-3*distance_between_centers_2/TMath::Sqrt(2),0, gagg2_z , rBottom));
   gagg2_6->SetLineColor(kGray);

   TGeoVolume *gagg2_7 = gGeoManager->MakeBox("Crystal_2_7", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_7, 20, new TGeoCombiTrans(0,3*distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_7->SetLineColor(kGray);

    TGeoVolume *gagg2_8 = gGeoManager->MakeBox("Crystal_2_8", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_8, 20, new TGeoCombiTrans(-distance_between_centers_2/TMath::Sqrt(2),0, gagg2_z , rBottom));
   gagg2_8->SetLineColor(kGray);

    TGeoVolume *gagg2_9 = gGeoManager->MakeBox("Crystal_2_9", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_9, 20, new TGeoCombiTrans(-2*distance_between_centers_2/TMath::Sqrt(2),-distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_9->SetLineColor(kGray);

    TGeoVolume *gagg2_10 = gGeoManager->MakeBox("Crystal_2_10", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_10, 20, new TGeoCombiTrans(distance_between_centers_2/TMath::Sqrt(2),2*distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_10->SetLineColor(kGray);

   TGeoVolume *gagg2_11 = gGeoManager->MakeBox("Crystal_2_11", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_11, 20, new TGeoCombiTrans(distance_between_centers_2/TMath::Sqrt(2),-2*distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_11->SetLineColor(kGray);

   TGeoVolume *gagg2_12 = gGeoManager->MakeBox("Crystal_2_12", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_12, 20, new TGeoCombiTrans(0,-3*distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_12->SetLineColor(kGray);

   TGeoVolume *gagg2_13 = gGeoManager->MakeBox("Crystal_2_13", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_13, 20, new TGeoCombiTrans(3*distance_between_centers_2/TMath::Sqrt(2),0, gagg2_z , rBottom));
   gagg2_13->SetLineColor(kGray);

   TGeoVolume *gagg2_14 = gGeoManager->MakeBox("Crystal_2_14", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_14, 20, new TGeoCombiTrans(0,-distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_14->SetLineColor(kGray);

   TGeoVolume *gagg2_15 = gGeoManager->MakeBox("Crystal_2_15", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_15, 20, new TGeoCombiTrans(-distance_between_centers_2/TMath::Sqrt(2),-2*distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_15->SetLineColor(kGray);

   TGeoVolume *gagg2_16 = gGeoManager->MakeBox("Crystal_2_16", CsI, gagg2_width/2, gagg2_width/2, gagg2_length/2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(gagg2_16, 20, new TGeoCombiTrans(2*distance_between_centers_2/TMath::Sqrt(2),distance_between_centers_2/TMath::Sqrt(2), gagg2_z , rBottom));
   gagg2_16->SetLineColor(kGray);






   return gagg1_1;
}
