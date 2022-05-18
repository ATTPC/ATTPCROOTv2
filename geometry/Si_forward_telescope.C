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
const TString geoVersion = "Si_forward_telescope_v1.0";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumSi = "silicon";
const TString MediumCsI = "CsI";
const TString MediumSteel = "steel";
const TString MediumPCB = "pcbmvd";
const TString MediumVacuum = "vacuum4";

TGeoManager *gGeoMan = new TGeoManager("ATTPCSi", "ATTPCSi"); // Pointer to TGeoManager instance
TGeoVolume *gModules;                                         // Global storage for module types

Double_t xSize = 10.0;
Double_t ySize = 10.0;
Double_t zDist = 120.0;

void create_materials_from_media_file();
TGeoVolume *create_detector();

void Si_forward_telescope()
{

   create_materials_from_media_file();

   // Get the GeoManager for later usage
   gGeoMan = (TGeoManager *)gROOT->FindObject("FAIRGeom");
   gGeoMan->SetVisLevel(7);

   TGeoVolume *top = new TGeoVolumeAssembly("TOP");
   gGeoMan->SetTopVolume(top);

   TGeoMedium *vac = gGeoMan->GetMedium(MediumVacuum);
   TGeoVolume *topvac = new TGeoVolumeAssembly(geoVersion);
   topvac->SetMedium(vac);
   top->AddNode(topvac, 1);

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

   FairGeoMedium *pcb = geoMedia->getMedium("pcbmvd");
   FairGeoMedium *steel = geoMedia->getMedium("steel");
   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *silicon = geoMedia->getMedium("silicon");
   FairGeoMedium *csi = geoMedia->getMedium("CsI");

   // include check if all media are found

   geoBuild->createMedium(silicon);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(pcb);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(csi);
}

TGeoVolume *create_detector()
{
   TGeoMedium *silicon = gGeoMan->GetMedium(MediumSi);
   TGeoMedium *steel = gGeoMan->GetMedium(MediumSteel);
   TGeoMedium *pcb = gGeoMan->GetMedium(MediumPCB);
   TGeoMedium *csi = gGeoMan->GetMedium(MediumCsI);
   TGeoMedium *vacuum = gGeoMan->GetMedium(MediumVacuum);

   // YZ plane (left beam side)
   TGeoVolume *dESi = gGeoManager->MakeBox("dESi", silicon, xSize / 2, ySize / 2, 0.05 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(dESi, 0, new TGeoTranslation(0.0, 0.0, 0.0 + zDist));
   dESi->SetLineColor(kGreen);

   TGeoVolume *ESi = gGeoManager->MakeBox("ESi", silicon, xSize / 2, ySize / 2, 0.1 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(ESi, 0, new TGeoTranslation(0.0, 0.0, 0.1 + zDist));
   ESi->SetLineColor(kGreen);

   TGeoVolume *ECsI = gGeoManager->MakeBox("CsI", csi, xSize / 2, ySize / 2, 4.0 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(ECsI, 0, new TGeoTranslation(0.0, 0.0, 2.5 + zDist));
   ECsI->SetLineColor(kYellow - 9);

   return dESi;
}
