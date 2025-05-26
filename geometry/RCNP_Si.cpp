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
const TString geoVersion = "RCNP_Si";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumSi = "silicon";
const TString MediumSteel = "steel";
const TString MediumPCB = "pcbmvd";
const TString MediumVacuum = "vacuum4";
const TString CellMedium = "3H_5torr"; // For testing
const TString TargetMedium = "CD2_polyethylene";

const Float_t x_off = 2.3;                      // Distance between Si in the same YZ plane
const Float_t y_off = 2.3;                      // Distance between Si in the same XZ plane
const Float_t Recoil_Z_dist_from_target = 33.0; // Distance of the recoil detector with respect to the target
const Float_t Si_width = 9.722;                   // Width of Si detectors
const Float_t Array_length = 1.0;              //
const float PCB_thickness = 0.5;                // cm
float blocker_length = 5.0;                     //
const Float_t Z_dist_from_target =
   Recoil_Z_dist_from_target + PCB_thickness + blocker_length + Array_length -
   4.0; // cm (if positive, the length of the array/2.0 (38) has to be added) 50+38/2+5/2(recoil det thickness)

// some global variables
TGeoManager *gGeoMan = new TGeoManager("HELIOS", "HELIOS");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void RCNP_Si()
{

   // Load needed material definition from media.geo file
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
   FairGeoMedium *tritium = geoMedia->getMedium("3H_5torr");
   FairGeoMedium *CD2 = geoMedia->getMedium("CD2_polyethylene");

   // include check if all media are found

   geoBuild->createMedium(silicon);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(pcb);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(tritium);
   geoBuild->createMedium(CD2);
}

TGeoVolume *create_detector()
{

   // needed materials
   TGeoMedium *silicon = gGeoMan->GetMedium(MediumSi);
   TGeoMedium *steel = gGeoMan->GetMedium(MediumSteel);
   TGeoMedium *pcb = gGeoMan->GetMedium(MediumPCB);
   TGeoMedium *tritiumgas = gGeoMan->GetMedium(CellMedium);
   TGeoMedium *cd2 = gGeoMan->GetMedium(TargetMedium);
   TGeoMedium *vacuum = gGeoMan->GetMedium(MediumVacuum);

   TGeoRotation *rBottom = new TGeoRotation("rTop", 0, 0, 45);
   rBottom->RegisterYourself();

   TGeoVolume *silicon1 = gGeoManager->MakeBox("silicon1", silicon, Si_width / 2, Si_width / 2, 0.025);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(silicon1, 1, new TGeoCombiTrans(0, 0, 120.0 , rBottom));
   silicon1->SetLineColor(kGray);

   TGeoVolume *silicon2 = gGeoManager->MakeBox("silicon2", silicon, Si_width / 2, Si_width / 2, 0.05);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(silicon2, 2, new TGeoCombiTrans(0, 0, 120.5525, rBottom));
   silicon2->SetLineColor(kGray);


   return silicon1;
}
