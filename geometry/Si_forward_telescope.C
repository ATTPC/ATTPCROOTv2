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
const TString geoVersion = "Si_forward_telescope_v1.0";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumSi = "silicon";
const TString MediumCsI = "csi";
const TString MediumSteel = "steel";
const TString MediumPCB = "pcbmvd";
const TString MediumVacuum = "vacuum4";

TGeoManager *gGeoMan = new TGeoManager("ATTPCSi", "ATTPCSi");// Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

void create_materials_from_media_file();
TGeoVolume *create_detector();

void Si_forward_telescope()
{ 


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
   FairGeoMedium *csi = geoMedia->getMedium("csi");
   

   // include check if all media are found

   geoBuild->createMedium(silicon);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(pcb);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(csi);
   
}

TGeoVolume *create_detector()
{


}
