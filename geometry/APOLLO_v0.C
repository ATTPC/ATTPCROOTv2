// APOLLO DETECTOR GEOMETRY - Oct 2020
// Questions to: hector.alvarez@usc.es
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

// SETUP Option
Bool_t crystalAlongBeamLine = kFALSE;

// Name of geometry version and output file
const TString geoVersion = "APOLLO_v0";
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
const Float_t HalfSphere_innerRadius = 11.176;
const Float_t HalfSphere_outerRadius = 12.7;
// THAT IS NOT EXACT. PIECE WILL HAVE A LATERAL TILT ANGLE.
// TO MAKE IT EXACT, ADD INNER AND OUTER SPHERICAL SURFACES FINISH AT
// DIFFERENT ANGLES. IMPLEMENT CORRECTION IF NEEDED.
const Float_t HalfSphere_minAngle = 0;
const Float_t HalfSphere_maxAngle = 108.553;
const Float_t HalfSphere_holeRadius = 5.5118 / 2;

////// Parameters for scintillators
const Float_t Sci_radius = 2.54; // assuming 1 inch radius, 3 inch long
const Float_t Sci_halfLength = 7.62 / 2;
const Float_t Sci_eccentricity = 1.8; // displacement wrt center at half sphere holes
const Float_t SciWrap_radius = 2.64;
const Float_t SciWrap_halfLength = 7.82 / 2;

// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();

TGeoRotation *fRefRot = NULL;

void APOLLO_v0()
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
   // gGeoMan->SetVisLevel(7);

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
   TGeoShape *pSphereBulk = new TGeoSphere("SphereBulk", HalfSphere_innerRadius, HalfSphere_outerRadius,
                                           HalfSphere_minAngle, HalfSphere_maxAngle, 0, 360);

   TGeoShape *pSphereHole1 = new TGeoTube("SphereHole1",         // hole to accommodate the crytals
                                          0.,                    // Rmin
                                          HalfSphere_holeRadius, // Rmax
                                          2.);                   // half length

   TGeoTranslation *refTrans =
      new TGeoTranslation("refTrans", 0., 0., (HalfSphere_innerRadius + HalfSphere_outerRadius) / 2);
   TGeoCombiTrans *hole0 =
      new TGeoCombiTrans("hole0", 0., 0., (HalfSphere_innerRadius + HalfSphere_outerRadius) / 2, fRefRot);
   hole0->RegisterYourself();

   // INTRODUCING 26 HOLES WITH THE PROPER ROTATIONS AND TRANSLATIONS
   TGeoCombiTrans **hole1 = new TGeoCombiTrans *[5];
   TGeoRotation *geoRot_1;
   TRotation rot_1[5];
   TVector3 tran_1[5];
   TString hole1Name = "hole1_";
   for (Int_t i = 0; i < 5; i++) {
      tran_1[i].SetX(0.);
      tran_1[i].SetY(0.);
      tran_1[i].SetZ((HalfSphere_innerRadius + HalfSphere_outerRadius) / 2);
      // POLAR 30 degrees, 5 different azimuthal regular angles
      rot_1[i].SetXEulerAngles(0, TMath::Pi() / 6, i * 2 * TMath::Pi() / 5);
      geoRot_1 = new TGeoRotation("geoRot_1", i * 72, 30, 0);
      tran_1[i] = rot_1[i] * tran_1[i];
      // std::cout << tran_2[i].X() <<" "<< tran_2[i].Y() <<" "<< tran_2[i].Z() << std::endl;
      string str = to_string(i);
      hole1[i] = new TGeoCombiTrans(hole1Name + str.c_str(), tran_1[i].X(), tran_1[i].Y(), tran_1[i].Z(), geoRot_1);

      hole1[i]->RegisterYourself();
   }

   TGeoCombiTrans **hole2 = new TGeoCombiTrans *[10];
   TGeoRotation *geoRot_2;
   TRotation rot_2[10];
   TVector3 tran_2[10];
   TString hole2Name = "hole2_";
   for (Int_t i = 0; i < 10; i++) {
      tran_2[i].SetX(0.);
      tran_2[i].SetY(0.);
      tran_2[i].SetZ((HalfSphere_innerRadius + HalfSphere_outerRadius) / 2);
      // POLAR 60 degrees, 10 different azimuthal regular angles with shift
      rot_2[i].SetXEulerAngles(0, TMath::Pi() / 3, TMath::Pi() / 10 + i * TMath::Pi() / 5);
      geoRot_2 = new TGeoRotation("geoRot_2", 18 + i * 36, 60, 0);
      tran_2[i] = rot_2[i] * tran_2[i];

      string str = to_string(i);
      hole2[i] = new TGeoCombiTrans(hole2Name + str.c_str(), tran_2[i].X(), tran_2[i].Y(), tran_2[i].Z(), geoRot_2);
      hole2[i]->RegisterYourself();
   }

   TGeoCombiTrans **hole3 = new TGeoCombiTrans *[10];
   TGeoRotation *geoRot_3;
   TRotation rot_3[10];
   TVector3 tran_3[10];
   TString hole3Name = "hole3_";
   for (Int_t i = 0; i < 10; i++) {
      tran_3[i].SetX(0.);
      tran_3[i].SetY(0.);
      tran_3[i].SetZ((HalfSphere_innerRadius + HalfSphere_outerRadius) / 2);
      // POLAR 60 degrees, 90 different azimuthal regular angles
      rot_3[i].SetXEulerAngles(0, TMath::Pi() / 2, i * TMath::Pi() / 5);
      geoRot_3 = new TGeoRotation("geoRot_3", i * 36, 90, 0);
      tran_3[i] = rot_3[i] * tran_3[i];

      string str = to_string(i);
      hole3[i] = new TGeoCombiTrans(hole3Name + str.c_str(), tran_3[i].X(), tran_3[i].Y(), tran_3[i].Z(), geoRot_3);

      hole3[i]->RegisterYourself();
   }

   TString command = "SphereBulk ";
   TString command0 = " - (SphereHole1:hole0 +";
   TString command1 =
      " SphereHole1:hole1_0 + SphereHole1:hole1_1 + SphereHole1:hole1_2 + SphereHole1:hole1_3 + SphereHole1:hole1_4 +";
   TString command2 =
      " SphereHole1:hole2_0 + SphereHole1:hole2_1 + SphereHole1:hole2_2 + SphereHole1:hole2_3 + SphereHole1:hole2_4 + "
      "SphereHole1:hole2_5 + SphereHole1:hole2_6 + SphereHole1:hole2_7 + SphereHole1:hole2_8 + SphereHole1:hole2_9 +";
   TString command3 =
      " SphereHole1:hole3_0 + SphereHole1:hole3_1 + SphereHole1:hole3_2 + SphereHole1:hole3_3 + SphereHole1:hole3_4 + "
      "SphereHole1:hole3_5 + SphereHole1:hole3_6 + SphereHole1:hole3_7 + SphereHole1:hole3_8 + SphereHole1:hole3_9)";

   TGeoCompositeShape *pCSSphere =
      new TGeoCompositeShape("halfSphere_CS", command + command0 + command1 + command2 + command3);
   TGeoVolume *halfSphere = new TGeoVolume("halfSphere", pCSSphere, Aluminum);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(halfSphere, 0, new TGeoCombiTrans(0., 0., 0., new TGeoRotation("halfSphere", 0, 0, 0)));
   // halfSphere->SetTransparency(80);

   // 26 CsI or LaBr CRYSTALS WITH "WRAPPING" CENTERED IN THE HALFSPHERE HOLES
   TGeoVolume **Cry_vol_wrap;
   Cry_vol_wrap = new TGeoVolume *[26];
   TGeoVolume **Cry_vol;
   Cry_vol = new TGeoVolume *[26];

   TString CrystalWrapName = "CrystalWrap_";
   TString CrystalName = "Crystal_";
   TString name_cry[26] = {"01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13",
                           "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26"};

   Cry_vol[0] = gGeoManager->MakeTube(CrystalName + name_cry[0], LaBr, 0, Sci_radius, Sci_halfLength);
   Cry_vol[0]->SetVisLeaves(kTRUE);
   Cry_vol[0]->SetVisibility(kTRUE);
   Cry_vol[0]->SetVisContainers(kTRUE);
   Cry_vol[0]->SetLineColor(kMagenta);
   Cry_vol_wrap[0] =
      gGeoManager->MakeTube(CrystalWrapName + name_cry[0], Aluminum, 0, SciWrap_radius, SciWrap_halfLength);
   Cry_vol_wrap[0]->SetVisLeaves(kTRUE);
   Cry_vol_wrap[0]->SetVisibility(kTRUE);
   Cry_vol_wrap[0]->SetVisContainers(kTRUE);
   Cry_vol_wrap[0]->SetLineColor(kBlue);
   Cry_vol_wrap[0]->AddNode(Cry_vol[0], 1, new TGeoCombiTrans(0., 0., 0., fRefRot));
   if (crystalAlongBeamLine)
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol_wrap[0], 1,
                   new TGeoCombiTrans(
                      0., 0., (HalfSphere_innerRadius + HalfSphere_outerRadius) / 2 + Sci_halfLength - Sci_eccentricity,
                      fRefRot));

   for (Int_t i = 0; i < 5; i++) {
      Cry_vol[i + 1] = gGeoManager->MakeTube(CrystalName + name_cry[i + 1], LaBr, 0, Sci_radius, Sci_halfLength);
      Cry_vol[i + 1]->SetVisLeaves(kTRUE);
      Cry_vol[i + 1]->SetVisibility(kTRUE);
      Cry_vol[i + 1]->SetVisContainers(kTRUE);
      Cry_vol[i + 1]->SetLineColor(kMagenta);
      Cry_vol_wrap[i + 1] =
         gGeoManager->MakeTube(CrystalWrapName + name_cry[i + 1], Aluminum, 0, SciWrap_radius, SciWrap_halfLength);
      Cry_vol_wrap[i + 1]->SetVisLeaves(kTRUE);
      Cry_vol_wrap[i + 1]->SetVisibility(kTRUE);
      Cry_vol_wrap[i + 1]->SetVisContainers(kTRUE);
      Cry_vol_wrap[i + 1]->SetLineColor(kBlue);
      tran_1[i].SetX(0.);
      tran_1[i].SetY(0.);
      tran_1[i].SetZ((HalfSphere_innerRadius + HalfSphere_outerRadius) / 2 + Sci_halfLength - Sci_eccentricity);
      rot_1[i].SetXEulerAngles(0, TMath::Pi() / 6, i * 2 * TMath::Pi() / 5);
      geoRot_1 = new TGeoRotation("geoRot_1", i * 72, 30, 0);
      tran_1[i] = rot_1[i] * tran_1[i];
      Cry_vol_wrap[i + 1]->AddNode(Cry_vol[i + 1], i + 2, new TGeoCombiTrans(0., 0., 0., fRefRot));
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol_wrap[i + 1], i + 2,
                   new TGeoCombiTrans(tran_1[i].X(), tran_1[i].Y(), tran_1[i].Z(), geoRot_1));
   }

   for (Int_t i = 0; i < 10; i++) {
      Cry_vol[i + 6] = gGeoManager->MakeTube(CrystalName + name_cry[i + 6], CsI, 0, Sci_radius, Sci_halfLength);
      Cry_vol[i + 6]->SetVisLeaves(kTRUE);
      Cry_vol[i + 6]->SetVisibility(kTRUE);
      Cry_vol[i + 6]->SetVisContainers(kTRUE);
      Cry_vol[i + 6]->SetLineColor(kMagenta);
      Cry_vol_wrap[i + 6] =
         gGeoManager->MakeTube(CrystalWrapName + name_cry[i + 6], Aluminum, 0, SciWrap_radius, SciWrap_halfLength);
      Cry_vol_wrap[i + 6]->SetVisLeaves(kTRUE);
      Cry_vol_wrap[i + 6]->SetVisibility(kTRUE);
      Cry_vol_wrap[i + 6]->SetVisContainers(kTRUE);
      Cry_vol_wrap[i + 6]->SetLineColor(kBlue);
      tran_2[i].SetX(0.);
      tran_2[i].SetY(0.);
      tran_2[i].SetZ((HalfSphere_innerRadius + HalfSphere_outerRadius) / 2 + Sci_halfLength - Sci_eccentricity);
      rot_2[i].SetXEulerAngles(0, TMath::Pi() / 3, TMath::Pi() / 10 + i * TMath::Pi() / 5);
      geoRot_2 = new TGeoRotation("geoRot_2", 18 + i * 36, 60, 0);
      tran_2[i] = rot_2[i] * tran_2[i];
      Cry_vol_wrap[i + 6]->AddNode(Cry_vol[i + 6], i + 7, new TGeoCombiTrans(0., 0., 0., fRefRot));
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol_wrap[i + 6], i + 7,
                   new TGeoCombiTrans(tran_2[i].X(), tran_2[i].Y(), tran_2[i].Z(), geoRot_2));
   }

   for (Int_t i = 0; i < 10; i++) {
      Cry_vol[i + 16] = gGeoManager->MakeTube(CrystalName + name_cry[i + 16], CsI, 0, Sci_radius, Sci_halfLength);
      Cry_vol[i + 16]->SetVisLeaves(kTRUE);
      Cry_vol[i + 16]->SetVisibility(kTRUE);
      Cry_vol[i + 16]->SetVisContainers(kTRUE);
      Cry_vol[i + 16]->SetLineColor(kMagenta);
      Cry_vol_wrap[i + 16] =
         gGeoManager->MakeTube(CrystalWrapName + name_cry[i + 16], Aluminum, 0, SciWrap_radius, SciWrap_halfLength);
      Cry_vol_wrap[i + 16]->SetVisLeaves(kTRUE);
      Cry_vol_wrap[i + 16]->SetVisibility(kTRUE);
      Cry_vol_wrap[i + 16]->SetVisContainers(kTRUE);
      Cry_vol_wrap[i + 16]->SetLineColor(kBlue);
      tran_3[i].SetX(0.);
      tran_3[i].SetY(0.);
      tran_3[i].SetZ((HalfSphere_innerRadius + HalfSphere_outerRadius) / 2 + Sci_halfLength - Sci_eccentricity);
      rot_3[i].SetXEulerAngles(0, TMath::Pi() / 2, i * TMath::Pi() / 5);
      geoRot_3 = new TGeoRotation("geoRot_3", i * 36, 90, 0);
      tran_3[i] = rot_3[i] * tran_3[i];
      Cry_vol_wrap[i + 16]->AddNode(Cry_vol[i + 16], i + 17, new TGeoCombiTrans(0., 0., 0., fRefRot));
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol_wrap[i + 16], i + 17,
                   new TGeoCombiTrans(tran_3[i].X(), tran_3[i].Y(), tran_3[i].Z(), geoRot_3));
   }

   return halfSphere;
}
