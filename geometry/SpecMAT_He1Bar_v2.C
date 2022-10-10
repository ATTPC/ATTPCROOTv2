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
#include "TMath.h"
#include "TROOT.h"
#include "TString.h"
#include "TSystem.h"
#include "TVector3.h"

#include <iostream>

// Name of geometry version and output file
const TString geoVersion = "SpecMAT_He1Bar_v2";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different used materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "He_1bar";
const TString MediumVacuum = "vacuum4";
// const TString MediumWindow = "aramid";
// const TString CylinderVolumeMedium = "steel";
const TString FlangeMaterial = "Aluminum5083";
const TString FieldCageMaterial = "Epoxy";
const TString ScintillatorWindowMaterial = "Quartz";
const TString ScintillatorCrystalMaterial = "CeBr3";
const TString ScintillatorReflectorMaterial = "TiO2";

// Parameters for SpecMAT detector geometry
const int TotalCrystNb = 45; // Number of scintillators in the scintillator array
TGeoVolumeAssembly *sci_seg[15];
const TString geoSegments = "sci_seg_";

// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void SpecMAT_He1Bar_v2()
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
   // FairGeoMedium *steel = geoMedia->getMedium("steel");
   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   // FairGeoMedium *aramid = geoMedia->getMedium("aramid");
   FairGeoMedium *Aluminum5083 = geoMedia->getMedium(FlangeMaterial);
   FairGeoMedium *Epoxy = geoMedia->getMedium(FieldCageMaterial);
   FairGeoMedium *TiO2 = geoMedia->getMedium(ScintillatorReflectorMaterial);
   FairGeoMedium *CeBr3 = geoMedia->getMedium(ScintillatorCrystalMaterial);
   FairGeoMedium *Quartz = geoMedia->getMedium(ScintillatorWindowMaterial);

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
   // geoBuild->createMedium(steel);
   geoBuild->createMedium(vacuum4);
   // geoBuild->createMedium(aramid);
   geoBuild->createMedium(Aluminum5083);
   geoBuild->createMedium(Epoxy);
   geoBuild->createMedium(TiO2);
   geoBuild->createMedium(CeBr3);
   geoBuild->createMedium(Quartz);
}

TGeoVolume *create_detector()
{

   // needed materials
   // TGeoMedium *OuterCylinder = gGeoMan->GetMedium(CylinderVolumeMedium);
   TGeoMedium *gas = gGeoMan->GetMedium(MediumGas);
   // TGeoMedium *windowmat = gGeoMan->GetMedium(MediumWindow);
   TGeoMedium *Vacuummat = gGeoMan->GetMedium("vacuum4");
   TGeoMedium *Aluminum5083mat = gGeoMan->GetMedium(FlangeMaterial);
   TGeoMedium *Epoxymat = gGeoMan->GetMedium(FieldCageMaterial);
   TGeoMedium *TiO2mat = gGeoMan->GetMedium(ScintillatorReflectorMaterial);
   TGeoMedium *CeBr3mat = gGeoMan->GetMedium(ScintillatorCrystalMaterial);
   TGeoMedium *Quartzmat = gGeoMan->GetMedium(ScintillatorWindowMaterial);

   /*** Unused IC media ***
   TGeoMedium* ICgas         = gGeoMan->GetMedium(ICMediumGas);
   TGeoMedium* ICwindowmat   = gGeoMan->GetMedium(ICWindowMedium);
   TGeoMedium* ICAlwindowmat = gGeoMan->GetMedium(ICAlWindowMedium);
   */

   // SpecMAT Main drift volume

   // double tpc_rot = TMath::Pi();
   double tpc_rot = 0;
   double SPMT_active_volume_radius = 11.0;
   double SPMT_active_volume_halflength = 16.175;

   TGeoVolume *drift_volume =
      gGeoManager->MakeTube("drift_volume", gas, 0, SPMT_active_volume_radius, SPMT_active_volume_halflength);
   // TGeoVolume *drift_volume = gGeoManager->MakeBox("drift_volume", gas,  100./2, 100./2, 100./2);
   // gGeoMan->GetVolume(geoVersion)->AddNode(drift_volume,1, new TGeoTranslation(0,0,drift_length/2));
   drift_volume->SetLineColor(kCyan);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(drift_volume, 1, new TGeoCombiTrans(0, 0, 16.175, new TGeoRotation("drift_volume", 0, tpc_rot, 0)));
   drift_volume->SetTransparency(80);

   // SpecMAT padplane

   double SPMT_padplane_innerradius = 0.4;
   double SPMT_padplane_outerradius = 13.5;
   double SPMT_padplane_halflength = 0.5;

   TGeoVolume *padplane = gGeoManager->MakeTube("pad_plane", Aluminum5083mat, SPMT_padplane_innerradius,
                                                SPMT_padplane_outerradius, SPMT_padplane_halflength);
   padplane->SetLineColor(kGray + 1);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(padplane, 1, new TGeoCombiTrans(0.0, 0.0, 32.850, new TGeoRotation("padplane", 0, tpc_rot, 0)));
   padplane->SetTransparency(40);

   // SpecMAT field cage

   double SPMT_fieldcage_innerradius = SPMT_active_volume_radius;
   double SPMT_fieldcage_outerradius = 12.65;
   double SPMT_fieldcage_halflength = 16.175;

   TGeoVolume *field_cage = gGeoManager->MakeTube("field_cage", Epoxymat, SPMT_active_volume_radius,
                                                  SPMT_fieldcage_outerradius, SPMT_fieldcage_halflength);
   field_cage->SetLineColor(kOrange + 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(field_cage, 1, new TGeoCombiTrans(0.0, 0.0, 16.175, new TGeoRotation("field_cage", 0, tpc_rot, 0)));
   field_cage->SetTransparency(35);

   // SpecMAT aluminum vacuum chamber parts (consists of seven tubes)

   // Firsttube
   double vacuumchamber_innerradius = 12.825;
   double vacuumchamber_outerradius = 13.125;
   double vacuumchamber_halflength = 10.225;
   TGeoVolume *vacuum_chamber = gGeoManager->MakeTube("vacuum_chamber", Aluminum5083mat, vacuumchamber_innerradius,
                                                      vacuumchamber_outerradius, vacuumchamber_halflength);
   vacuum_chamber->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vacuum_chamber, 1,
                new TGeoCombiTrans(0.0, 0.0, 19.425, new TGeoRotation("vacuum_chamber", 0, tpc_rot, 0)));
   vacuum_chamber->SetTransparency(35);

   // Secondtube
   double vacuumchamber2_innerradius = vacuumchamber_innerradius;
   double vacuumchamber2_outerradius = 22.6;
   double vacuumchamber2_halflength = 0.5;
   TGeoVolume *vacuum_chamber2 = gGeoManager->MakeTube("vacuum_chamber2", Aluminum5083mat, vacuumchamber2_innerradius,
                                                       vacuumchamber2_outerradius, vacuumchamber2_halflength);
   vacuum_chamber2->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vacuum_chamber2, 1,
                new TGeoCombiTrans(0.0, 0.0, 30.15, new TGeoRotation("vacuum_chamber2", 0, tpc_rot, 0)));
   vacuum_chamber2->SetTransparency(35);

   // Thirdtube
   double vacuumchamber3_innerradius = 15.0;
   double vacuumchamber3_outerradius = 25.5;
   double vacuumchamber3_halflength = 0.5;
   TGeoVolume *vacuum_chamber3 = gGeoManager->MakeTube("vacuum_chamber3", Aluminum5083mat, vacuumchamber3_innerradius,
                                                       vacuumchamber3_outerradius, vacuumchamber3_halflength);
   vacuum_chamber3->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vacuum_chamber3, 1,
                new TGeoCombiTrans(0.0, 0.0, 31.15, new TGeoRotation("vacuum_chamber3", 0, tpc_rot, 0)));
   vacuum_chamber3->SetTransparency(35);

   // Fourthtube
   double vacuumchamber4_innerradius = 20.0;
   double vacuumchamber4_outerradius = 25.5;
   double vacuumchamber4_halflength = 1.5;
   TGeoVolume *vacuum_chamber4 = gGeoManager->MakeTube("vacuum_chamber4", Aluminum5083mat, vacuumchamber4_innerradius,
                                                       vacuumchamber4_outerradius, vacuumchamber4_halflength);
   vacuum_chamber4->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vacuum_chamber4, 1,
                new TGeoCombiTrans(0.0, 0.0, 33.15, new TGeoRotation("vacuum_chamber4", 0, tpc_rot, 0)));
   vacuum_chamber4->SetTransparency(35);

   // Fifthtube
   double vacuumchamber5_innerradius = vacuumchamber_innerradius;
   double vacuumchamber5_outerradius = 25.4;
   double vacuumchamber5_halflength = 0.75;
   TGeoVolume *vacuum_chamber5 = gGeoManager->MakeTube("vacuum_chamber5", Aluminum5083mat, vacuumchamber5_innerradius,
                                                       vacuumchamber5_outerradius, vacuumchamber5_halflength);
   vacuum_chamber5->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vacuum_chamber5, 1,
                new TGeoCombiTrans(0.0, 0.0, 8.45, new TGeoRotation("vacuum_chamber5", 0, tpc_rot, 0)));
   vacuum_chamber5->SetTransparency(35);

   // Sixthtube
   double vacuumchamber6_innerradius = 23.9;
   double vacuumchamber6_outerradius = 25.4;
   double vacuumchamber6_halflength = 3.75;
   TGeoVolume *vacuum_chamber6 = gGeoManager->MakeTube("vacuum_chamber6", Aluminum5083mat, vacuumchamber6_innerradius,
                                                       vacuumchamber6_outerradius, vacuumchamber6_halflength);
   vacuum_chamber6->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vacuum_chamber6, 1,
                new TGeoCombiTrans(0.0, 0.0, 3.95, new TGeoRotation("vacuum_chamber6", 0, tpc_rot, 0)));
   vacuum_chamber6->SetTransparency(35);

   // Seventhtube
   double vacuumchamber7_innerradius = 23.9;
   double vacuumchamber7_outerradius = 30.5;
   double vacuumchamber7_halflength = 1.0;
   TGeoVolume *vacuum_chamber7 = gGeoManager->MakeTube("vacuum_chamber7", Aluminum5083mat, vacuumchamber7_innerradius,
                                                       vacuumchamber7_outerradius, vacuumchamber7_halflength);
   vacuum_chamber7->SetLineColor(kGray);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(vacuum_chamber7, 1,
                new TGeoCombiTrans(0.0, 0.0, -0.8, new TGeoRotation("vacuum_chamber7", 0, tpc_rot, 0)));
   vacuum_chamber7->SetTransparency(35);

   // SpecMAT scintillator array

   // Dimensions of the crystal
   double sciCrystSizeX = 2.4; // Size and position of all components depends on Crystal size and position.
   double sciCrystSizeY = 2.4;
   double sciCrystSizeZ = 2.4;
   // Position of the crystal
   double sciCrystPosX = 0; // Position of the Crystal along the X axis
   double sciCrystPosY = 0; // Position of the Crystal along the Y axis
   double sciCrystPosZ = 0; // Position of the Crystal along the Z axis

   TGeoVolume *sci_cryst_vol =
      gGeoManager->MakeBox("sci_cryst_vol", CeBr3mat, sciCrystSizeX, sciCrystSizeY, sciCrystSizeZ);
   sci_cryst_vol->SetLineColor(kBlue + 3);
   sci_cryst_vol->SetTransparency(35);

   // Thickness of reflector walls
   double sciReflWallThickX = 0.05;
   double sciReflWallThickY = 0.05;
   double sciReflWindThick = 0.05;

   // Outer dimensions of the reflector relative to the crystal size
   double sciReflSizeX = sciCrystSizeX + sciReflWallThickX;
   double sciReflSizeY = sciCrystSizeY + sciReflWallThickY;
   double sciReflSizeZ = sciCrystSizeZ + sciReflWindThick / 2;

   // Position of the reflector relative to the crystal position
   double sciReflPosX = sciCrystPosX;
   double sciReflPosY = sciCrystPosY;
   double sciReflPosZ =
      sciCrystPosZ - sciReflWindThick / 2; // Position of the Reflector relative to the Al Housing along the Z axis

   TGeoVolume *sci_crystreflbox =
      gGeoManager->MakeBox("sci_crystreflbox", TiO2mat, sciReflSizeX, sciReflSizeY, sciReflSizeZ);
   TGeoTranslation *m1 = new TGeoTranslation(sciCrystPosX, sciCrystPosY, sciReflWindThick / 2);
   m1->SetName("m1");
   m1->RegisterYourself();
   TGeoCompositeShape *cs = new TGeoCompositeShape("cs", "sci_crystreflbox-(sci_cryst_vol:m1)");
   TGeoVolume *sci_crystrefl = new TGeoVolume("sci_crystrefl", cs, TiO2mat);
   sci_crystrefl->SetLineColor(kRed + 3);
   sci_crystrefl->SetTransparency(35);

   // Dimensions of Housing (half-side)
   double sciHousWallThickX = 0.3;
   double sciHousWallThickY = 0.3;
   double sciHousWindThick = 0.1;

   // Outer dimensions of the housing relative to the crystal size and to the thickness of the reflector
   double sciHousSizeX = sciCrystSizeX + sciReflWallThickX + sciHousWallThickX;
   double sciHousSizeY = sciCrystSizeY + sciReflWallThickY + sciHousWallThickY;
   double sciHousSizeZ = sciCrystSizeZ + sciReflWindThick / 2 + sciHousWindThick / 2;

   // Position of the housing relative to the crystal position
   double sciHousPosX = sciCrystPosX;
   double sciHousPosY = sciCrystPosY;
   double sciHousPosZ = sciCrystPosZ - (sciReflWindThick / 2 + sciHousWindThick / 2);

   TGeoVolume *sci_hous_box =
      gGeoManager->MakeBox("sci_hous_box", Aluminum5083mat, sciHousSizeX, sciHousSizeY, sciHousSizeZ);
   TGeoTranslation *m2 = new TGeoTranslation(sciReflPosX, sciReflPosY, sciHousWindThick / 2);
   m2->SetName("m2");
   m2->RegisterYourself();
   TGeoCompositeShape *cs2 = new TGeoCompositeShape("cs2", "sci_hous_box-(sci_crystreflbox:m2)");
   TGeoVolume *sci_hous = new TGeoVolume("sci_hous", cs2, Aluminum5083mat);
   sci_hous->SetLineColor(kGreen + 3);
   sci_hous->SetTransparency(35);

   // Dimensions of the Window (half-side)
   double sciWindSizeX = sciCrystSizeX + sciReflWallThickX + sciHousWallThickX; // X half-size of the Window
   double sciWindSizeY = sciCrystSizeY + sciReflWallThickY + sciHousWallThickY; // Y half-size of the Window
   double sciWindSizeZ = 0.1;                                                   // Z half-size of the Window

   // Position of the window relative to the crystal
   double sciWindPosX = sciCrystPosX; // Position of the Window along the X axis
   double sciWindPosY = sciCrystPosY; // Position of the Window along the Y axis
   double sciWindPosZ =
      sciCrystPosZ + sciCrystSizeZ + sciWindSizeZ; // Position of the Window relative to the Al Housing along the Z axis

   TGeoVolume *sci_window =
      gGeoManager->MakeBox("sci_window", Aluminum5083mat, sciWindSizeX, sciWindSizeY, sciWindSizeZ);

   // Positioning of segments and crystals in the segment

   // In TotalCrystNb array will be stored coordinates of the all crystals, which could be used for further Doppler
   // correction

   TVector3 crystalPositionsArray[TotalCrystNb]; // Dinamic mamory allocation for the array
   for (int i = 0; i < TotalCrystNb; i++) {
      crystalPositionsArray[i].SetXYZ(0., 0., 0.); // Initialize all elements of the array to zero.
   }

   int i = 0;      // counter for reconstruction of crystal positions
   int crysNb = 1; // crystal counter
   double phi;
   double twopi = TMath::Pi() * 2;
   for (int iseg = 0; iseg < 15; iseg++) {
      phi = (iseg * twopi / 15 + twopi / 4) * 360 / twopi; // +twopi/4  Rotates crystalls around the Z axes by 90deg to
                                                           // reproduce the same position as in the real setup
      TRotation rotm, rotm2, rotm3;                        //** rotation matrix definition

      rotm.RotateY(90);  //** rotation matrix for positioning segments
      rotm.RotateZ(phi); //** rotation matrix for positioning segments
      TGeoRotation rotmGeo;
      rotmGeo.RotateY(90);  //** rotation matrix for positioning segments
      rotmGeo.RotateZ(phi); //** rotation matrix for positioning segments

      rotm2.RotateX(360 - phi); //### rotation matrix for reconstruction of crystal positions
      rotm3.RotateY(90);        //### rotation matrix for reconstruction of crystal positions

      TVector3 uz(std::cos(phi * twopi / 360), std::sin(phi * twopi / 360),
                  0.); // coeficient which will be used for preliminary rotation of the segments and crystals
      // segmentBoxLog = new G4LogicalVolume(segmentBox, segment_mat, "segmentBoxLog");
      // TGeoVolume *sci_seg = gGeoManager->MakeBox("sci_seg", Vacuummat, sciHousSizeX*3+3*(3-1)/2, sciHousSizeY*1,
      // sciHousSizeZ+sciWindSizeZ);
      TString Segnr;
      Segnr.Form("%d", iseg);
      const TString Current_seg_name = geoSegments + Segnr;
      sci_seg[iseg] = new TGeoVolumeAssembly(Current_seg_name);
      TVector3 positionInSegment;
      positionInSegment = TVector3(-(3 * sciHousSizeX + 0.3 * (3 - 1) / 2 - sciHousSizeX),
                                   -(1 * sciHousSizeY - sciHousSizeY), (sciHousSizeZ - sciCrystSizeZ - sciWindSizeZ));

      for (int icrystRow = 0; icrystRow < 1; icrystRow++) {
         for (int icrystCol = 0; icrystCol < 3; icrystCol++) {
            TRotation rotm1;

            TVector3 positionCryst, positionWind, positionRefl, positionHous;
            positionCryst = (TVector3(0., 0., sciCrystPosZ) + positionInSegment);
            positionWind = (TVector3(0., 0., sciWindPosZ) + positionInSegment);
            positionRefl = (TVector3(0., 0., sciReflPosZ) + positionInSegment);
            positionHous = (TVector3(0., 0., sciHousPosZ) + positionInSegment);

            crystalPositionsArray[crysNb - 1] =
               positionCryst; // assigning initial crystal positions in a segment into array

            TVector3 transformCryst, transformWind, transformRefl, transformHous;
            transformCryst = positionCryst.Transform(rotm1);
            transformWind = positionWind.Transform(rotm1);
            transformRefl = positionRefl.Transform(rotm1);
            transformHous = positionHous.Transform(rotm1);

            // Placement of scintillator parts inside segment
            gGeoMan->GetVolume(Current_seg_name)
               ->AddNode(sci_cryst_vol, crysNb,
                         new TGeoCombiTrans(transformCryst.X(), transformCryst.Y(), transformCryst.Z(),
                                            new TGeoRotation("sci_crystal", 0, tpc_rot, 0)));

            gGeoMan->GetVolume(Current_seg_name)
               ->AddNode(sci_window, crysNb,
                         new TGeoCombiTrans(transformWind.X(), transformWind.Y(), transformWind.Z(),
                                            new TGeoRotation("sci_window", 0, tpc_rot, 0)));

            gGeoMan->GetVolume(Current_seg_name)
               ->AddNode(sci_crystrefl, crysNb,
                         new TGeoCombiTrans(transformRefl.X(), transformRefl.Y(), transformRefl.Z(),
                                            new TGeoRotation("sci_reflector", 0, tpc_rot, 0)));

            gGeoMan->GetVolume(Current_seg_name)
               ->AddNode(sci_hous, crysNb,
                         new TGeoCombiTrans(transformHous.X(), transformHous.Y(), transformHous.Z(),
                                            new TGeoRotation("sci_housing", 0, tpc_rot, 0)));

            crysNb += 1;
            positionInSegment += TVector3(sciHousSizeX * 2 + 0.3, 0., 0.);
         }
         positionInSegment -= TVector3(3 * sciHousSizeX * 2 + 0.3 * (3), 0., 0.);
         positionInSegment += TVector3(0., sciHousSizeY * 2, 0.);
      }

      // Placement of segment inside main volume
      TVector3 vz(0, 0, 16.175 + 3.25);
      TVector3 positionSegment = (13.125 + (sciHousSizeZ + sciWindSizeZ)) * uz + vz;
      TGeoTranslation transformSegment_translation(positionSegment.X(), positionSegment.Y(), positionSegment.Z());
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(sci_seg[iseg], iseg + 1, new TGeoCombiTrans(transformSegment_translation, rotmGeo));
      // cout << "Segment number is:" << iseg << endl;
      // cout << "segment position is: (" << positionSegment.X() << "," << positionSegment.Y() << ","
      // << positionSegment.Z() << ")" << endl;
   }

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
