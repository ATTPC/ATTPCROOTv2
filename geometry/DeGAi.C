/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
// Note: In root all sizes are given in cm

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
#include "TMath.h"
#include "TString.h"
#include "TSystem.h"

#include <iostream>

// Name of geometry version and output file
const TString geoVersion = "DeGAi";
const TString FileName = geoVersion + ".root";
const TString FileName1 = geoVersion + "_geomanager.root";

// Names of the different materials which are used to build the modules
// The materials are defined in the global media.geo file
const TString MediumGas = "GADGET_5IsoAr_800";
const TString CylinderVolumeMedium = "steel";
const TString MediumVacuum = "vacuum4";
const TString WindowMedium = "kapton";
const TString MicromegasMedium = "G10";
const TString DeGAiMatter = "germanium";
const TString ShellMatter = "aluminium";
// Detector Dimensions (cm)
const Float_t tpc_diameter_in = 15.4051;
const Float_t tpc_diameter_out = 16.51;
const Float_t drift_length = 40.;
const Float_t upstream_ring_dia_in = 11.684;
const Float_t upstream_ring_dia_out = tpc_diameter_out;
const Float_t upstream_ring_length = 2.6162 + 0.9398;
const Float_t downstream_ring_dia_in = 15.4051;
const Float_t downstream_ring_dia_out = 22.86;
const Float_t downstream_ring_length = 1.5748;
const Float_t upstream_cap_dia_in = 5.08;
const Float_t upstream_cap_dia_out = tpc_diameter_out;
const Float_t upstream_cap_length = 1.7272;
const Float_t downstream_cap_dia_in = 18.161;
const Float_t downstream_cap_dia_out = 22.86;
const Float_t downstream_cap_length = 2.6162;
const Float_t window_dia_in = 5.08;
const Float_t window_dia_out = 8.4074;
const Float_t window_length = 0.8636;
const Float_t micromegas_dia_in = 0;
const Float_t micromegas_dia_out = 22.86;
const Float_t micromegas_length = 0.2794;
const Float_t micromegas_ring_dia_in = 0;
const Float_t micromegas_ring_dia_out = 22.86;
const Float_t micromegas_ring_length = 0.635;
const Float_t cathode_dia_in = 5.5118;
const Float_t cathode_dia_out = 12.1844;
const Float_t cathode_length = 1.1016;
const Float_t cathode_mount_dia_in = 6.096;
const Float_t cathode_mount_dia_out = 14.8844;
const Float_t cathode_mount_length = 0.9525;
const Float_t cage_dia_in = 12.2074;
const Float_t cage_dia_out = (12.2074 + (12.2174 - 12.2074) * 0.8);
const Float_t cage_length = 38.0956;
const Float_t cage_support_shift = 2.71655;
const Float_t cage_support_dia_in = 12.2174;
const Float_t cage_support_dia_out = 13.8684;
const Float_t cage_support_length = 38.6867;


// DeGAi
const Float_t Box_Width = 10.1;
const Float_t Box_Length = 7.0;

const Float_t Crystal_Diameter = 2.5;

// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void DeGAi()
{
   // Load the necessary FairRoot libraries
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

   FairGeoMedium *GADGET_5IsoAr_800 = geoMedia->getMedium("GADGET_5IsoAr_800");
   FairGeoMedium *steel = geoMedia->getMedium("steel");
   FairGeoMedium *vacuum4 = geoMedia->getMedium("vacuum4");
   FairGeoMedium *kapton = geoMedia->getMedium("kapton");
   FairGeoMedium *G10 = geoMedia->getMedium("G10");

   FairGeoMedium *germanium = geoMedia->getMedium("germanium");
   FairGeoMedium *aluminium = geoMedia->getMedium("aluminium");

   // include check if all media are found

   geoBuild->createMedium(GADGET_5IsoAr_800);
   geoBuild->createMedium(steel);
   geoBuild->createMedium(vacuum4);
   geoBuild->createMedium(kapton);
   geoBuild->createMedium(G10);
   geoBuild->createMedium(germanium);
   geoBuild->createMedium(aluminium);


}

TGeoVolume *create_detector()
{

   // needed materials
   TGeoMedium *OuterCylinder = gGeoMan->GetMedium(CylinderVolumeMedium);
   TGeoMedium *gas = gGeoMan->GetMedium(MediumGas);
   TGeoMedium *windowmat = gGeoMan->GetMedium(WindowMedium);
   TGeoMedium *micromegasmat = gGeoMan->GetMedium(MicromegasMedium);

   TGeoMedium *shellmatter = gGeoMan->GetMedium(ShellMatter);
   TGeoMedium *degaimatter = gGeoMan->GetMedium(DeGAiMatter);
   TGeoMedium *mediumvacuum4 = gGeoMan->GetMedium(MediumVacuum);

   // Shell 
/*
   TGeoVolume *shell = gGeoManager->MakeSphere("shell", shellmatter, Hull_Inner_Radii/2, Hull_Outer_Radii/2);
   shell->SetLineColor(kCyan);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(shell, 1,
                new TGeoCombiTrans(0.0, 0,39/2, new TGeoRotation("shell", 0, 0, 0)));
   shell->SetTransparency(80);
*/


   TGeoVolume *dummy = gGeoManager->MakeTube("dummy", mediumvacuum4, 0, 0, 0);
   gGeoMan->GetVolume(geoVersion)->AddNode(dummy, 1, new TGeoCombiTrans(0.0, 0, 0, new TGeoRotation("dummy", 0, 0, 0)));
   dummy->SetTransparency(100);
/*
   //Shell with holes
   TGeoTube *entrance = new TGeoTube("entrance", 0, Hull_Outer_Radii/2, Hull_Outer_Radii +3 );

   TGeoSphere *body = new TGeoSphere("body", Hull_Inner_Radii, Hull_Outer_Radii);
   TGeoCompositeShape *shell = new TGeoCompositeShape("shell", "(body - entrance)");

   TGeoVolume *shelly = new TGeoVolume("shelly", shell,shellmatter);
   shelly->SetLineColor(kCyan);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(shelly, 1,
                new TGeoCombiTrans(0.0, 0,39/2, new TGeoRotation("shelly", 0, 0, 0)));
   shelly->SetTransparency(70);

 */
   //detector
   TGeoVolume **Cry_vol;
   Cry_vol = new TGeoVolume *[40];

   TString CrystalName = "Crystal_";
   TString name_cry[40] = {"01", "02", "03", "04", "05", "06", "07", "08", "09","10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40"};

   Double_t RvalsDet[40] = {19.05112315,19.05112315,19.05112315,19.05112315,19.05022437,19.05022437,19.05022437,19.05022437,19.05060231,19.05060231,19.05060231,19.05060231,10.66749804,10.66749804,10.66749804,10.66749804,10.66898269,10.66898269,10.66898269,10.66898269,10.66915413,10.66915413,10.66915413,10.66915413,19.04863682,19.04863682,19.04863682,19.04863682,19.0493892,19.0493892,19.0493892,19.0493892,14.47761989,14.47761989,14.47761989,14.47761989,14.47761989,14.47761989,14.47761989,14.47761989};

   Double_t Rvals[40] ={19.77683114,19.77683114,19.77647621,19.77647621,19.77759725,19.77759725,19.77713387,19.77713387,19.77644147,19.77591297,19.77633936,19.77707173,11.56007718,11.56116122,11.56116122,11.56007718,11.56118215,11.56120531,11.56120531,11.56118215,11.56103566,11.56072343,11.56072343,11.56103566,19.77724805,19.77604363,19.77762107,19.77681679,19.77604363,19.77724805,19.77681679,19.77762107,19.77604363,19.77724805,19.77681679,19.77762107,19.77604363,19.77724805,19.77681679,19.77762107};

   Double_t Rnew[40] = {19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,10.67,10.67,10.67,10.67,10.67,10.67,10.67,10.67,10.67,10.67,10.67,10.67,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05,19.05};

   Double_t ThetaDet[40] = {128.8960565,128.8960565,116.6719632,116.6719632 ,128.8942659,128.8942659,116.6710061,116.6710061,128.4539467,128.4551626,116.2283226,116.2272773,100.5205834,100.5195857,79.4804143,79.47941659,100.5195664,100.5195451,79.48045487,79.48043356,100.5197013,100.5199886,79.4800114,79.48029875,66.30557111,66.30403974,54.07972043,54.07803245,66.30403974,66.30557111,54.07803245,54.07972043,53.82653458,58.20928332,43.77046075,38.21971664,58.20928332,53.82653458,38.21971664,43.77046075};

   Double_t PhiDet[40] = {322.8804903,307.1195097,308.1415006,321.8584994,277.8814104,262.1185896,263.1405474,276.8594526,231.8670515,216.2007913,217.2029612,230.8687376,344.988577,323.5782166,323.5782166,344.988577,293.5633984,272.1514237,272.1514237,293.5633984,242.1293534,220.7318372,220.7318372,242.1293534,296.6944274,283.3072445,282.4272964,297.5769804,256.6927555,243.3055726,242.4230196,257.5727036,346.4647812,328.0076872,319.4166341,342.2518078,211.9923128,193.5352188,197.7481922,220.5833659};

   Double_t Phi[40] = {315,315,315,315,270,270,270,270,224.0322363,224.0322363,224.0322363,224.0322363,334.2891357,334.2891357,334.2891357,334.2891357,282.8617039,282.8617039,282.8617039,282.8617039,231.4275328,231.4275328,231.4275328,231.4275328,289.9974273,289.9974273,289.9974273,289.9974273,249.994283,249.994283,249.994283,249.994283,333.9975437,333.9975437,333.9975437,333.9975437,206.0024563,206.0024563,206.0024563,206.0024563};

   Double_t Theta[40] = {122.999694,122.999694,122.999694,122.999694,123.0014495,123.0014495,123.0014495,123.0014495,122.5555004,122.5555004,122.5555004,122.5555004,90,90,90,90,90,90,90,90,90,90,90,90,59.99763268,59.99763268,59.99763268,59.99763268,59.99893933,59.99893933,59.99893933,59.99893933,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684};

   Double_t Phinew[40] = {322.2045774,307.7954226,307.7952947,322.2047053,277.2043012,262.7956988,262.7955317,277.2044683,231.2369542,216.8273279,216.8274816,231.236727,346.4920848,322.0872965,322.0872965,346.4920848,295.0635217,270.6599098,270.6599098,295.0635217,243.6295006,219.2252454,219.2252454,243.6295006,297.2018544,282.792566,282.7931346,297.2020099,257.1991443,242.7898559,242.7897004,257.1985757,343.2938624,324.7020908,324.7012027,343.2938468,215.2979092,196.7061376,196.7061532,215.2987973};

   Double_t Thetanew[40] = {130.2042714,130.2042714,115.7949887,115.7949887,130.2057507,130.2057507,115.7969812,115.7969812,129.7602183,129.7604088,115.3507457,115.3510097,102.2029491,102.2018392,77.79816079,77.79705086,102.2018178,102.2017941,77.79820593,77.79818222,102.2019678,102.2022874,77.79771257,77.79803224,67.2020598,67.20249396,52.79334002,52.79305011,67.20380061,67.20336645,52.79435676,52.79464667,57.29556551,57.29469972,38.70290581,38.7029437,57.29469972,57.29556551,38.7029437,38.70290581};

   Int_t Color[40] = {600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632};
/*Cry_vol[nora] = gGeoManager->MakeTube(CrystalName + name_cry[nora], segamatter, sega_inner_dead_layer_rad_out,
                                            sega_active_rad_out, sega_active_length / 2);
      Cry_vol[nora]->SetLineColor(kBlue);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[nora], nora + 1,
                   new TGeoCombiTrans(x_posSega, y_posSega, z_posSega,
                                      new TGeoRotation("activeL", phirot[nora], updown * thetarot[nora], 0)));
      Cry_vol[nora]->SetTransparency(80);*/

   /*         322.8804903,307.1195097,308.1415006,321.8584994      -Phi[i],Theta[i]            */
   TString PositionType = "given";
   if(PositionType=="center"){

   for(Int_t i =0;i<10;i++){
   Cry_vol[i] = gGeoManager->MakeTube(CrystalName + name_cry[i], degaimatter, 0,
                                            Crystal_Diameter/2, Box_Length / 2);
      Cry_vol[i]->SetLineColor(Color[4*i]);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[i], i + 1,
                   new TGeoCombiTrans(-Rnew[4*i]*TMath::Sin(TMath::DegToRad()*Theta[4*i])*TMath::Sin(TMath::DegToRad()*Phi[4*i]),
                   Rnew[4*i]*TMath::Cos(TMath::DegToRad()*Phi[4*i]), 
                   Rnew[4*i]*TMath::Cos(TMath::DegToRad()*Theta[4*i])*TMath::Sin(TMath::DegToRad()*Phi[4*i]) + cage_support_length/2,
                                      new TGeoRotation(CrystalName + name_cry[i],Phi[4*i],Theta[4*i] , 0)));
      Cry_vol[i]->SetTransparency(0);
   };
   }
   // DeGAi given values

   else if(PositionType=="given"){

   for(Int_t i =0;i<40;i++){
   Cry_vol[i] = gGeoManager->MakeTube(CrystalName + name_cry[i], degaimatter, 0,
                                            Crystal_Diameter/2, Box_Length / 2);
      Cry_vol[i]->SetLineColor(Color[i]);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[i], i + 1,
                   new TGeoCombiTrans(-Rnew[i]*TMath::Sin(TMath::DegToRad()*ThetaDet[i])*TMath::Sin(TMath::DegToRad()*PhiDet[i]),
                   Rnew[i]*TMath::Cos(TMath::DegToRad()*PhiDet[i]), 
                   Rnew[i]*TMath::Cos(TMath::DegToRad()*ThetaDet[i])*TMath::Sin(TMath::DegToRad()*PhiDet[i]) + cage_support_length/2,
                                      new TGeoRotation(CrystalName + name_cry[i],Phi[i],Theta[i] , 0)));
      Cry_vol[i]->SetTransparency(0);
   };
   }
  
   else if(PositionType=="mine"){

   for(Int_t i =0;i<40;i++){
   Cry_vol[i] = gGeoManager->MakeTube(CrystalName + name_cry[i], degaimatter, 0,
                                            Crystal_Diameter/2, Box_Length / 2);
      Cry_vol[i]->SetLineColor(Color[i]);
      gGeoMan->GetVolume(geoVersion)
         ->AddNode(Cry_vol[i], i + 1,
                   new TGeoCombiTrans(-Rnew[i]*TMath::Sin(TMath::DegToRad()*Thetanew[i])*TMath::Sin(TMath::DegToRad()*Phinew[i]),
                   Rnew[i]*TMath::Cos(TMath::DegToRad()*Phinew[i]), 
                   Rnew[i]*TMath::Cos(TMath::DegToRad()*Thetanew[i])*TMath::Sin(TMath::DegToRad()*Phinew[i]) + cage_support_length/2,
                                      new TGeoRotation(CrystalName + name_cry[i],Phi[i],Theta[i] , 0)));
      Cry_vol[i]->SetTransparency(0);
   };
   };




   // GADGET Main drift volume
   double tpc_rot = 0;
   TGeoVolume *drift_volume = gGeoManager->MakeTube("drift_volume", gas, 0, tpc_diameter_in / 2, drift_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(drift_volume, 1,
                new TGeoCombiTrans(0.0, 0, drift_length / 2.0, new TGeoRotation("drift_volume", 0, tpc_rot, 0)));
   drift_volume->SetTransparency(80);
   // GADGET Outer steel chamber
   TGeoVolume *steel_chamber = gGeoManager->MakeTube("steel_chamber", OuterCylinder, tpc_diameter_in / 2,
                                                     tpc_diameter_out / 2, drift_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(steel_chamber, 1,
                new TGeoCombiTrans(0.0, 0, drift_length / 2.0, new TGeoRotation("steel_chamber", 0, tpc_rot, 0)));
   steel_chamber->SetTransparency(80);
   // GADGET Upstream  Ring
   TGeoVolume *upstream_ring = gGeoManager->MakeTube("upstream_ring", OuterCylinder, upstream_ring_dia_in / 2,
                                                     upstream_ring_dia_out / 2, upstream_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(
         upstream_ring, 1,
         new TGeoCombiTrans(0.0, 0, -upstream_ring_length / 2.0, new TGeoRotation("upstream_ring", 0, tpc_rot, 0)));
   upstream_ring->SetTransparency(80);
   // GADGET Upstream Cap
   TGeoVolume *upstream_cap = gGeoManager->MakeTube("upstream_cap", OuterCylinder, upstream_cap_dia_in / 2,
                                                    upstream_cap_dia_out / 2, upstream_cap_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(upstream_cap, 1,
                new TGeoCombiTrans(0.0, 0, -upstream_cap_length / 2.0 - upstream_ring_length,
                                   new TGeoRotation("upstream_cap", 0, tpc_rot, 0)));
   upstream_cap->SetTransparency(80);
   // GADGET Downstream  Ring
   TGeoVolume *downstream_ring = gGeoManager->MakeTube("downstream_ring", OuterCylinder, downstream_ring_dia_in / 2,
                                                       downstream_ring_dia_out / 2, downstream_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(downstream_ring, 1,
                new TGeoCombiTrans(0.0, 0, drift_length + downstream_ring_length / 2.0,
                                   new TGeoRotation("downstream_ring", 0, tpc_rot, 0)));
   downstream_ring->SetTransparency(80);
   // GADGET Downstream Cap
   TGeoVolume *downstream_cap = gGeoManager->MakeTube("downstream_cap", OuterCylinder, downstream_cap_dia_in / 2,
                                                      downstream_cap_dia_out / 2, downstream_cap_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(downstream_cap, 1,
                new TGeoCombiTrans(0.0, 0, downstream_cap_length / 2.0 + downstream_ring_length + drift_length,
                                   new TGeoRotation("downstream_cap", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(80);
   // GADGET Entrance Window
   TGeoVolume *tpc_window =
      gGeoManager->MakeTube("tpc_window", OuterCylinder, window_dia_in / 2, window_dia_out / 2, window_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(tpc_window, 1,
                new TGeoCombiTrans(0.0, 0, -upstream_ring_length - upstream_cap_length - window_length / 2,
                                   new TGeoRotation("tpc_window", 0, tpc_rot, 0)));
   tpc_window->SetTransparency(80);
   // GADGET Kapton Window
   TGeoVolume *kapton_window = gGeoManager->MakeTube("kapton_window", windowmat, 0, 5.08 / 2.0, 0.0127 / 2.0);
   kapton_window->SetLineColor(kBlue);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(kapton_window, 1,
                new TGeoCombiTrans(0.0, 0.0, -upstream_ring_length - upstream_cap_length,
                                   new TGeoRotation("kapton_window", 0, tpc_rot, 0)));
   kapton_window->SetTransparency(50);
   // GADGET Micromeags
   TGeoVolume *micromegas =
      gGeoManager->MakeTube("micromegas", micromegasmat, 0, micromegas_dia_out / 2, micromegas_length / 2);
   micromegas->SetLineColor(kRed);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(micromegas, 1,
                new TGeoCombiTrans(
                   0.0, 0.0, drift_length + downstream_ring_length + downstream_cap_length + micromegas_length / 2,
                   new TGeoRotation("micromegas", 0, tpc_rot, 0)));
   micromegas->SetTransparency(50);
   // GADGET Micromegas Ring
   TGeoVolume *micromegas_ring = gGeoManager->MakeTube("micromegas_ring", OuterCylinder, micromegas_ring_dia_in / 2,
                                                       micromegas_ring_dia_out / 2, micromegas_ring_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(micromegas_ring, 1,
                new TGeoCombiTrans(0.0, 0,
                                   micromegas_ring_length / 2.0 + drift_length + micromegas_length +
                                      downstream_ring_length + downstream_cap_length,
                                   new TGeoRotation("micromegas_ring", 0, tpc_rot, 0)));
   micromegas_ring->SetTransparency(80);
   // GADGET Cathode
   TGeoVolume *cathode =
      gGeoManager->MakeTube("cathode", OuterCylinder, cathode_dia_in / 2, cathode_dia_out / 2, cathode_length / 2);
   cathode->SetLineColor(kGreen);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(cathode, 1, new TGeoCombiTrans(0.0, 0, cathode_length / 2, new TGeoRotation("cathode", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(80);
   // GADGET Cathode Mount
   TGeoVolume *cathode_mount = gGeoManager->MakeTube("cathode_mount", OuterCylinder, cathode_mount_dia_in / 2,
                                                     cathode_mount_dia_out / 2, cathode_mount_length / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(cathode_mount, 1,
                new TGeoCombiTrans(0.0, 0, cathode_mount_length / 2, new TGeoRotation("cathode_mount", 0, tpc_rot, 0)));
   downstream_cap->SetTransparency(80);

   return dummy;
}
