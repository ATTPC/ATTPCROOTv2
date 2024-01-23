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

#include <Math/Polar3D.h>
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
const Float_t Crystal_Diameter = 5;
const Float_t Crystal_Length = 7.0;
const Float_t Hole_Diameter = 1.0;
const Float_t Hole_Length = 4.0;

// some global variables
TGeoManager *gGeoMan = new TGeoManager("ATTPC", "ATTPC");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// function to convert spherical coordinates to Cartesian coordinates
void convert2coords(Double_t r, Double_t phi, Double_t theta, Double_t outputArray[3]) {
    // Convert spherical coordinates to Cartesian coordinates with modified axes
    Double_t x = -r * TMath::Sin(theta) * TMath::Cos(phi);
    Double_t y = r * TMath::Cos(theta);
    Double_t z = r * TMath::Sin(theta) * TMath::Sin(phi);

    // Assign values to the output array
    outputArray[0] = x;
    outputArray[1] = y;
    outputArray[2] = z;
}

  

  
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


 // CLover

   TGeoTube *Crystal = new TGeoTube("Det1", 0, Crystal_Diameter/2, Crystal_Length/2);
   TGeoTube *Hole = new TGeoTube("Hole", 0, Hole_Diameter/2, Hole_Length/2);
   TGeoCombiTrans *Holetrans = new TGeoCombiTrans("Holetrans", 0, 0, 2.0, new TGeoRotation("Holetrans", 0, 0, 0));
   Holetrans->RegisterYourself();
   TGeoBBox *FlattenCrystalV = new TGeoBBox("FlattenCrystalV", 0.38, Crystal_Diameter/2, Crystal_Length/2+1);
   TGeoCombiTrans *FlattenCrystaltrans1 = new TGeoCombiTrans("FlattenCrystaltrans1", 2.5, 0, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   TGeoCombiTrans *FlattenCrystaltrans2 = new TGeoCombiTrans("FlattenCrystaltrans2", -2.5, 0, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   TGeoBBox *FlattenCrystalH = new TGeoBBox("FlattenCrystalH",Crystal_Diameter/2, 0.38, Crystal_Length/2+1);
   TGeoCombiTrans *FlattenCrystaltrans3 = new TGeoCombiTrans("FlattenCrystaltrans3", 0,2.5, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   TGeoCombiTrans *FlattenCrystaltrans4 = new TGeoCombiTrans("FlattenCrystaltrans4", 0,-2.5, 0, new TGeoRotation("FlattenCrystaltrans", 0, 0, 0));
   FlattenCrystaltrans1->RegisterYourself();
   FlattenCrystaltrans2->RegisterYourself();
   FlattenCrystaltrans3->RegisterYourself();
   FlattenCrystaltrans4->RegisterYourself();

 // Top Right clover
   TGeoCompositeShape *Clover1 = new TGeoCompositeShape("Clover1", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans4)");
// Top Left clover
   TGeoCompositeShape *Clover2 = new TGeoCompositeShape("Clover2", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans4)");
// Bottom Right clover
   TGeoCompositeShape *Clover3 = new TGeoCompositeShape("Clover3", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans3)");
// Bottom Left clover
   TGeoCompositeShape *Clover4 = new TGeoCompositeShape("Clover4", "(Det1 - Hole:Holetrans - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans3)");
  

// Cover
   TGeoCombiTrans *TRtrans = new TGeoCombiTrans("TRtrans",-2.5+0.38, 2.5-0.38, 0, new TGeoRotation("TRtrans", 0, 0, 0));
   TRtrans->RegisterYourself();
   TGeoCombiTrans *TLtrans = new TGeoCombiTrans("TLtrans",2.5-0.38, 2.5-0.38, 0, new TGeoRotation("TLtrans", 0, 0, 0));
   TLtrans->RegisterYourself();
   TGeoCombiTrans *BRtrans = new TGeoCombiTrans("BRtrans",-2.5+0.38, -2.5+0.38, 0, new TGeoRotation("BRtrans", 0, 0, 0));
   BRtrans->RegisterYourself();
   TGeoCombiTrans *BLtrans = new TGeoCombiTrans("BLtrans",2.5-0.38,-2.5+0.38, 0, new TGeoRotation("BLtrans", 0, 0, 0));
   BLtrans->RegisterYourself();
   
   TGeoCompositeShape *Clover1hole = new TGeoCompositeShape("Clover1hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans4)");
   TGeoCompositeShape *Clover2hole = new TGeoCompositeShape("Clover2hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans4)");
   TGeoCompositeShape *Clover3hole = new TGeoCompositeShape("Clover3hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans1  - FlattenCrystalH:FlattenCrystaltrans3)");
   TGeoCompositeShape *Clover4hole = new TGeoCompositeShape("Clover4hole", "(Det1 - FlattenCrystalV:FlattenCrystaltrans2  - FlattenCrystalH:FlattenCrystaltrans3)");

   TGeoBBox *Box = new TGeoBBox("Box", Box_Width/2, Box_Width/2, Box_Length/2-0.01);
   TGeoCompositeShape *CoverB = new TGeoCompositeShape("Cover", "Box - Clover1hole:TRtrans - Clover2hole:TLtrans - Clover3hole:BRtrans - Clover4hole:BLtrans ");
   TGeoCompositeShape *Detector = new TGeoCompositeShape("Detector", "Cover + Clover1:TRtrans + Clover2:TLtrans + Clover3:BRtrans + Clover4:BLtrans");
   //detector
   TGeoVolume **Cry_vol;
   Cry_vol = new TGeoVolume *[41];

   TString CrystalName = "Crystal_";
   TString name_cry[40] = {"01", "02", "03", "04", "05", "06", "07", "08", "09","10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40"};

   Double_t Poffset =7.54 ;
   Double_t Toffset =6.14 ;
   Double_t Rvals[40] ={19.77683114,19.77683114,19.77647621,19.77647621,19.77759725,19.77759725,19.77713387,19.77713387,19.77644147,19.77591297,19.77633936,19.77707173,11.56007718,11.56116122,11.56116122,11.56007718,11.56118215,11.56120531,11.56120531,11.56118215,11.56103566,11.56072343,11.56072343,11.56103566,19.77724805,19.77604363,19.77762107,19.77681679,19.77604363,19.77724805,19.77681679,19.77762107,19.77604363,19.77724805,19.77681679,19.77762107,19.77604363,19.77724805,19.77681679,19.77762107};

   Double_t ThetaDet[40] = {128.8960565,128.8960565,116.6719632,116.6719632 ,128.8942659,128.8942659,116.6710061,116.6710061,128.4539467,128.4551626,116.2283226,116.2272773,100.5205834,100.5195857,79.4804143,79.47941659,100.5195664,100.5195451,79.48045487,79.48043356,100.5197013,100.5199886,79.4800114,79.48029875,66.30557111,66.30403974,54.07972043,54.07803245,66.30403974,66.30557111,54.07803245,54.07972043,
   47.99924684+Toffset,47.99924684+Toffset,47.99924684-Toffset,47.99924684-Toffset,47.99924684+Toffset,47.99924684+Toffset,47.99924684-Toffset,47.99924684-Toffset}; 
   //53.82653458,58.20928332,43.77046075, 38.21971664,58.21928332, 53.82653458, 38.21971664, 41.77046075};

   Double_t PhiDet[40] = {322.8804903,307.1195097,308.1415006,321.8584994,277.8814104,262.1185896,263.1405474,276.8594526,231.8670515,216.2007913,217.2029612,230.8687376,344.988577,323.5782166,323.5782166,344.988577,293.5633984,272.1514237,272.1514237,293.5633984,242.1293534,220.7318372,220.7318372,242.1293534,296.6944274,283.3072445,282.4272964,297.5769804,256.6927555,243.3055726,242.4230196,257.5727036,
   333.9975437+Poffset,333.9975437-Poffset,333.9975437-Poffset-2.0,333.9975437+Poffset+2.0,206.0024563+Poffset,206.0024563-Poffset,206.0024563-Poffset-2.0,206.0024563+Poffset+2.0};
   //346.4647812,328.0076872, 319.4166341, 342.2518078,211.9923128, 193.5352188, 197.7481922, 220.5833659}; offset

   Double_t Phi[40] = {315,315,315,315,270,270,270,270,224.0322363,224.0322363,224.0322363,224.0322363,334.2891357,334.2891357,334.2891357,334.2891357,282.8617039,282.8617039,282.8617039,282.8617039,231.4275328,231.4275328,231.4275328,231.4275328,289.9974273,289.9974273,289.9974273,289.9974273,249.994283,249.994283,249.994283,249.994283,333.9975437,333.9975437,333.9975437,333.9975437,206.0024563,206.0024563,206.0024563,206.0024563};

   Double_t Theta[40] = {122.999694,122.999694,122.999694,122.999694,123.0014495,123.0014495,123.0014495,123.0014495,122.5555004,122.5555004,122.5555004,122.5555004,90,90,90,90,90,90,90,90,90,90,90,90,59.99763268,59.99763268,59.99763268,59.99763268,59.99893933,59.99893933,59.99893933,59.99893933,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684,47.99924684};

   Int_t Color[40] = {600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632,600,400,416,632};

	
  
  

for (Int_t i = 0; i < 10; i++) {
    Int_t index = 4 * i;
    
    Double_t avec0[6];
    convert2coords(Rvals[index]-.5, TMath::DegToRad() * Phi[index], TMath::DegToRad() * Theta[index], avec0);
    TGeoVolume *Cover = new TGeoVolume(CrystalName + name_cry[index], CoverB, shellmatter);
    Cover->SetLineColor(kGray);
    gGeoMan->GetVolume(geoVersion)->AddNode(Cover, index + 1, new TGeoCombiTrans(avec0[2], avec0[0], avec0[1] + cage_support_length / 2, 
    new TGeoRotation(CrystalName + name_cry[index], Phi[index], Theta[index], 0)));
    Cover->SetTransparency(0);

    for (Int_t j = 0; j < 4; j++) {
        Int_t cryIndex = index + j;
        Double_t avec[6];
        convert2coords(Rvals[cryIndex], TMath::DegToRad() * PhiDet[cryIndex], TMath::DegToRad() * ThetaDet[cryIndex], avec);
        TGeoVolume *Cry_vol = nullptr;
        if (j == 0) {
            Cry_vol = new TGeoVolume(CrystalName + name_cry[cryIndex], Clover4, degaimatter);
        } else if (j == 1) {
            Cry_vol = new TGeoVolume(CrystalName + name_cry[cryIndex], Clover3, degaimatter);
        } else if (j == 2) {
            Cry_vol = new TGeoVolume(CrystalName + name_cry[cryIndex], Clover1, degaimatter);
        } else if (j == 3) {
            Cry_vol = new TGeoVolume(CrystalName + name_cry[cryIndex], Clover2, degaimatter);
        }
        Cry_vol->SetLineColor(Color[cryIndex]);
        gGeoMan->GetVolume(geoVersion)->AddNode(Cry_vol, cryIndex + 1, new TGeoCombiTrans(avec[2], avec[0], avec[1] + cage_support_length / 2, 
        new TGeoRotation(CrystalName + name_cry[cryIndex], Phi[cryIndex], Theta[cryIndex], 0)));
        Cry_vol->SetTransparency(0);
    }
}


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

   return drift_volume;
}

