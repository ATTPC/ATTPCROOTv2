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
const TString geoVersion = "SOLARIS_SiArray_v1.0";
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

const Float_t Z_dist_from_target = -10.0;        // cm
const Float_t x_off = 2.3;                       // Distance between Si in the same YZ plane
const Float_t y_off = 2.3;                       // Distance between Si in the same XZ plane
const Float_t Recoil_Z_dist_from_target = 140.0; // Distance of the recoil detector with respect to the target
const Float_t Si_width = 1.0;                    // Width of Si detectors

// some global variables
TGeoManager *gGeoMan = new TGeoManager("HELIOS", "HELIOS");
;                     // Pointer to TGeoManager instance
TGeoVolume *gModules; // Global storage for module types

// Forward declarations
void create_materials_from_media_file();
TGeoVolume *create_detector();
void position_detector();
void add_alignable_volumes();

void SOLARIS_SiArray_v1_0()
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

   // YZ plane (left beam side)
   TGeoVolume *siliconYZL_1 = gGeoManager->MakeBox("siliconYZL_1", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZL_1, 2, new TGeoTranslation(y_off / 2, 0, 0.0 + Z_dist_from_target));
   siliconYZL_1->SetLineColor(kGray);

   TGeoVolume *siliconYZL_2 = gGeoManager->MakeBox("siliconYZL_2", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZL_2, 3, new TGeoTranslation(y_off / 2, 0, -6.087 + Z_dist_from_target));
   siliconYZL_2->SetLineColor(kGray);

   TGeoVolume *siliconYZL_3 = gGeoManager->MakeBox("siliconYZL_3", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZL_3, 4, new TGeoTranslation(y_off / 2, 0, -12.093 + Z_dist_from_target));
   siliconYZL_3->SetLineColor(kGray);

   TGeoVolume *siliconYZL_4 = gGeoManager->MakeBox("siliconYZL_4", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZL_4, 5, new TGeoTranslation(y_off / 2, 0, -17.601 + Z_dist_from_target));
   siliconYZL_4->SetLineColor(kGray);

   TGeoVolume *siliconYZL_5 = gGeoManager->MakeBox("siliconYZL_5", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZL_5, 6, new TGeoTranslation(y_off / 2, 0, -23.487 + Z_dist_from_target));
   siliconYZL_5->SetLineColor(kGray);

   TGeoVolume *siliconYZL_6 = gGeoManager->MakeBox("siliconYZL_6", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZL_5, 35, new TGeoTranslation(y_off / 2, 0, -29.487 + Z_dist_from_target));
   siliconYZL_5->SetLineColor(kGray);

   TGeoVolume *PCBYZL = gGeoManager->MakeBox("PCBYZL", pcb, 0.1 / 2, 2.0 / 2, 38.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(PCBYZL, 7, new TGeoTranslation((y_off - 0.17) / 2, 0, -15.0 + Z_dist_from_target));
   PCBYZL->SetLineColor(kGreen);

   // YZ plane (right beam side)
   TGeoVolume *siliconYZR_1 = gGeoManager->MakeBox("siliconYZR_1", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZR_1, 8, new TGeoTranslation(-y_off / 2, 0, 0.0 + Z_dist_from_target));
   siliconYZR_1->SetLineColor(kGray);

   TGeoVolume *siliconYZR_2 = gGeoManager->MakeBox("siliconYZR_2", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZR_2, 9, new TGeoTranslation(-y_off / 2, 0, -6.087 + Z_dist_from_target));
   siliconYZR_2->SetLineColor(kGray);

   TGeoVolume *siliconYZR_3 = gGeoManager->MakeBox("siliconYZR_3", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZR_3, 10, new TGeoTranslation(-y_off / 2, 0, -12.093 + Z_dist_from_target));
   siliconYZR_3->SetLineColor(kGray);

   TGeoVolume *siliconYZR_4 = gGeoManager->MakeBox("siliconYZR_4", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZR_4, 11, new TGeoTranslation(-y_off / 2, 0, -17.601 + Z_dist_from_target));
   siliconYZR_4->SetLineColor(kGray);

   TGeoVolume *siliconYZR_5 = gGeoManager->MakeBox("siliconYZR_5", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZR_5, 12, new TGeoTranslation(-y_off / 2, 0, -23.487 + Z_dist_from_target));
   siliconYZR_5->SetLineColor(kGray);

   TGeoVolume *siliconYZR_6 = gGeoManager->MakeBox("siliconYZR_6", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconYZR_5, 36, new TGeoTranslation(-y_off / 2, 0, -29.487 + Z_dist_from_target));
   siliconYZR_5->SetLineColor(kGray);

   TGeoVolume *PCBYZR = gGeoManager->MakeBox("PCBYZR", pcb, 0.1 / 2, 2.0 / 2, 38.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(PCBYZR, 13, new TGeoTranslation(-(y_off - 0.17) / 2, 0, -15.00 + Z_dist_from_target));
   PCBYZR->SetLineColor(kGreen);

   // XZ plane (top beam side)
   TGeoRotation *rTop = new TGeoRotation("rTop", 90, 0, 0);
   rTop->RegisterYourself();

   TGeoVolume *siliconXZT_1 = gGeoManager->MakeBox("siliconXZT_1", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZT_1, 14, new TGeoCombiTrans(0, x_off / 2.0, 0.0 + Z_dist_from_target, rTop));
   siliconXZT_1->SetLineColor(kGray);

   TGeoVolume *siliconXZT_2 = gGeoManager->MakeBox("siliconXZT_2", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZT_2, 15, new TGeoCombiTrans(0, x_off / 2.0, -6.087 + Z_dist_from_target, rTop));
   siliconXZT_2->SetLineColor(kGray);

   TGeoVolume *siliconXZT_3 = gGeoManager->MakeBox("siliconXZT_3", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZT_3, 16, new TGeoCombiTrans(0, x_off / 2.0, -12.093 + Z_dist_from_target, rTop));
   siliconXZT_3->SetLineColor(kGray);

   TGeoVolume *siliconXZT_4 = gGeoManager->MakeBox("siliconXZT_4", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZT_4, 17, new TGeoCombiTrans(0, x_off / 2.0, -17.601 + Z_dist_from_target, rTop));
   siliconXZT_4->SetLineColor(kGray);

   TGeoVolume *siliconXZT_5 = gGeoManager->MakeBox("siliconXZT_5", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZT_5, 18, new TGeoCombiTrans(0, x_off / 2.0, -23.487 + Z_dist_from_target, rTop));
   siliconXZT_5->SetLineColor(kGray);

   TGeoVolume *siliconXZT_6 = gGeoManager->MakeBox("siliconXZT_6", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZT_5, 37, new TGeoCombiTrans(0, x_off / 2.0, -29.487 + Z_dist_from_target, rTop));
   siliconXZT_5->SetLineColor(kGray);

   TGeoVolume *PCBXZT = gGeoManager->MakeBox("PCBXZT", pcb, 0.1 / 2, 2.0 / 2, 38.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(PCBXZT, 19, new TGeoCombiTrans(0, (x_off - 0.17) / 2, -15.00 + Z_dist_from_target, rTop));
   PCBXZT->SetLineColor(kGreen);

   // XZ plane (bottom beam side)
   TGeoRotation *rBottom = new TGeoRotation("rTop", -90, 0, 0);
   rBottom->RegisterYourself();

   TGeoVolume *siliconXZB_1 = gGeoManager->MakeBox("siliconXZB_1", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZB_1, 20, new TGeoCombiTrans(0, -x_off / 2.0, 0.0 + Z_dist_from_target, rBottom));
   siliconXZB_1->SetLineColor(kGray);

   TGeoVolume *siliconXZB_2 = gGeoManager->MakeBox("siliconXZB_2", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZB_2, 21, new TGeoCombiTrans(0, -x_off / 2.0, -6.087 + Z_dist_from_target, rBottom));
   siliconXZB_2->SetLineColor(kGray);

   TGeoVolume *siliconXZB_3 = gGeoManager->MakeBox("siliconXZB_3", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZB_3, 22, new TGeoCombiTrans(0, -x_off / 2.0, -12.093 + Z_dist_from_target, rBottom));
   siliconXZB_3->SetLineColor(kGray);

   TGeoVolume *siliconXZB_4 = gGeoManager->MakeBox("siliconXZB_4", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZB_4, 23, new TGeoCombiTrans(0, -x_off / 2.0, -17.601 + Z_dist_from_target, rBottom));
   siliconXZB_4->SetLineColor(kGray);

   TGeoVolume *siliconXZB_5 = gGeoManager->MakeBox("siliconXZB_5", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZB_4, 24, new TGeoCombiTrans(0, -x_off / 2.0, -23.487 + Z_dist_from_target, rBottom));
   siliconXZB_5->SetLineColor(kGray);

   TGeoVolume *siliconXZB_6 = gGeoManager->MakeBox("siliconXZB_6", silicon, 0.07 / 2, Si_width / 2, 5.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(siliconXZB_4, 38, new TGeoCombiTrans(0, -x_off / 2.0, -29.487 + Z_dist_from_target, rBottom));
   siliconXZB_5->SetLineColor(kGray);

   TGeoVolume *PCBXZB = gGeoManager->MakeBox("PCBXZB", pcb, 0.1 / 2, 2.0 / 2, 38.0 / 2);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(PCBXZB, 25, new TGeoCombiTrans(0, -(x_off - 0.17) / 2, -15.00 + Z_dist_from_target, rBottom));
   PCBXZB->SetLineColor(kGreen);

   // Annular recoil detectors
   TGeoVolume *QQQ_Sector1 = gGeoManager->MakeTubs("siliconQQQ_Sector1", silicon, 1.8 / 2, 10.0 / 2, .1 / 2, 4, 86);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(QQQ_Sector1, 26,
                new TGeoCombiTrans(0, 0, Recoil_Z_dist_from_target, new TGeoRotation("rQQQ1", 0, 0, 0)));
   QQQ_Sector1->SetLineColor(kGray);

   TGeoVolume *QQQ_Sector2 = gGeoManager->MakeTubs("siliconQQQ_Sector2", silicon, 1.8 / 2, 10.0 / 2, .1 / 2, 4, 86);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(QQQ_Sector2, 27,
                new TGeoCombiTrans(0, 0, Recoil_Z_dist_from_target, new TGeoRotation("rQQQ2", 0, 0, 90)));
   QQQ_Sector2->SetLineColor(kGray);

   TGeoVolume *QQQ_Sector3 = gGeoManager->MakeTubs("siliconQQQ_Sector3", silicon, 1.8 / 2, 10.0 / 2, .1 / 2, 4, 86);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(QQQ_Sector3, 28,
                new TGeoCombiTrans(0, 0, Recoil_Z_dist_from_target, new TGeoRotation("rQQQ3", 0, 0, 180)));
   QQQ_Sector3->SetLineColor(kGray);

   TGeoVolume *QQQ_Sector4 = gGeoManager->MakeTubs("siliconQQQ_Sector4", silicon, 1.8 / 2, 10.0 / 2, .1 / 2, 4, 86);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(QQQ_Sector4, 29,
                new TGeoCombiTrans(0, 0, Recoil_Z_dist_from_target, new TGeoRotation("rQQQ4", 0, 0, 270)));
   QQQ_Sector4->SetLineColor(kGray);

   // Annular recoil detectors PCB Support
   float PCB_thickness = 0.5; // cm
   TGeoTube *mainPCB = new TGeoTube("mainPCB", 1.150 / 2.0, 10.300 / 2, PCB_thickness / 2.0);
   TGeoTubeSeg *wedge1 = new TGeoTubeSeg("wedge1", 1.8 / 2, 10.0 / 2, (PCB_thickness + 0.01) / 2.0, 4, 86);
   TGeoTubeSeg *wedge2 = new TGeoTubeSeg("wedge2", 1.8 / 2, 10.0 / 2, (PCB_thickness + 0.01) / 2.0, 94, 176);
   TGeoTubeSeg *wedge3 = new TGeoTubeSeg("wedge3", 1.8 / 2, 10.0 / 2, (PCB_thickness + 0.01) / 2.0, 184, 266);
   TGeoTubeSeg *wedge4 = new TGeoTubeSeg("wedge4", 1.8 / 2, 10.0 / 2, (PCB_thickness + 0.01) / 2.0, 274, 356);

   TGeoCompositeShape *RecoilDetPCB = new TGeoCompositeShape("RecoilDetPCB", "(mainPCB-wedge1)-(wedge2+wedge3+wedge4)");
   TGeoVolume *RecoilDetPCBComp = new TGeoVolume("RecoilDetPCBComp", RecoilDetPCB);
   RecoilDetPCBComp->SetLineColor(kGreen);
   gGeoMan->GetVolume(geoVersion)
      ->AddNode(RecoilDetPCBComp, 30,
                new TGeoCombiTrans(0.0, 0.0, Recoil_Z_dist_from_target, new TGeoRotation("DetPCBr", 0, 0, 0)));

   // Target Holder
   TGeoRotation *r1 = new TGeoRotation("r1", 90, 90, 90);
   r1->RegisterYourself();

   TGeoBBox *targetHolderA = new TGeoBBox("TargetHolderA", 0.20 / 2.0, 4.0 / 2.0, 4.0 / 2.0);
   TGeoBBox *targetHolderB = new TGeoBBox("TargetHolderB", 0.20001 / 2.0, 3.0 / 2.0, 3.0 / 2.0);
   TGeoCompositeShape *targetHolder = new TGeoCompositeShape("targetHolder", "(TargetHolderA-TargetHolderB)");
   TGeoVolume *targetHolderComp = new TGeoVolume("TargetHolderComp", targetHolder);
   targetHolderComp->SetLineColor(kGray + 5);
   gGeoMan->GetVolume(geoVersion)->AddNode(targetHolderComp, 31, new TGeoCombiTrans(0.0, 0.0, 0.0, r1));

   // Target
   TGeoVolume *target = gGeoManager->MakeBox("siliconTarget", cd2, 4.0 / 2, 4.0 / 2, 0.000326 / 2);
   gGeoMan->GetVolume(geoVersion)->AddNode(target, 32, new TGeoTranslation(0.0, 0.0, 0.0));
   target->SetLineColor(kGray + 1);

   // Target
   /*TGeoVolume *target2 = gGeoManager->MakeBox("siliconTarget2", silicon,  100.0/2, 100.0/2, 0.0001/2);
   gGeoMan->GetVolume(geoVersion)->AddNode(target2,33, new TGeoTranslation(0.0,0.0,-1.0));
   target2->SetLineColor(kGray+1);*/

   /*TGeoVolume *drift_volume = gGeoManager->MakeTube("silicon_drift_volume",vacuum,0.0/2, 100/2, 100.0/2);
   gGeoMan->GetVolume(geoVersion)->AddNode(drift_volume,34, new TGeoTranslation(0,0,-5.0));
   drift_volume->SetTransparency(80);*/

   return siliconYZL_1;
}
