#include "AtTPC20MgDecay.h"

#include <FairLogger.h> // for Logger, LOG
#include <FairPrimaryGenerator.h>

#include <TF1.h>
#include <TH1.h>
#include <TMath.h>
#include <TRandom.h>

#include <cmath>    // for acos
#include <iostream> // for operator<<, endl, basic_ostream, cout
#include <memory>   // for make_unique, unique_ptr

Bool_t AtTPC20MgDecay::Init()
{
   // Initialize generatorTH1F*h1 = new TH1F("h1", "h1", 1000,0,11236.8);
   return true;
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPC20MgDecay::ReadEvent(FairPrimaryGenerator *primGen)
{

   if (fBoxVtxIsSet) {
      fX = gRandom->Uniform(fX1, fX2);
      fY = gRandom->Uniform(fY1, fY2);
      fZ = gRandom->Uniform(fZ1, fZ2);
   }

   // Proton of 1210keV and alpha of 506keV
   Int_t protonPDGID = 2212;
   Int_t alphaPDGID = 1000020040;
   Int_t gammaPDGID = 22;
   Int_t betaPDGID = 11;
   // Check for particle type

   // Protons from the decay of 20Na to 19Ne
   // Double32_t kinEneProton = 0.001210;  //GeV
   // Double32_t kinEneAlpha = 0.000506;  //GeV
   // Double32_t kinEneGamma =0.004033; //GeV  and it has zero rest mass
   Double32_t ptProton = 0, pxProton = 0, pyProton = 0, pzProton = 0;
   Double32_t pabsProton = 0.0469; // GeV/c    , 1.2 MeV
   Double32_t thetaProton = acos(gRandom->Uniform(-1, 1));
   Double32_t brp = 0;
   Double32_t phiProton = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton = pabsProton * TMath::Cos(thetaProton);
   ptProton = pabsProton * TMath::Sin(thetaProton);
   pxProton = ptProton * TMath::Cos(phiProton);
   pyProton = ptProton * TMath::Sin(phiProton);

   Double32_t ptAlpha = 0, pxAlpha = 0, pyAlpha = 0, pzAlpha = 0;
   // Double32_t bra=0;
   Double32_t pabsAlpha = 0.06162; // GeV/c, 506 keV from decay of 19Ne to 15O
   Double32_t thetaAlpha = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha = pabsAlpha * TMath::Cos(thetaAlpha);
   ptAlpha = pabsAlpha * TMath::Sin(thetaAlpha);
   pxAlpha = ptAlpha * TMath::Cos(phiAlpha);
   pyAlpha = ptAlpha * TMath::Sin(phiAlpha);

   Double32_t ptProton1 = 0, pxProton1 = 0, pyProton1 = 0, pzProton1 = 0;
   Double32_t pabsProton1 = 0.0389; // GeV/c , 806 keV
   Double32_t thetaProton1 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton1 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton1 = pabsProton1 * TMath::Cos(thetaProton1);
   ptProton1 = pabsProton1 * TMath::Sin(thetaProton1);
   pxProton1 = ptProton1 * TMath::Cos(phiProton1);
   pyProton1 = ptProton1 * TMath::Sin(phiProton1);

   Double32_t ptProton2 = 0, pxProton2 = 0, pyProton2 = 0, pzProton2 = 0;
   Double32_t pabsProton2 = 0.04476; // GeV/c, 1056 keV
   Double32_t thetaProton2 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton2 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton2 = pabsProton2 * TMath::Cos(thetaProton2);
   ptProton2 = pabsProton2 * TMath::Sin(thetaProton2);
   pxProton2 = ptProton2 * TMath::Cos(phiProton2);
   pyProton2 = ptProton2 * TMath::Sin(phiProton2);

   Double32_t ptProton3 = 0, pxProton3 = 0, pyProton3 = 0, pzProton3 = 0;
   Double32_t pabsProton3 = 0.05169; // GeV/c, 1416 keV
   Double32_t thetaProton3 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton3 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton3 = pabsProton3 * TMath::Cos(thetaProton3);
   ptProton3 = pabsProton3 * TMath::Sin(thetaProton3);
   pxProton3 = ptProton3 * TMath::Cos(phiProton3);
   pyProton3 = ptProton3 * TMath::Sin(phiProton3);

   Double32_t ptProton4 = 0, pxProton4 = 0, pyProton4 = 0, pzProton4 = 0;
   Double32_t pabsProton4 = 0.05636; // GeV/c, 1679 keV
   Double32_t thetaProton4 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton4 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton4 = pabsProton4 * TMath::Cos(thetaProton4);
   ptProton4 = pabsProton4 * TMath::Sin(thetaProton4);
   pxProton4 = ptProton4 * TMath::Cos(phiProton4);
   pyProton4 = ptProton4 * TMath::Sin(phiProton4);

   Double32_t ptProton5 = 0, pxProton5 = 0, pyProton5 = 0, pzProton5 = 0;
   Double32_t pabsProton5 = 0.06018; // GeV/c, 1928 keV
   Double32_t thetaProton5 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton5 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton5 = pabsProton5 * TMath::Cos(thetaProton5);
   ptProton5 = pabsProton5 * TMath::Sin(thetaProton5);
   pxProton5 = ptProton5 * TMath::Cos(phiProton5);
   pyProton5 = ptProton5 * TMath::Sin(phiProton5);

   Double32_t ptProton6 = 0, pxProton6 = 0, pyProton6 = 0, pzProton6 = 0;
   Double32_t pabsProton6 = 0.0651; // GeV/c, 2256 keV
   Double32_t thetaProton6 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton6 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton6 = pabsProton6 * TMath::Cos(thetaProton6);
   ptProton6 = pabsProton6 * TMath::Sin(thetaProton6);
   pxProton6 = ptProton6 * TMath::Cos(phiProton6);
   pyProton6 = ptProton6 * TMath::Sin(phiProton6);

   Double32_t ptProton7 = 0, pxProton7 = 0, pyProton7 = 0, pzProton7 = 0;
   Double32_t pabsProton7 = 0.06636; // GeV/c, 2344 keV
   Double32_t thetaProton7 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton7 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton7 = pabsProton7 * TMath::Cos(thetaProton7);
   ptProton7 = pabsProton7 * TMath::Sin(thetaProton7);
   pxProton7 = ptProton7 * TMath::Cos(phiProton7);
   pyProton7 = ptProton7 * TMath::Sin(phiProton7);

   Double32_t ptProton8 = 0, pxProton8 = 0, pyProton8 = 0, pzProton8 = 0;
   Double32_t pabsProton8 = 0.06934; // GeV/c, 2559 keV
   Double32_t thetaProton8 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton8 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton8 = pabsProton8 * TMath::Cos(thetaProton8);
   ptProton8 = pabsProton8 * TMath::Sin(thetaProton8);
   pxProton8 = ptProton8 * TMath::Cos(phiProton8);
   pyProton8 = ptProton8 * TMath::Sin(phiProton8);

   Double32_t ptProton9 = 0, pxProton9 = 0, pyProton9 = 0, pzProton9 = 0;
   Double32_t pabsProton9 = 0.07513; // GeV/c, 3003 keV
   Double32_t thetaProton9 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton9 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton9 = pabsProton9 * TMath::Cos(thetaProton9);
   ptProton9 = pabsProton9 * TMath::Sin(thetaProton9);
   pxProton9 = ptProton9 * TMath::Cos(phiProton9);
   pyProton9 = ptProton9 * TMath::Sin(phiProton9);

   Double32_t ptProton10 = 0, pxProton10 = 0, pyProton10 = 0, pzProton10 = 0;
   Double32_t pabsProton10 = 0.07982; // GeV/c, 3389 keV
   Double32_t thetaProton10 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton10 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton10 = pabsProton10 * TMath::Cos(thetaProton10);
   ptProton10 = pabsProton10 * TMath::Sin(thetaProton10);
   pxProton10 = ptProton10 * TMath::Cos(phiProton10);
   pyProton10 = ptProton10 * TMath::Sin(phiProton10);

   Double32_t ptProton11 = 0, pxProton11 = 0, pyProton11 = 0, pzProton11 = 0;
   Double32_t pabsProton11 = 0.08475; // GeV/c, 3820 keV
   Double32_t thetaProton11 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton11 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton11 = pabsProton11 * TMath::Cos(thetaProton11);
   ptProton11 = pabsProton11 * TMath::Sin(thetaProton11);
   pxProton11 = ptProton11 * TMath::Cos(phiProton11);
   pyProton11 = ptProton11 * TMath::Sin(phiProton11);

   Double32_t ptProton12 = 0, pxProton12 = 0, pyProton12 = 0, pzProton12 = 0;
   Double32_t pabsProton12 = 0.0875; // GeV/c, 4071 keV
   Double32_t thetaProton12 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton12 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton12 = pabsProton12 * TMath::Cos(thetaProton12);
   ptProton12 = pabsProton12 * TMath::Sin(thetaProton12);
   pxProton12 = ptProton12 * TMath::Cos(phiProton12);
   pyProton12 = ptProton12 * TMath::Sin(phiProton12);

   Double32_t ptProton13 = 0, pxProton13 = 0, pyProton13 = 0, pzProton13 = 0;
   Double32_t pabsProton13 = 0.0902; // GeV/c, 4326 keV
   Double32_t thetaProton13 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton13 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton13 = pabsProton13 * TMath::Cos(thetaProton13);
   ptProton13 = pabsProton13 * TMath::Sin(thetaProton13);
   pxProton13 = ptProton13 * TMath::Cos(phiProton13);
   pyProton13 = ptProton13 * TMath::Sin(phiProton13);

   // beta particles emitted from 20Mg decay to 20Na which will undergo proton emission
   auto h1 = std::make_unique<TH1F>("h1", "h1", 1000, 0, 8650); // 984 keV
   auto f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 8650);
   f1->SetParameters(1.4373e-6, 2.8193e-8, 1.7812e-11, -5.31546e-15, 3.35539e-19, -3.31743e-25);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r1 = f1->GetRandom();
   // std::cout<<"r1="<<r1<<std::endl;
   Double32_t ptBeta1 = 0, pxBeta1 = 0, pyBeta1 = 0, pzBeta1 = 0;
   Double32_t pabsBeta1 = (TMath::Sqrt((r1 + 511) * (r1 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta1="<<pabsBeta1<<std::endl;

   Double32_t brbeta = 0;
   Double32_t brb = 0;
   Double32_t thetaBeta1 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta1 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta1 = pabsBeta1 * TMath::Cos(thetaBeta1);
   ptBeta1 = pabsBeta1 * TMath::Sin(thetaBeta1);
   pxBeta1 = ptBeta1 * TMath::Cos(phiBeta1);
   pyBeta1 = ptBeta1 * TMath::Sin(phiBeta1);

   h1 = std::make_unique<TH1F>("h2", "h2", 1000, 0, 7000); // 2645 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 7000);
   f1->SetParameters(3.21795e-9, 7.31673e-11, 4.05604e-14, -1.61734e-17, 1.25431e-21, 2.00144e-27);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r2 = f1->GetRandom();
   // std::cout<<"r2="<<r2<<std::endl;

   Double32_t ptBeta2 = 0, pxBeta2 = 0, pyBeta2 = 0, pzBeta2 = 0;
   Double32_t pabsBeta2 = (TMath::Sqrt((r2 + 511) * (r2 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta2="<<pabsBeta2<<std::endl;

   Double32_t thetaBeta2 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta2 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta2 = pabsBeta2 * TMath::Cos(thetaBeta2);
   ptBeta2 = pabsBeta2 * TMath::Sin(thetaBeta2);
   pxBeta2 = ptBeta2 * TMath::Cos(phiBeta2);
   pyBeta2 = ptBeta2 * TMath::Sin(phiBeta2);

   h1 = std::make_unique<TH1F>("h3", "h3", 1000, 0, 6600); // 3001 kev
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 6600);
   f1->SetParameters(4.12126e-7, 9.72611e-9, 5.16976e-12, -2.22576e-15, 1.81306e-19, 4.8888e-25);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r3 = f1->GetRandom();
   // std::cout<<"r3="<<r3<<std::endl;

   Double32_t ptBeta3 = 0, pxBeta3 = 0, pyBeta3 = 0, pzBeta3 = 0;
   Double32_t pabsBeta3 = (TMath::Sqrt((r3 + 511) * (r3 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta3="<<pabsBeta3<<std::endl;

   Double32_t thetaBeta3 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta3 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta3 = pabsBeta3 * TMath::Cos(thetaBeta3);
   ptBeta3 = pabsBeta3 * TMath::Sin(thetaBeta3);
   pxBeta3 = ptBeta3 * TMath::Cos(phiBeta3);
   pyBeta3 = ptBeta3 * TMath::Sin(phiBeta3);

   h1 = std::make_unique<TH1F>("h4", "h4", 1000, 0, 5750); // 3874 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 5750);
   f1->SetParameters(2.28812e-7, 5.99596e-9, 2.76505e-12, -1.49003e-15, 1.37999e-19, 1.00668e-24);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r4 = f1->GetRandom();
   // std::cout<<"r4="<<r4<<std::endl;

   Double32_t ptBeta4 = 0, pxBeta4 = 0, pyBeta4 = 0, pzBeta4 = 0;
   Double32_t pabsBeta4 = (TMath::Sqrt((r4 + 511) * (r4 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta4="<<pabsBeta4<<std::endl;

   Double32_t thetaBeta4 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta4 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta4 = pabsBeta4 * TMath::Cos(thetaBeta4);
   ptBeta4 = pabsBeta4 * TMath::Sin(thetaBeta4);
   pxBeta4 = ptBeta4 * TMath::Cos(phiBeta4);
   pyBeta4 = ptBeta4 * TMath::Sin(phiBeta4);

   h1 = std::make_unique<TH1F>("h5", "h5", 1000, 0, 5500); // 4123 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 5500);
   f1->SetParameters(1.40485e-7, 3.80994e-9, 1.66204e-12, -9.67707e-16, 9.31417e-20, 8.78505e-25);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r5 = f1->GetRandom();
   // std::cout<<"r5="<<r5<<std::endl;

   Double32_t ptBeta5 = 0, pxBeta5 = 0, pyBeta5 = 0, pzBeta5 = 0;
   Double32_t pabsBeta5 = (TMath::Sqrt((r5 + 511) * (r5 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta5="<<pabsBeta5<<std::endl;

   Double32_t thetaBeta5 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta5 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta5 = pabsBeta5 * TMath::Cos(thetaBeta5);
   ptBeta5 = pabsBeta5 * TMath::Sin(thetaBeta5);
   pxBeta5 = ptBeta5 * TMath::Cos(phiBeta5);
   pyBeta5 = ptBeta5 * TMath::Sin(phiBeta5);

   h1 = std::make_unique<TH1F>("h6", "h6", 1000, 0, 4810); // 4800 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 4810);
   f1->SetParameters(1.27496e-7, 3.84163e-9, 1.35989e-12, -1.02747e-15, 1.10166e-19, 2.06696e-24);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r6 = f1->GetRandom();
   // std::cout<<"r6="<<r6<<std::endl;

   Double32_t ptBeta6 = 0, pxBeta6 = 0, pyBeta6 = 0, pzBeta6 = 0;
   Double32_t pabsBeta6 = (TMath::Sqrt((r6 + 511) * (r6 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta6="<<pabsBeta6<<std::endl;
   Double32_t thetaBeta6 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta6 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta6 = pabsBeta6 * TMath::Cos(thetaBeta6);
   ptBeta6 = pabsBeta6 * TMath::Sin(thetaBeta6);
   pxBeta6 = ptBeta6 * TMath::Cos(phiBeta6);
   pyBeta6 = ptBeta6 * TMath::Sin(phiBeta6);

   h1 = std::make_unique<TH1F>("h7", "h7", 1000, 0, 4010); // 5600
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 4010);
   f1->SetParameters(1.40707e-7, 4.9678e-9, 1.07582e-12, -1.36341e-15, 1.65065e-19, 7.62092e-24);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r7 = f1->GetRandom();
   // std::cout<<"r7="<<r7<<std::endl;

   Double32_t ptBeta7 = 0, pxBeta7 = 0, pyBeta7 = 0, pzBeta7 = 0;
   Double32_t pabsBeta7 = (TMath::Sqrt((r7 + 511) * (r7 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta7="<<pabsBeta7<<std::endl;

   Double32_t thetaBeta7 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta7 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta7 = pabsBeta7 * TMath::Cos(thetaBeta7);
   ptBeta7 = pabsBeta7 * TMath::Sin(thetaBeta7);
   pxBeta7 = ptBeta7 * TMath::Cos(phiBeta7);
   pyBeta7 = ptBeta7 * TMath::Sin(phiBeta7);

   h1 = std::make_unique<TH1F>("h8", "h8", 1000, 0, 3770.); // 5836 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 3770);
   f1->SetParameters(7.89154e-7, 2.94918e-8, 4.8237e-12, -8.02751e-15, 9.98793e-19, 6.29715e-23);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r8 = f1->GetRandom();
   // std::cout<<"r8="<<r8<<std::endl;

   Double32_t ptBeta8 = 0, pxBeta8 = 0, pyBeta8 = 0, pzBeta8 = 0;
   Double32_t pabsBeta8 = (TMath::Sqrt((r8 + 511) * (r8 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta8="<<pabsBeta8<<std::endl;
   Double32_t thetaBeta8 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta8 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta8 = pabsBeta8 * TMath::Cos(thetaBeta8);
   ptBeta8 = pabsBeta8 * TMath::Sin(thetaBeta8);
   pxBeta8 = ptBeta8 * TMath::Cos(phiBeta8);
   pyBeta8 = ptBeta8 * TMath::Sin(phiBeta8);

   h1 = std::make_unique<TH1F>("h9", "h9", 1000, 0, 3340); // 6266 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 3340);
   f1->SetParameters(1.5398e-7, 6.4777e-9, 3.04221e-13, -1.67316e-15, 2.11676e-19, 2.6079e-23);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r9 = f1->GetRandom();
   // std::cout<<"r9="<<r9<<std::endl;

   Double32_t ptBeta9 = 0, pxBeta9 = 0, pyBeta9 = 0, pzBeta9 = 0;
   Double32_t pabsBeta9 = (TMath::Sqrt((r9 + 511) * (r9 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta9="<<pabsBeta9<<std::endl;

   Double32_t thetaBeta9 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta9 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta9 = pabsBeta9 * TMath::Cos(thetaBeta9);
   ptBeta9 = pabsBeta9 * TMath::Sin(thetaBeta9);
   pxBeta9 = ptBeta9 * TMath::Cos(phiBeta9);
   pyBeta9 = ptBeta9 * TMath::Sin(phiBeta9);

   h1 = std::make_unique<TH1F>("h10", "h10", 1000, 0, 3071); // 6534
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 3071);
   f1->SetParameters(4.84941e-7, 2.22562e-8, -9.262068e-13, -5.32068e-15, 6.40919e-19, 1.36991e-22);
   h1->FillRandom("f1");
   h1->Draw();

   Double32_t r10 = f1->GetRandom();
   // std::cout<<"r10="<<r10<<std::endl;

   Double32_t ptBeta10 = 0, pxBeta10 = 0, pyBeta10 = 0, pzBeta10 = 0;
   Double32_t pabsBeta10 = (TMath::Sqrt((r10 + 511) * (r10 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta10="<<pabsBeta1<<std::endl;
   Double32_t thetaBeta10 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta10 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta10 = pabsBeta10 * TMath::Cos(thetaBeta10);
   ptBeta10 = pabsBeta10 * TMath::Sin(thetaBeta10);
   pxBeta10 = ptBeta10 * TMath::Cos(phiBeta10);
   pyBeta10 = ptBeta10 * TMath::Sin(phiBeta10);

   h1 = std::make_unique<TH1F>("h11", "h11", 1000, 0, 2835); // 6770 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 2835);
   f1->SetParameters(1.33516e-6, 6.68463e-8, -9.17265e-12, -1.41377e-14, 1.4453e-18, 6.10901e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r11 = f1->GetRandom();
   // std::cout<<"r11="<<r11<<std::endl;

   Double32_t ptBeta11 = 0, pxBeta11 = 0, pyBeta11 = 0, pzBeta11 = 0;
   Double32_t pabsBeta11 = (TMath::Sqrt((r11 + 511) * (r11 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta11="<<pabsBeta11<<std::endl;
   Double32_t thetaBeta11 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta11 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta11 = pabsBeta11 * TMath::Cos(thetaBeta11);
   ptBeta11 = pabsBeta11 * TMath::Sin(thetaBeta11);
   pxBeta11 = ptBeta11 * TMath::Cos(phiBeta11);
   pyBeta11 = ptBeta11 * TMath::Sin(phiBeta11);

   h1 = std::make_unique<TH1F>("h12", "h12", 1000, 0, 2685); // 6920 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 2685);
   f1->SetParameters(9.03068e-7, 4.81333e-8, -9.93371e-12, -8.9499e-15, 6.66662e-19, 5.74898e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r12 = f1->GetRandom();
   // std::cout<<"r12="<<r12<<std::endl;

   Double32_t ptBeta12 = 0, pxBeta12 = 0, pyBeta12 = 0, pzBeta12 = 0;
   Double32_t pabsBeta12 = (TMath::Sqrt((r12 + 511) * (r12 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta12="<<pabsBeta12<<std::endl;

   Double32_t thetaBeta12 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta12 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta12 = pabsBeta12 * TMath::Cos(thetaBeta12);
   ptBeta12 = pabsBeta12 * TMath::Sin(thetaBeta12);
   pxBeta12 = ptBeta12 * TMath::Cos(phiBeta12);
   pyBeta12 = ptBeta12 * TMath::Sin(phiBeta12);

   h1 = std::make_unique<TH1F>("h13", "h13", 1000, 0, 2165); // 7440 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 2165);
   f1->SetParameters(2.42189e-9, 1.68134e-10, -8.74415e-14, -3.26775e-18, -8.54988e-21, 5.56934e-24);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r13 = f1->GetRandom();
   // std::cout<<"r13="<<r13<<std::endl;

   Double32_t ptBeta13 = 0, pxBeta13 = 0, pyBeta13 = 0, pzBeta13 = 0;
   Double32_t pabsBeta13 = (TMath::Sqrt((r13 + 511) * (r13 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta13="<<pabsBeta13<<std::endl;

   Double32_t thetaBeta13 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta13 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta13 = pabsBeta13 * TMath::Cos(thetaBeta13);
   ptBeta13 = pabsBeta13 * TMath::Sin(thetaBeta13);
   pxBeta13 = ptBeta13 * TMath::Cos(phiBeta13);
   pyBeta13 = ptBeta13 * TMath::Sin(phiBeta13);

   // Beta particles from decay of 20Na to 20Ne

   h1 = std::make_unique<TH1F>("h14", "h14", 1000, 0, 11236.8);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 11236.8);
   f1->SetParameters(1.59711e-7, 1.68487e-8, 1.0013e-11, -2.1646e-15, 9.92165e-20, 2.01015e-25);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r14 = f1->GetRandom();
   // std::cout<<"r1="<<r1<<std::endl;

   Double32_t ptBeta14 = 0, pxBeta14 = 0, pyBeta14 = 0, pzBeta14 = 0;
   Double32_t pabsBeta14 = (TMath::Sqrt((r14 + 511) * (r14 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta14="<<pabsBeta14<<std::endl;

   Double32_t thetaBeta14 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta14 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta14 = pabsBeta14 * TMath::Cos(thetaBeta14);
   ptBeta14 = pabsBeta14 * TMath::Sin(thetaBeta14);
   pxBeta14 = ptBeta14 * TMath::Cos(phiBeta14);
   pyBeta14 = ptBeta14 * TMath::Sin(phiBeta14);

   h1 = std::make_unique<TH1F>("h15", "h15", 1000, 0, 5448.6); // 11320 keV
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 5448.6);
   f1->SetParameters(3.1588e-8, 1.25472e-9, 6.2571e-14, -3.16054e-16, 3.84227e-20, 5.00163e-24);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r15 = f1->GetRandom();

   // std::cout<<"r15="<<r15<<std::endl;
   Double32_t ptBeta15 = 0, pxBeta15 = 0, pyBeta15 = 0, pzBeta15 = 0;
   Double32_t pabsBeta15 = (TMath::Sqrt((r15 + 511) * (r15 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta15="<<pabsBeta15<<std::endl;

   Double32_t thetaBeta15 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta15 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta15 = pabsBeta15 * TMath::Cos(thetaBeta15);
   ptBeta15 = pabsBeta15 * TMath::Sin(thetaBeta15);
   pxBeta15 = ptBeta15 * TMath::Cos(phiBeta15);
   pyBeta15 = ptBeta15 * TMath::Sin(phiBeta15);

   h1 = std::make_unique<TH1F>("h16", "h16", 1000, 0, 5041.5);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 5041.5);
   f1->SetParameters(4.55e-9, 2.01854e-10, -1.63887e-14, -4.4892e-17, 4.87227e-21, 1.46295e-24);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r16 = f1->GetRandom();
   // std::cout<<"r16="<<r16<<std::endl;

   Double32_t ptBeta16 = 0, pxBeta16 = 0, pyBeta16 = 0, pzBeta16 = 0;
   Double32_t pabsBeta16 = (TMath::Sqrt((r16 + 511) * (r16 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta6="<<pabsBeta6<<std::endl;
   Double32_t thetaBeta16 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta16 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta16 = pabsBeta16 * TMath::Cos(thetaBeta16);
   ptBeta16 = pabsBeta16 * TMath::Sin(thetaBeta16);
   pxBeta16 = ptBeta16 * TMath::Cos(phiBeta16);
   pyBeta16 = ptBeta16 * TMath::Sin(phiBeta16);

   h1 = std::make_unique<TH1F>("h17", "h17", 1000, 0, 4070.5);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 4070.5);
   f1->SetParameters(5.85765e-7, 3.02486e-8, -7.97958e-12, -4.63986e-15, 7.78501e-20, 4.44419e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r17 = f1->GetRandom();
   // std::cout<<"r17="<<r17<<std::endl;

   Double32_t ptBeta17 = 0, pxBeta17 = 0, pyBeta17 = 0, pzBeta17 = 0;
   Double32_t pabsBeta17 = (TMath::Sqrt((r17 + 511) * (r17 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta17="<<pabsBeta17<<std::endl;

   Double32_t thetaBeta17 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta17 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta17 = pabsBeta17 * TMath::Cos(thetaBeta17);
   ptBeta17 = pabsBeta17 * TMath::Sin(thetaBeta17);
   pxBeta17 = ptBeta17 * TMath::Cos(phiBeta17);
   pyBeta17 = ptBeta17 * TMath::Sin(phiBeta17);

   h1 = std::make_unique<TH1F>("h18", "h18", 1000, 0, 6000.);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 3383.5);
   f1->SetParameters(2.16857e-8, 1.29089e-9, -5.80327e-13, -7.02664e-17, -4.61969e-20, 3.46765e-23);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r18 = f1->GetRandom();
   // std::cout<<"r18="<<r18<<std::endl;

   Double32_t ptBeta18 = 0, pxBeta18 = 0, pyBeta18 = 0, pzBeta18 = 0;
   Double32_t pabsBeta18 = (TMath::Sqrt((r18 + 511) * (r18 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta81="<<pabsBeta8<<std::endl;
   Double32_t thetaBeta18 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta18 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta18 = pabsBeta18 * TMath::Cos(thetaBeta18);
   ptBeta18 = pabsBeta18 * TMath::Sin(thetaBeta18);
   pxBeta18 = ptBeta18 * TMath::Cos(phiBeta18);
   pyBeta18 = ptBeta18 * TMath::Sin(phiBeta18);

   h1 = std::make_unique<TH1F>("h19", "h19", 1000, 0, 2997.5);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 2997.5);
   f1->SetParameters(5.06675e-8, 3.48106e-9, -2.25971e-12, 2.97006e-16, -3.48781e-19, 1.63705e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r19 = f1->GetRandom();
   // std::cout<<"r19="<<r19<<std::endl;

   Double32_t ptBeta19 = 0, pxBeta19 = 0, pyBeta19 = 0, pzBeta19 = 0;
   Double32_t pabsBeta19 = (TMath::Sqrt((r19 + 511) * (r19 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta19="<<pabsBeta19<<std::endl;

   Double32_t thetaBeta19 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta19 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta19 = pabsBeta19 * TMath::Cos(thetaBeta19);
   ptBeta19 = pabsBeta9 * TMath::Sin(thetaBeta19);
   pxBeta19 = ptBeta19 * TMath::Cos(phiBeta19);
   pyBeta19 = ptBeta19 * TMath::Sin(phiBeta19);

   h1 = std::make_unique<TH1F>("h20", "h20", 1000, 0, 2596.5);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 2596.5);
   f1->SetParameters(3.50494e-8, 2.46542e-9, -1.68544e-12, 2.73518e-16, -2.77506e-19, 1.26138e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r20 = f1->GetRandom();
   // std::cout<<"r20="<<r20<<std::endl;

   Double32_t ptBeta20 = 0, pxBeta20 = 0, pyBeta20 = 0, pzBeta20 = 0;
   Double32_t pabsBeta20 = (TMath::Sqrt((r20 + 511) * (r20 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta20="<<pabsBeta20<<std::endl;
   Double32_t thetaBeta20 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta20 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta20 = pabsBeta20 * TMath::Cos(thetaBeta20);
   ptBeta20 = pabsBeta20 * TMath::Sin(thetaBeta20);
   pxBeta20 = ptBeta20 * TMath::Cos(phiBeta20);
   pyBeta20 = ptBeta20 * TMath::Sin(phiBeta20);

   h1 = std::make_unique<TH1F>("h21", "h21", 1000, 0, 2286.5);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 2286.5);
   f1->SetParameters(7.84682e-8, 7.34482e-9, -8.13364e-12, 3.91808e-15, -2.64262e-18, 9.8004e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r21 = f1->GetRandom();
   // std::cout<<"r21="<<r21<<std::endl;

   Double32_t ptBeta21 = 0, pxBeta21 = 0, pyBeta21 = 0, pzBeta21 = 0;
   Double32_t pabsBeta21 = (TMath::Sqrt((r21 + 511) * (r21 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta21="<<pabsBeta21<<std::endl;
   Double32_t thetaBeta21 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta21 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta21 = pabsBeta21 * TMath::Cos(thetaBeta21);
   ptBeta21 = pabsBeta21 * TMath::Sin(thetaBeta21);
   pxBeta21 = ptBeta21 * TMath::Cos(phiBeta21);
   pyBeta21 = ptBeta21 * TMath::Sin(phiBeta21);

   h1 = std::make_unique<TH1F>("h22", "h22", 1000, 0, 2027.5);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 2027.5);
   f1->SetParameters(1.1966e-8, 1.17991e-9, -1.40285e-12, 7.42593e-16, -4.98985e-19, 1.84071e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r22 = f1->GetRandom();
   // std::cout<<"r22="<<r22<<std::endl;

   Double32_t ptBeta22 = 0, pxBeta22 = 0, pyBeta22 = 0, pzBeta22 = 0;
   Double32_t pabsBeta22 = (TMath::Sqrt((r22 + 511) * (r22 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta22="<<pabsBetav<<std::endl;

   Double32_t thetaBeta22 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta22 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta22 = pabsBeta22 * TMath::Cos(thetaBeta22);
   ptBeta22 = pabsBeta22 * TMath::Sin(thetaBeta22);
   pxBeta22 = ptBeta22 * TMath::Cos(phiBeta22);
   pyBeta22 = ptBeta22 * TMath::Sin(phiBeta22);

   h1 = std::make_unique<TH1F>("h23", "h23", 1000, 0, 1986.5);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 1986.5);
   f1->SetParameters(8.82998e-10, 1.87941e-10, -4.67582e-13, 5.54255e-16, -4.80667e-19, 2.05681e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r23 = f1->GetRandom();
   // std::cout<<"r23="<<r13<<std::endl;

   Double32_t ptBeta23 = 0, pxBeta23 = 0, pyBeta23 = 0, pzBeta23 = 0;
   Double32_t pabsBeta23 = (TMath::Sqrt((r23 + 511) * (r23 + 511) - (511 * 511))) * 1e-6;

   // std::cout<<"pabsBeta23="<<pabsBeta23<<std::endl;

   Double32_t thetaBeta23 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta23 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta23 = pabsBeta23 * TMath::Cos(thetaBeta23);
   ptBeta23 = pabsBeta23 * TMath::Sin(thetaBeta23);
   pxBeta23 = ptBeta23 * TMath::Cos(phiBeta23);
   pyBeta23 = ptBeta23 * TMath::Sin(phiBeta23);

   h1 = std::make_unique<TH1F>("h24", "h24", 1000, 0, 1605);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 1605);
   f1->SetParameters(7.8520e-8, 7.34997e-9, -8.14208e-12, 3.92149e-15, -2.6442e-18, 9.80965e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r24 = f1->GetRandom();
   // std::cout<<"r24="<<r24<<std::endl;

   Double32_t ptBeta24 = 0, pxBeta24 = 0, pyBeta24 = 0, pzBeta24 = 0;
   Double32_t pabsBeta24 = (TMath::Sqrt((r24 + 511) * (r24 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta24="<<pabsBeta24<<std::endl;
   Double32_t thetaBeta24 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta24 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta24 = pabsBeta24 * TMath::Cos(thetaBeta24);
   ptBeta24 = pabsBeta24 * TMath::Sin(thetaBeta24);
   pxBeta24 = ptBeta24 * TMath::Cos(phiBeta24);
   pyBeta24 = ptBeta24 * TMath::Sin(phiBeta24);

   h1 = std::make_unique<TH1F>("h25", "h25", 1000, 0, 1550);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 1550);
   f1->SetParameters(1.43596e-8, 1.41593e-9, -1.68346e-12, 8.91141e-16, -5.98803e-19, 2.20892e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r25 = f1->GetRandom();
   // std::cout<<"r25="<<r25<<std::endl;

   Double32_t ptBeta25 = 0, pxBeta25 = 0, pyBeta25 = 0, pzBeta25 = 0;
   Double32_t pabsBeta25 = (TMath::Sqrt((r25 + 511) * (r25 + 511) - (511 * 511))) * 1e-6;
   // std::cout<<"pabsBeta25="<<pabsBeta25<<std::endl;

   Double32_t thetaBeta25 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta25 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta25 = pabsBeta25 * TMath::Cos(thetaBeta25);
   ptBeta25 = pabsBeta25 * TMath::Sin(thetaBeta25);
   pxBeta25 = ptBeta25 * TMath::Cos(phiBeta25);
   pyBeta25 = ptBeta25 * TMath::Sin(phiBeta25);

   h1 = std::make_unique<TH1F>("h26", "h26", 900, 0, 984);
   f1 = std::make_unique<TF1>("f1", "[0]+[1]*x+[2]*x*x+[3]*x*x*x+[4]*x*x*x*x+[5]*x*x*x*x*x", 0, 984);
   f1->SetParameters(8.82998e-10, 1.87941e-10, -4.67582e-13, 5.54255e-16, -4.80667e-19, 2.05681e-22);
   h1->FillRandom("f1");
   h1->Draw();
   Double32_t r26 = f1->GetRandom();
   // std::cout<<"r26="<<r26<<std::endl;
   //
   Double32_t ptBeta26 = 0, pxBeta26 = 0, pyBeta26 = 0, pzBeta26 = 0;
   Double32_t pabsBeta26 = (TMath::Sqrt((r26 + 511) * (r26 + 511) - (511 * 511))) * 1e-6;
   //::cout<<"pabsBeta26="<<pabsBeta26<<std::endl;

   Double32_t thetaBeta26 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiBeta26 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzBeta26 = pabsBeta26 * TMath::Cos(thetaBeta26);
   ptBeta26 = pabsBeta26 * TMath::Sin(thetaBeta26);
   pxBeta26 = ptBeta26 * TMath::Cos(phiBeta26);
   pyBeta26 = ptBeta26 * TMath::Sin(phiBeta26);

   // Alpha particles from decay of 20Ne to 16O

   Double32_t ptAlpha1 = 0, pxAlpha1 = 0, pyAlpha1 = 0, pzAlpha1 = 0;
   Double32_t pabsAlpha1 = 0.2328; // GeV/c, 7.26 MeV
   Double32_t thetaAlpha1 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha1 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha1 = pabsAlpha1 * TMath::Cos(thetaAlpha1);
   ptAlpha1 = pabsAlpha1 * TMath::Sin(thetaAlpha1);
   pxAlpha1 = ptAlpha1 * TMath::Cos(phiAlpha1);
   pyAlpha1 = ptAlpha1 * TMath::Sin(phiAlpha1);

   Double32_t ptAlpha2 = 0, pxAlpha2 = 0, pyAlpha2 = 0, pzAlpha2 = 0;
   Double32_t pabsAlpha2 = 0.2204; // GeV/c, 6.561 MeV
   Double32_t thetaAlpha2 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha2 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha2 = pabsAlpha2 * TMath::Cos(thetaAlpha2);
   ptAlpha2 = pabsAlpha2 * TMath::Sin(thetaAlpha2);
   pxAlpha2 = ptAlpha2 * TMath::Cos(phiAlpha2);
   pyAlpha2 = ptAlpha2 * TMath::Sin(phiAlpha2);

   Double32_t ptAlpha3 = 0, pxAlpha3 = 0, pyAlpha3 = 0, pzAlpha3 = 0;
   Double32_t pabsAlpha3 = 0.2134; // GeV/c, 6.106 Mev
   Double32_t thetaAlpha3 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha3 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha3 = pabsAlpha3 * TMath::Cos(thetaAlpha3);
   ptAlpha3 = pabsAlpha3 * TMath::Sin(thetaAlpha3);
   pxAlpha3 = ptAlpha3 * TMath::Cos(phiAlpha3);
   pyAlpha3 = ptAlpha3 * TMath::Sin(phiAlpha3);

   Double32_t ptAlpha4 = 0, pxAlpha4 = 0, pyAlpha4 = 0, pzAlpha4 = 0;
   Double32_t pabsAlpha4 = 0.2088; // GeV/c, 5.844 MeV
   Double32_t thetaAlpha4 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha4 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha4 = pabsAlpha4 * TMath::Cos(thetaAlpha4);
   ptAlpha4 = pabsAlpha4 * TMath::Sin(thetaAlpha4);
   pxAlpha4 = ptAlpha4 * TMath::Cos(phiAlpha4);
   pyAlpha4 = ptAlpha4 * TMath::Sin(phiAlpha4);

   Double32_t ptAlpha5 = 0, pxAlpha5 = 0, pyAlpha5 = 0, pzAlpha5 = 0;
   Double32_t pabsAlpha5 = 0.2033; // GeV/c, 5.540
   Double32_t thetaAlpha5 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha5 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha5 = pabsAlpha5 * TMath::Cos(thetaAlpha5);
   ptAlpha5 = pabsAlpha5 * TMath::Sin(thetaAlpha5);
   pxAlpha5 = ptAlpha5 * TMath::Cos(phiAlpha5);
   pyAlpha5 = ptAlpha5 * TMath::Sin(phiAlpha5);

   Double32_t ptAlpha6 = 0, pxAlpha6 = 0, pyAlpha6 = 0, pzAlpha6 = 0;
   Double32_t pabsAlpha6 = 0.1882; // GeV/c, 4.749 MeV
   Double32_t thetaAlpha6 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha6 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha6 = pabsAlpha6 * TMath::Cos(thetaAlpha6);
   ptAlpha6 = pabsAlpha6 * TMath::Sin(thetaAlpha6);
   pxAlpha6 = ptAlpha6 * TMath::Cos(phiAlpha6);
   pyAlpha6 = ptAlpha6 * TMath::Sin(phiAlpha6);

   Double32_t ptAlpha7 = 0, pxAlpha7 = 0, pyAlpha7 = 0, pzAlpha7 = 0;
   Double32_t pabsAlpha7 = 0.1757; // GeV/c, 4.140 MeV
   Double32_t thetaAlpha7 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha7 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha7 = pabsAlpha7 * TMath::Cos(thetaAlpha7);
   ptAlpha7 = pabsAlpha7 * TMath::Sin(thetaAlpha7);
   pxAlpha7 = ptAlpha7 * TMath::Cos(phiAlpha7);
   pyAlpha7 = ptAlpha7 * TMath::Sin(phiAlpha7);

   Double32_t ptAlpha8 = 0, pxAlpha8 = 0, pyAlpha8 = 0, pzAlpha8 = 0;
   Double32_t pabsAlpha8 = 0.152; // GeV/c, 3.099 MeV
   Double32_t thetaAlpha8 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha8 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha8 = pabsAlpha8 * TMath::Cos(thetaAlpha8);
   ptAlpha8 = pabsAlpha8 * TMath::Sin(thetaAlpha8);
   pxAlpha8 = ptAlpha8 * TMath::Cos(phiAlpha8);
   pyAlpha8 = ptAlpha8 * TMath::Sin(phiAlpha8);

   Double32_t ptAlpha9 = 0, pxAlpha9 = 0, pyAlpha9 = 0, pzAlpha9 = 0;
   Double32_t pabsAlpha9 = 0.1417; // GeV/c, 2.692 MeV
   Double32_t thetaAlpha9 = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha9 = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha9 = pabsAlpha9 * TMath::Cos(thetaAlpha9);
   ptAlpha9 = pabsAlpha9 * TMath::Sin(thetaAlpha9);
   pxAlpha9 = ptAlpha9 * TMath::Cos(phiAlpha9);
   pyAlpha9 = ptAlpha9 * TMath::Sin(phiAlpha9);

   Double32_t ran = 0;
   Double32_t bra1 = 0;
   Double32_t bra = 0;
   if (fNuclearDecayChainIsSet) {

      if (!(protonPDGID == 2212))
         LOG(fatal) << "AtTPC20MgDecayGenerator:PDG code" << protonPDGID << "is not a proton!";
      brp = gRandom->Uniform(0, 1);    // uniform random number for proton
      bra = gRandom->Uniform(0, 1);    // uniform random number for alpha (20Ne decay to 16O)
      brbeta = gRandom->Uniform(0, 1); // uniform random number for beta (20Mg decay)
      brb = gRandom->Uniform(0, 1);    // uniform random number for beta (20Mgdecay to 20Na followed by 19Ne)
      ran = gRandom->Uniform(0, 1);    // uniform random number for beta(20Na decay to 20Ne)
      bra1 = gRandom->Uniform(0, 1);   // uniform random number for alpha (19Ne decay to 15O)

      for (Int_t i = 0; i < fParticlesDefinedInNuclearDecay; i++) {

         if ((0 < brbeta) && (brbeta <= 0.72)) {
            std::cout << "proton branch along with alpha decay to 15O" << std::endl;
            if ((0 < brb) && (brb <= 0.7142)) {
               Double32_t BetaMomentum1 = TMath::Sqrt(pxBeta1 * pxBeta1 + pyBeta1 * pyBeta1 + pzBeta1 * pzBeta1);
               pxBeta1 = pxBeta1 * r1 * 1e-6 * fParticleEnergies[i] / BetaMomentum1 * fParticleEnergies[i];
               pyBeta1 = pyBeta1 * fParticleEnergies[i] * r1 * 1e-6 / BetaMomentum1 * fParticleEnergies[i];
               pzBeta1 = pzBeta1 * r1 * 1e-6 * fParticleEnergies[i] / BetaMomentum1 * fParticleEnergies[i];
            }

            else if ((0.7142 < brb) && (brb <= 0.71523)) {
               Double32_t BetaMomentum2 = TMath::Sqrt(pxBeta2 * pxBeta2 + pyBeta2 * pyBeta2 + pzBeta2 * pzBeta2);
               pxBeta2 = pxBeta2 * r2 * 1e-6 * fParticleEnergies[i] / BetaMomentum2 * fParticleEnergies[i];
               pyBeta2 = pyBeta2 * r2 * 1e-6 * fParticleEnergies[i] / BetaMomentum2 * fParticleEnergies[i];
               pzBeta2 = pzBeta2 * r2 * 1e-6 * fParticleEnergies[i] / BetaMomentum2 * fParticleEnergies[i];
            }

            else if ((0.999733 < brb) && (brb <= 1)) {
               {
                  Double32_t BetaMomentum13 =
                     TMath::Sqrt(pxBeta13 * pxBeta13 + pyBeta13 * pyBeta13 + pzBeta13 * pzBeta13);
                  pxBeta13 = pxBeta13 * r13 * 1e-6 * fParticleEnergies[i] / BetaMomentum13 * fParticleEnergies[i];
                  pyBeta13 = pyBeta13 * r13 * 1e-6 * fParticleEnergies[i] / BetaMomentum13 * fParticleEnergies[i];
                  pzBeta13 = pzBeta13 * r13 * 1e-6 * fParticleEnergies[i] / BetaMomentum13 * fParticleEnergies[i];
               }
               if (brp == 1) {
                  {
                     Double32_t ProtonMomentum =
                        TMath::Sqrt(pxProton * pxProton + pyProton * pyProton + pzProton * pzProton);
                     pxProton = pxProton * fParticleEnergies[i] / ProtonMomentum;
                     pyProton = pyProton * fParticleEnergies[i] / ProtonMomentum;
                     pzProton = pzProton * fParticleEnergies[i] / ProtonMomentum;
                  }

                  if (bra1 == 1) {
                     Double32_t AlphaMomentum = TMath::Sqrt(pxAlpha * pxAlpha + pyAlpha * pyAlpha + pzAlpha * pzAlpha);

                     pxAlpha = pxAlpha * fParticleEnergies[i] / AlphaMomentum;
                     pyAlpha = pyAlpha * fParticleEnergies[i] / AlphaMomentum;
                     pzAlpha = pzAlpha * fParticleEnergies[i] / AlphaMomentum;
                  }
               }
            }

            else if ((0.71523 < brb) && (brb <= 0.83426)) {
               {
                  Double32_t BetaMomentum3 = TMath::Sqrt(pxBeta3 * pxBeta3 + pyBeta3 * pyBeta3 + pzBeta3 * pzBeta3);
                  pxBeta3 = pxBeta3 * r3 * 1e-6 * fParticleEnergies[i] / BetaMomentum3 * fParticleEnergies[i];
                  pyBeta3 = pyBeta3 * r3 * 1e-6 * fParticleEnergies[i] / BetaMomentum3 * fParticleEnergies[i];
                  pzBeta3 = pzBeta3 * r3 * 1e-6 * fParticleEnergies[i] / BetaMomentum3 * fParticleEnergies[i];
               }
               if (brp == 1) {

                  Double32_t ProtonMomentum1 =
                     TMath::Sqrt(pxProton1 * pxProton1 + pyProton1 * pyProton1 + pzProton1 * pzProton1);
                  pxProton1 = pxProton1 * fParticleEnergies[i] / ProtonMomentum1;
                  pyProton1 = pyProton1 * fParticleEnergies[i] / ProtonMomentum1;
                  pzProton1 = pzProton1 * fParticleEnergies[i] / ProtonMomentum1;
               }
            }

            else if ((0.9118 < brb) && (brb <= 0.93146)) {
               {
                  Double32_t BetaMomentum6 = TMath::Sqrt(pxBeta6 * pxBeta6 + pyBeta6 * pyBeta6 + pzBeta6 * pzBeta6);
                  pxBeta6 = pxBeta6 * r6 * 1e-6 * fParticleEnergies[i] / BetaMomentum6 * fParticleEnergies[i];
                  pyBeta6 = pyBeta6 * r6 * 1e-6 * fParticleEnergies[i] / BetaMomentum6 * fParticleEnergies[i];
                  pzBeta6 = pzBeta6 * r6 * 1e-6 * fParticleEnergies[i] / BetaMomentum6 * fParticleEnergies[i];
               }
               if ((0 < brp) && (brp <= 0.4375)) {
                  Double32_t ProtonMomentum2 =
                     TMath::Sqrt(pxProton2 * pxProton2 + pyProton2 * pyProton2 + pzProton2 * pzProton2);
                  pxProton2 = pxProton2 * fParticleEnergies[i] / ProtonMomentum2;
                  pyProton2 = pyProton2 * fParticleEnergies[i] / ProtonMomentum2;
                  pzProton2 = pzProton2 * fParticleEnergies[i] / ProtonMomentum2;
               } else if ((0.4375 < brp) && (brp <= 0.6250)) {
                  Double32_t ProtonMomentum6 =
                     TMath::Sqrt(pxProton6 * pxProton6 + pyProton6 * pyProton6 + pzProton6 * pzProton6);
                  pxProton6 = pxProton6 * fParticleEnergies[i] / ProtonMomentum6;
                  pyProton6 = pyProton6 * fParticleEnergies[i] / ProtonMomentum6;
                  pzProton6 = pzProton6 * fParticleEnergies[i] / ProtonMomentum6;
               } else if ((0.625 < brp) && (brp <= 0.875)) {
                  Double32_t ProtonMomentum7 =
                     TMath::Sqrt(pxProton7 * pxProton7 + pyProton7 * pyProton7 + pzProton7 * pzProton7);
                  pxProton7 = pxProton7 * fParticleEnergies[i] / ProtonMomentum7;
                  pyProton7 = pyProton7 * fParticleEnergies[i] / ProtonMomentum7;
                  pzProton7 = pzProton7 * fParticleEnergies[i] / ProtonMomentum7;
               } else if ((0.875 < brp) && (brp <= 1)) {
                  Double32_t ProtonMomentum8 =
                     TMath::Sqrt(pxProton8 * pxProton8 + pyProton8 * pyProton8 + pzProton8 * pzProton8);
                  pxProton8 = pxProton8 * fParticleEnergies[i] / ProtonMomentum8;
                  pyProton8 = pyProton8 * fParticleEnergies[i] / ProtonMomentum8;
                  pzProton8 = pzProton8 * fParticleEnergies[i] / ProtonMomentum8;
               }
            }

            else if ((0.83426 < brb) && (brb <= 0.88386)) {
               {
                  Double32_t BetaMomentum4 = TMath::Sqrt(pxBeta4 * pxBeta4 + pyBeta4 * pyBeta4 + pzBeta4 * pzBeta4);
                  pxBeta4 = pxBeta4 * r4 * 1e-6 * fParticleEnergies[i] / BetaMomentum4 * fParticleEnergies[i];
                  pyBeta4 = pyBeta4 * r4 * 1e-6 * fParticleEnergies[i] / BetaMomentum4 * fParticleEnergies[i];
                  pzBeta4 = pzBeta4 * r4 * 1e-6 * fParticleEnergies[i] / BetaMomentum4 * fParticleEnergies[i];
               }
               if ((0 < brp) && (brp <= 0.0769)) {
                  Double32_t ProtonMomentum3 =
                     TMath::Sqrt(pxProton3 * pxProton3 + pyProton3 * pyProton3 + pzProton3 * pzProton3);
                  pxProton3 = pxProton3 * fParticleEnergies[i] / ProtonMomentum3;
                  pyProton3 = pyProton3 * fParticleEnergies[i] / ProtonMomentum3;
                  pzProton3 = pzProton3 * fParticleEnergies[i] / ProtonMomentum3;
               }

               else if ((0.0769 < brp) && (brp <= 1)) {
                  Double32_t ProtonMomentum4 =
                     TMath::Sqrt(pxProton4 * pxProton4 + pyProton4 * pyProton4 + pzProton4 * pzProton4);
                  pxProton4 = pxProton4 * fParticleEnergies[i] / ProtonMomentum4;
                  pyProton4 = pyProton4 * fParticleEnergies[i] / ProtonMomentum4;
                  pzProton4 = pzProton4 * fParticleEnergies[i] / ProtonMomentum4;
               }
            }

            else if ((0.88386 < brb) && (brb <= 0.9118)) {
               {
                  Double32_t BetaMomentum5 = TMath::Sqrt(pxBeta5 * pxBeta5 + pyBeta5 * pyBeta5 + pzBeta5 * pzBeta5);
                  pxBeta5 = pxBeta5 * r5 * 1e-6 * fParticleEnergies[i] / BetaMomentum5 * fParticleEnergies[i];
                  pyBeta5 = pyBeta5 * r5 * 1e-6 * fParticleEnergies[i] / BetaMomentum5 * fParticleEnergies[i];
                  pzBeta5 = pzBeta5 * r5 * 1e-6 * fParticleEnergies[i] / BetaMomentum5 * fParticleEnergies[i];
               }
               if (brp == 1) {
                  Double32_t ProtonMomentum5 =
                     TMath::Sqrt(pxProton5 * pxProton5 + pyProton5 * pyProton5 + pzProton5 * pzProton5);
                  pxProton5 = pxProton5 * fParticleEnergies[i] / ProtonMomentum5;
                  pyProton5 = pyProton5 * fParticleEnergies[i] / ProtonMomentum5;
                  pzProton5 = pzProton5 * fParticleEnergies[i] / ProtonMomentum5;
               }
            }

            else if ((0.93146 < brb) && (brb <= 0.94698)) {
               {
                  Double32_t BetaMomentum7 = TMath::Sqrt(pxBeta7 * pxBeta7 + pyBeta7 * pyBeta7 + pzBeta7 * pzBeta7);
                  pxBeta7 = pxBeta7 * r7 * 1e-6 * fParticleEnergies[i] / BetaMomentum7 * fParticleEnergies[i];
                  pyBeta7 = pyBeta7 * r7 * 1e-6 * fParticleEnergies[i] / BetaMomentum7 * fParticleEnergies[i];
                  pzBeta7 = pzBeta7 * r7 * 1e-6 * fParticleEnergies[i] / BetaMomentum7 * fParticleEnergies[i];
               }
               if (brp == 1) {
                  Double32_t ProtonMomentum9 =
                     TMath::Sqrt(pxProton9 * pxProton9 + pyProton9 * pyProton9 + pzProton9 * pzProton9);
                  pxProton9 = pxProton9 * fParticleEnergies[i] / ProtonMomentum9;
                  pyProton9 = pyProton9 * fParticleEnergies[i] / ProtonMomentum9;
                  pzProton9 = pzProton9 * fParticleEnergies[i] / ProtonMomentum9;
               }
            }

            else if ((0.94698 < brb) && (brb <= 0.95277)) {
               {
                  Double32_t BetaMomentum8 = TMath::Sqrt(pxBeta8 * pxBeta8 + pyBeta8 * pyBeta8 + pzBeta8 * pzBeta8);
                  pxBeta8 = pxBeta8 * r8 * 1e-6 * fParticleEnergies[i] / BetaMomentum8 * fParticleEnergies[i];
                  pyBeta8 = pyBeta8 * r8 * 1e-6 * fParticleEnergies[i] / BetaMomentum8 * fParticleEnergies[i];
                  pzBeta8 = pzBeta8 * r8 * 1e-6 * fParticleEnergies[i] / BetaMomentum8 * fParticleEnergies[i];
               }
               if (brp == 1) {
                  Double32_t ProtonMomentum10 =
                     TMath::Sqrt(pxProton10 * pxProton10 + pyProton10 * pyProton10 + pzProton10 * pzProton10);
                  pxProton10 = pxProton10 * fParticleEnergies[i] / ProtonMomentum10;
                  pyProton10 = pyProton10 * fParticleEnergies[i] / ProtonMomentum10;
                  pzProton10 = pzProton10 * fParticleEnergies[i] / ProtonMomentum10;
               }
            }

            else if ((0.95277 < brb) && (brb <= 0.96517)) {
               {
                  Double32_t BetaMomentum9 = TMath::Sqrt(pxBeta9 * pxBeta9 + pyBeta9 * pyBeta9 + pzBeta9 * pzBeta9);
                  pxBeta9 = pxBeta9 * r9 * 1e-6 * fParticleEnergies[i] / BetaMomentum9 * fParticleEnergies[i];
                  pyBeta9 = pyBeta9 * r9 * 1e-6 * fParticleEnergies[i] / BetaMomentum9 * fParticleEnergies[i];
                  pzBeta9 = pzBeta9 * r9 * 1e-6 * fParticleEnergies[i] / BetaMomentum9 * fParticleEnergies[i];
               }
               if (brp == 1) {
                  Double32_t ProtonMomentum11 =
                     TMath::Sqrt(pxProton11 * pxProton11 + pyProton11 * pyProton11 + pzProton11 * pzProton11);
                  pxProton11 = pxProton11 * fParticleEnergies[i] / ProtonMomentum11;
                  pyProton11 = pyProton11 * fParticleEnergies[i] / ProtonMomentum11;
                  pzProton11 = pzProton11 * fParticleEnergies[i] / ProtonMomentum11;
               }
            }

            else if ((0.96517 < brb) && (brb <= 0.99932)) {
               {
                  Double32_t BetaMomentum10 =
                     TMath::Sqrt(pxBeta10 * pxBeta10 + pyBeta10 * pyBeta10 + pzBeta10 * pzBeta10);
                  pxBeta10 = pxBeta10 * r10 * 1e-6 * fParticleEnergies[i] / BetaMomentum10 * fParticleEnergies[i];
                  pyBeta10 = pyBeta10 * r10 * 1e-6 * fParticleEnergies[i] / BetaMomentum10 * fParticleEnergies[i];
                  pzBeta10 = pzBeta10 * r10 * 1e-6 * fParticleEnergies[i] / BetaMomentum10 * fParticleEnergies[i];
               }
               if ((0 < brp) && (brp <= 0.333)) {
                  Double32_t ProtonMomentum12 =
                     TMath::Sqrt(pxProton12 * pxProton12 + pyProton12 * pyProton12 + pzProton12 * pzProton12);
                  pxProton12 = pxProton12 * fParticleEnergies[i] / ProtonMomentum12;
                  pyProton12 = pyProton12 * fParticleEnergies[i] / ProtonMomentum12;
                  pzProton12 = pzProton12 * fParticleEnergies[i] / ProtonMomentum12;
               }

               else if ((0.333 < brp) && (brp <= 1)) {
                  Double32_t ProtonMomentum13 =
                     TMath::Sqrt(pxProton13 * pxProton13 + pyProton13 * pyProton13 + pzProton13 * pzProton13);
                  pxProton13 = pxProton13 * fParticleEnergies[i] / ProtonMomentum13;
                  pyProton13 = pyProton13 * fParticleEnergies[i] / ProtonMomentum13;
                  pzProton13 = pzProton13 * fParticleEnergies[i] / ProtonMomentum13;
               }
            }

            else if ((0.99932 < brb) && (brb <= 0.99963)) {
               Double32_t BetaMomentum11 = TMath::Sqrt(pxBeta11 * pxBeta11 + pyBeta11 * pyBeta11 + pzBeta11 * pzBeta11);
               pxBeta11 = pxBeta11 * r11 * 1e-6 * fParticleEnergies[i] / BetaMomentum11 * fParticleEnergies[i];
               pyBeta11 = pyBeta11 * r11 * 1e-6 * fParticleEnergies[i] / BetaMomentum11 * fParticleEnergies[i];
               pzBeta11 = pzBeta11 * r11 * 1e-6 * fParticleEnergies[i] / BetaMomentum11 * fParticleEnergies[i];
            }

            else if ((0.99963 < brb) && (brb <= 0.999733)) {
               Double32_t BetaMomentum12 = TMath::Sqrt(pxBeta12 * pxBeta12 + pyBeta12 * pyBeta12 + pzBeta12 * pzBeta12);
               pxBeta12 = pxBeta12 * r12 * 1e-6 * fParticleEnergies[i] / BetaMomentum12 * fParticleEnergies[i];
               pyBeta12 = pyBeta12 * r12 * 1e-6 * fParticleEnergies[i] / BetaMomentum12 * fParticleEnergies[i];
               pzBeta12 = pzBeta12 * r12 * 1e-6 * fParticleEnergies[i] / BetaMomentum12 * fParticleEnergies[i];
            }
         } else if ((0.72 < brbeta) && (brbeta <= 1)) {
            // std::cout<<"beta branch starts"<<std::endl;
            if (ran == 1) {
               {
                  Double32_t BetaMomentum14 =
                     TMath::Sqrt(pxBeta14 * pxBeta14 + pyBeta14 * pyBeta14 + pzBeta14 * pzBeta14);
                  pxBeta14 = pxBeta14 * r14 * 1e-6 * fParticleEnergies[i] / BetaMomentum14 * fParticleEnergies[i];
                  pyBeta14 = pyBeta14 * r14 * 1e-6 * fParticleEnergies[i] / BetaMomentum14 * fParticleEnergies[i];
                  pzBeta14 = pzBeta14 * r14 * 1e-6 * fParticleEnergies[i] / BetaMomentum14 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum1 =
                     TMath::Sqrt(pxAlpha1 * pxAlpha1 + pyAlpha1 * pyAlpha1 + pzAlpha1 * pzAlpha1);

                  pxAlpha1 = pxAlpha1 * fParticleEnergies[i] / AlphaMomentum1;
                  pyAlpha1 = pyAlpha1 * fParticleEnergies[i] / AlphaMomentum1;
                  pzAlpha1 = pzAlpha1 * fParticleEnergies[i] / AlphaMomentum1;
               }
            }

            else if ((0.000016 < ran) && (ran <= 0.002646)) {
               {
                  Double32_t BetaMomentum15 =
                     TMath::Sqrt(pxBeta15 * pxBeta15 + pyBeta15 * pyBeta15 + pzBeta15 * pzBeta15);
                  pxBeta15 = pxBeta15 * r15 * 1e-6 * fParticleEnergies[i] / BetaMomentum15 * fParticleEnergies[i];
                  pyBeta15 = pyBeta15 * r15 * 1e-6 * fParticleEnergies[i] / BetaMomentum15 * fParticleEnergies[i];
                  pzBeta15 = pzBeta15 * r15 * 1e-6 * fParticleEnergies[i] / BetaMomentum15 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum2 =
                     TMath::Sqrt(pxAlpha2 * pxAlpha2 + pyAlpha2 * pyAlpha2 + pzAlpha2 * pzAlpha2);

                  pxAlpha2 = pxAlpha2 * fParticleEnergies[i] / AlphaMomentum2;
                  pyAlpha2 = pyAlpha2 * fParticleEnergies[i] / AlphaMomentum2;
                  pzAlpha2 = pzAlpha2 * fParticleEnergies[i] / AlphaMomentum2;
               }
            }

            else if ((0.002646 < ran) && (ran <= 0.004696)) {
               Double32_t BetaMomentum16 = TMath::Sqrt(pxBeta16 * pxBeta16 + pyBeta16 * pyBeta16 + pzBeta16 * pzBeta16);
               pxBeta16 = pxBeta16 * r16 * 1e-6 * fParticleEnergies[i] / BetaMomentum16 * fParticleEnergies[i];
               pyBeta16 = pyBeta16 * r16 * 1e-6 * fParticleEnergies[i] / BetaMomentum16 * fParticleEnergies[i];
               pzBeta16 = pzBeta16 * r16 * 1e-6 * fParticleEnergies[i] / BetaMomentum16 * fParticleEnergies[i];
            }

            else if ((0.004696 < ran) && (ran <= 0.005866)) {
               Double32_t BetaMomentum17 = TMath::Sqrt(pxBeta17 * pxBeta17 + pyBeta17 * pyBeta17 + pzBeta17 * pzBeta17);
               pxBeta17 = pxBeta1 * r17 * 1e-6 * fParticleEnergies[i] / BetaMomentum17 * fParticleEnergies[i];
               pyBeta17 = pyBeta1 * r17 * 1e-6 * fParticleEnergies[i] / BetaMomentum17 * fParticleEnergies[i];
               pzBeta17 = pzBeta1 * r17 * 1e-6 * fParticleEnergies[i] / BetaMomentum17 * fParticleEnergies[i];
            }

            else if ((0.005866 < ran) && (ran <= 0.007606)) {
               {
                  Double32_t BetaMomentum18 =
                     TMath::Sqrt(pxBeta18 * pxBeta18 + pyBeta18 * pyBeta18 + pzBeta18 * pzBeta18);
                  pxBeta18 = pxBeta18 * r18 * 1e-6 * fParticleEnergies[i] / BetaMomentum18 * fParticleEnergies[i];
                  pyBeta18 = pyBeta18 * r18 * 1e-6 * fParticleEnergies[i] / BetaMomentum18 * fParticleEnergies[i];
                  pzBeta18 = pzBeta18 * r18 * 1e-6 * fParticleEnergies[i] / BetaMomentum18 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum3 =
                     TMath::Sqrt(pxAlpha3 * pxAlpha3 + pyAlpha3 * pyAlpha3 + pzAlpha3 * pzAlpha3);

                  pxAlpha3 = pxAlpha3 * fParticleEnergies[i] / AlphaMomentum3;
                  pyAlpha3 = pyAlpha3 * fParticleEnergies[i] / AlphaMomentum3;
                  pzAlpha3 = pzAlpha3 * fParticleEnergies[i] / AlphaMomentum3;
               }
            }

            else if ((0.007606 < ran) && (ran <= 0.008489)) {
               {
                  Double32_t BetaMomentum19 =
                     TMath::Sqrt(pxBeta19 * pxBeta19 + pyBeta19 * pyBeta19 + pzBeta19 * pzBeta19);
                  pxBeta19 = pxBeta19 * r19 * 1e-6 * fParticleEnergies[i] / BetaMomentum19 * fParticleEnergies[i];
                  pyBeta19 = pyBeta19 * r19 * 1e-6 * fParticleEnergies[i] / BetaMomentum19 * fParticleEnergies[i];
                  pzBeta19 = pzBeta19 * r19 * 1e-6 * fParticleEnergies[i] / BetaMomentum19 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum4 =
                     TMath::Sqrt(pxAlpha4 * pxAlpha4 + pyAlpha4 * pyAlpha4 + pzAlpha4 * pzAlpha4);

                  pxAlpha4 = pxAlpha4 * fParticleEnergies[i] / AlphaMomentum4;
                  pyAlpha4 = pyAlpha4 * fParticleEnergies[i] / AlphaMomentum4;
                  pzAlpha4 = pzAlpha4 * fParticleEnergies[i] / AlphaMomentum4;
               }
            }

            else if ((0.008489 < ran) && (ran <= 0.037259)) {
               {
                  Double32_t BetaMomentum20 =
                     TMath::Sqrt(pxBeta20 * pxBeta20 + pyBeta20 * pyBeta20 + pzBeta20 * pzBeta20);
                  pxBeta20 = pxBeta20 * r20 * 1e-6 * fParticleEnergies[i] / BetaMomentum20 * fParticleEnergies[i];
                  pyBeta20 = pyBeta20 * r20 * 1e-6 * fParticleEnergies[i] / BetaMomentum20 * fParticleEnergies[i];
                  pzBeta20 = pzBeta20 * r20 * 1e-6 * fParticleEnergies[i] / BetaMomentum20 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum5 =
                     TMath::Sqrt(pxAlpha5 * pxAlpha5 + pyAlpha5 * pyAlpha5 + pzAlpha5 * pzAlpha5);

                  pxAlpha5 = pxAlpha5 * fParticleEnergies[i] / AlphaMomentum5;
                  pyAlpha5 = pyAlpha5 * fParticleEnergies[i] / AlphaMomentum5;
                  pzAlpha5 = pzAlpha5 * fParticleEnergies[i] / AlphaMomentum5;
               }
            }

            else if ((0.037259 < ran) && (ran <= 0.037539)) {
               Double32_t BetaMomentum21 = TMath::Sqrt(pxBeta21 * pxBeta21 + pyBeta21 * pyBeta21 + pzBeta21 * pzBeta21);
               pxBeta21 = pxBeta21 * r21 * 1e-6 * fParticleEnergies[i] / BetaMomentum21 * fParticleEnergies[i];
               pyBeta21 = pyBeta21 * r21 * 1e-6 * fParticleEnergies[i] / BetaMomentum21 * fParticleEnergies[i];
               pzBeta21 = pzBeta21 * r21 * 1e-6 * fParticleEnergies[i] / BetaMomentum21 * fParticleEnergies[i];
            }

            else if ((0.037539 < ran) && (ran <= 0.039949)) {
               {
                  Double32_t BetaMomentum22 =
                     TMath::Sqrt(pxBeta22 * pxBeta22 + pyBeta22 * pyBeta22 + pzBeta22 * pzBeta22);
                  pxBeta22 = pxBeta22 * r22 * 1e-6 * fParticleEnergies[i] / BetaMomentum22 * fParticleEnergies[i];
                  pyBeta22 = pyBeta22 * r22 * 1e-6 * fParticleEnergies[i] / BetaMomentum22 * fParticleEnergies[i];
                  pzBeta22 = pzBeta22 * r22 * 1e-6 * fParticleEnergies[i] / BetaMomentum22 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum6 =
                     TMath::Sqrt(pxAlpha6 * pxAlpha6 + pyAlpha6 * pyAlpha6 + pzAlpha6 * pzAlpha6);

                  pxAlpha6 = pxAlpha6 * fParticleEnergies[i] / AlphaMomentum6;
                  pyAlpha6 = pyAlpha6 * fParticleEnergies[i] / AlphaMomentum6;
                  pzAlpha6 = pzAlpha6 * fParticleEnergies[i] / AlphaMomentum6;
               }
            }

            else if ((0.039949 < ran) && (ran <= 0.040574)) {
               {
                  Double32_t BetaMomentum23 =
                     TMath::Sqrt(pxBeta23 * pxBeta23 + pyBeta23 * pyBeta23 + pzBeta23 * pzBeta23);
                  pxBeta23 = pxBeta23 * r23 * 1e-6 * fParticleEnergies[i] / BetaMomentum23 * fParticleEnergies[i];
                  pyBeta23 = pyBeta23 * r23 * 1e-6 * fParticleEnergies[i] / BetaMomentum23 * fParticleEnergies[i];
                  pzBeta23 = pzBeta23 * r23 * 1e-6 * fParticleEnergies[i] / BetaMomentum23 * fParticleEnergies[i];
               }

               if (bra == 1) {
                  Double32_t AlphaMomentum7 =
                     TMath::Sqrt(pxAlpha7 * pxAlpha7 + pyAlpha7 * pyAlpha7 + pzAlpha7 * pzAlpha7);

                  pxAlpha7 = pxAlpha7 * fParticleEnergies[i] / AlphaMomentum7;
                  pyAlpha7 = pyAlpha7 * fParticleEnergies[i] / AlphaMomentum7;
                  pzAlpha7 = pzAlpha7 * fParticleEnergies[i] / AlphaMomentum7;
               }
            }

            else if ((0.040574 < ran) && (ran <= 0.046404)) {
               {
                  Double32_t BetaMomentum24 =
                     TMath::Sqrt(pxBeta24 * pxBeta24 + pyBeta24 * pyBeta24 + pzBeta24 * pzBeta24);
                  pxBeta24 = pxBeta24 * r24 * 1e-6 * fParticleEnergies[i] / BetaMomentum24 * fParticleEnergies[i];
                  pyBeta24 = pyBeta24 * r24 * 1e-6 * fParticleEnergies[i] / BetaMomentum24 * fParticleEnergies[i];
                  pzBeta24 = pzBeta24 * r24 * 1e-6 * fParticleEnergies[i] / BetaMomentum24 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum8 =
                     TMath::Sqrt(pxAlpha8 * pxAlpha8 + pyAlpha8 * pyAlpha8 + pzAlpha8 * pzAlpha8);

                  pxAlpha8 = pxAlpha8 * fParticleEnergies[i] / AlphaMomentum8;
                  pyAlpha8 = pyAlpha8 * fParticleEnergies[i] / AlphaMomentum8;
                  pzAlpha8 = pzAlpha8 * fParticleEnergies[i] / AlphaMomentum8;
               }
            }

            else if ((0.046404 < ran) && (ran <= 0.206004)) {
               {
                  Double32_t BetaMomentum25 =
                     TMath::Sqrt(pxBeta25 * pxBeta25 + pyBeta25 * pyBeta25 + pzBeta25 * pzBeta25);
                  pxBeta25 = pxBeta25 * r25 * 1e-6 * fParticleEnergies[i] / BetaMomentum25 * fParticleEnergies[i];
                  pyBeta25 = pyBeta25 * r25 * 1e-6 * fParticleEnergies[i] / BetaMomentum25 * fParticleEnergies[i];
                  pzBeta25 = pzBeta25 * r25 * 1e-6 * fParticleEnergies[i] / BetaMomentum25 * fParticleEnergies[i];
               }
               if (bra == 1) {
                  Double32_t AlphaMomentum9 =
                     TMath::Sqrt(pxAlpha9 * pxAlpha9 + pyAlpha9 * pyAlpha9 + pzAlpha9 * pzAlpha9);

                  pxAlpha9 = pxAlpha9 * fParticleEnergies[i] / AlphaMomentum9;
                  pyAlpha9 = pyAlpha9 * fParticleEnergies[i] / AlphaMomentum9;
                  pzAlpha9 = pzAlpha9 * fParticleEnergies[i] / AlphaMomentum9;
               }
            }

            else if ((0.206004 < ran) && (ran <= 1)) {
               Double32_t BetaMomentum26 = TMath::Sqrt(pxBeta26 * pxBeta26 + pyBeta26 * pyBeta26 + pzBeta26 * pzBeta26);
               pxBeta26 = pxBeta26 * r26 * 1e-6 * fParticleEnergies[i] / BetaMomentum26 * fParticleEnergies[i];
               pyBeta26 = pyBeta26 * r26 * 1e-6 * fParticleEnergies[i] / BetaMomentum26 * fParticleEnergies[i];
               pzBeta26 = pzBeta26 * r26 * 1e-6 * fParticleEnergies[i] / BetaMomentum26 * fParticleEnergies[i];
            }
         }
      }

      primGen->AddTrack(22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // dummy photon for track ID 0
      if ((0 < brb) && (brb <= 0.7142)) {
         primGen->AddTrack(betaPDGID, pxBeta1, pyBeta1, pzBeta1, fX, fY, fZ);
      } else if ((0.7142 < brb) && (brb <= 0.71523)) {
         primGen->AddTrack(betaPDGID, pxBeta2, pyBeta2, pzBeta2, fX, fY, fZ);
      } else if ((0.71523 < brb) && (brb <= 0.83426)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta3, pyBeta3, pzBeta3, fX, fY, fZ);
         }
         if (brp == 1) {
            primGen->AddTrack(protonPDGID, pxProton1, pyProton1, pzProton1, fX, fY, fZ);
         }
      } else if ((0.83426 < brb) && (brb <= 0.88386)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta4, pyBeta4, pzBeta4, fX, fY, fZ);
         }
         if ((0 < brp) && (brp <= 0.0769)) {
            primGen->AddTrack(protonPDGID, pxProton3, pyProton3, pzProton3, fX, fY, fZ);
         } else if ((0.0769 < brp) && (brp <= 1)) {
            primGen->AddTrack(protonPDGID, pxProton4, pyProton4, pzProton4, fX, fY, fZ);
         }
      } else if ((0.88386 < brb) && (brb <= 0.9118)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta5, pyBeta5, pzBeta5, fX, fY, fZ);
         }
         if (brp == 1) {
            primGen->AddTrack(protonPDGID, pxProton5, pyProton5, pzProton5, fX, fY, fZ);
         }
      } else if ((0.9118 < brb) && (brb <= 0.93146)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta6, pyBeta6, pzBeta6, fX, fY, fZ);
         }
         if ((0 < brp) && (brp <= 0.4375)) {
            primGen->AddTrack(protonPDGID, pxProton2, pyProton2, pzProton2, fX, fY, fZ);
         } else if ((0.4375 < brp) && (brp <= 0.6250)) {
            primGen->AddTrack(protonPDGID, pxProton6, pyProton6, pzProton6, fX, fY, fZ);
         } else if ((0.625 < brp) && (brp <= 0.875)) {
            primGen->AddTrack(protonPDGID, pxProton7, pyProton7, pzProton7, fX, fY, fZ);
         } else if ((0.875 < brp) && (brp <= 1)) {
            primGen->AddTrack(protonPDGID, pxProton8, pyProton8, pzProton8, fX, fY, fZ);
         }
      } else if ((0.93146 < brb) && (brb <= 0.94698)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta7, pyBeta7, pzBeta7, fX, fY, fZ);
         }
         if (brp == 1) {
            primGen->AddTrack(protonPDGID, pxProton9, pyProton9, pzProton9, fX, fY, fZ);
         }
      } else if ((0.94698 < brb) && (brb <= 0.95277)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta8, pyBeta8, pzBeta8, fX, fY, fZ);
         }
         if (brp == 1) {
            primGen->AddTrack(protonPDGID, pxProton10, pyProton10, pzProton10, fX, fY, fZ);
         }
      } else if ((0.95277 < brb) && (brb <= 0.96517)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta9, pyBeta9, pzBeta9, fX, fY, fZ);
         }
         if (brp == 1) {
            primGen->AddTrack(protonPDGID, pxProton11, pyProton11, pzProton11, fX, fY, fZ);
         }
      } else if ((0.96517 < brb) && (brb <= 0.99932)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta10, pyBeta10, pzBeta10, fX, fY, fZ);
         }
         if ((0 < brp) && (brp <= 0.333)) {
            primGen->AddTrack(protonPDGID, pxProton12, pyProton12, pzProton12, fX, fY, fZ);
         } else if ((0.333 < brp) && (brp <= 1)) {
            primGen->AddTrack(protonPDGID, pxProton13, pyProton13, pzProton13, fX, fY, fZ);
         }
      } else if ((0.99932 < brb) && (brb <= 0.99963)) {
         primGen->AddTrack(betaPDGID, pxBeta11, pyBeta11, pzBeta11, fX, fY, fZ);
      } else if ((0.99963 < brb) && (brb <= 0.999733)) {
         primGen->AddTrack(betaPDGID, pxBeta12, pyBeta12, pzBeta12, fX, fY, fZ);
      } else if ((0.999733 < brb) && (brb <= 1)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta13, pyBeta13, pzBeta13, fX, fY, fZ);
         }
         if (brp == 1) {
            {
               primGen->AddTrack(protonPDGID, pxProton, pyProton, pzProton, fX, fY, fZ);
            }
            if (bra1 <= 1) {
               primGen->AddTrack(alphaPDGID, pxAlpha, pyAlpha, pzAlpha, fX, fY, fZ);
            }
         }
      }

      if ((0 < ran) && (ran <= 0.000016)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta14, pyBeta14, pzBeta14, fX, fY, fZ);
         }
         if (bra1 == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha1, pyAlpha1, pzAlpha1, fX, fY, fZ);
         }
      } else if ((0.000016 < ran) && (ran <= 0.002646)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta15, pyBeta15, pzBeta15, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha2, pyAlpha2, pzAlpha2, fX, fY, fZ);
         }
      } else if ((0.002646 < ran) && (ran <= 0.004696)) {
         primGen->AddTrack(betaPDGID, pxBeta16, pyBeta16, pzBeta16, fX, fY, fZ);
      } else if ((0.004696 < ran) && (ran <= 0.005866)) {
         primGen->AddTrack(betaPDGID, pxBeta17, pyBeta17, pzBeta17, fX, fY, fZ);
      } else if ((0.005866 < ran) && (ran <= 0.007606)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta18, pyBeta18, pzBeta18, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha3, pyAlpha3, pzAlpha3, fX, fY, fZ);
         }
      } else if ((0.007606 < ran) && (ran <= 0.008489)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta19, pyBeta19, pzBeta19, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha4, pyAlpha4, pzAlpha4, fX, fY, fZ);
         }
      } else if ((0.008489 < ran) && (ran <= 0.037259)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta20, pyBeta20, pzBeta20, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha5, pyAlpha5, pzAlpha5, fX, fY, fZ);
         }
      } else if ((0.037259 < ran) && (ran <= 0.037539)) {
         primGen->AddTrack(betaPDGID, pxBeta21, pyBeta21, pzBeta21, fX, fY, fZ);
      } else if ((0.037539 < ran) && (ran <= 0.039949)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta22, pyBeta22, pzBeta22, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha6, pyAlpha6, pzAlpha6, fX, fY, fZ);
         }
      } else if ((0.039949 < ran) && (ran <= 0.040574)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta23, pyBeta23, pzBeta23, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha7, pyAlpha7, pzAlpha7, fX, fY, fZ);
         }
      } else if ((0.040574 < ran) && (ran <= 0.046404)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta24, pyBeta24, pzBeta24, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha8, pyAlpha8, pzAlpha8, fX, fY, fZ);
         }
      } else if ((0.046404 < ran) && (ran <= 0.206004)) {
         {
            primGen->AddTrack(betaPDGID, pxBeta25, pyBeta25, pzBeta25, fX, fY, fZ);
         }
         if (bra == 1) {
            primGen->AddTrack(alphaPDGID, pxAlpha9, pyAlpha9, pzAlpha9, fX, fY, fZ);
         }
      } else if ((0.206004 < ran) && (ran <= 1)) {
         primGen->AddTrack(betaPDGID, pxBeta26, pyBeta26, pzBeta26, fX, fY, fZ);
      }
   }
   return kTRUE;
}
void AtTPC20MgDecay::SetDecayChainPoint(Double32_t ParticleEnergy, Double32_t ParticleBranchingRatio)
{

   for (Int_t i = 0; i < fParticlesDefinedInNuclearDecay; i++) {
      fParticleEnergies[i] = ParticleEnergy;
      fParticleBranchingRatios[i] = ParticleBranchingRatio;
   }
}

ClassImp(AtTPC20MgDecay)
