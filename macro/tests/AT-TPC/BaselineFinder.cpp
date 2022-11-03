#ifndef __CLING__

#include "AtEvent.h"
#include "AtPSADeconv.h"
#include "AtPadArray.h"
#include "AtPadFFT.h"
#include "AtPadReference.h"
#include "AtPadValue.h"
#include "AtPatternEvent.h"
#include "AtRawEvent.h"
#include "AtTpcMap.h"

#include <RQ_OBJECT.h>
#include <TCanvas.h>
#include <TChain.h>
#include <TCutG.h>
#include <TF1.h>
#include <TFile.h>
#include <TGButton.h>
#include <TGClient.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>
#include <TGTab.h>
#include <TGraph.h>
#include <TGraph2D.h>
#include <TGraph2DErrors.h>
#include <TH1.h>
#include <TRandom.h>
#include <TRootEmbeddedCanvas.h>
#include <TString.h>
#include <TSystem.h>
#include <TTreeReader.h>

#include <fstream>
#include <string>
#include <vector>
#endif

using namespace std;

void BaselineFinder()
{
   cout << "starting macro" << endl;
   TChain *runChain = new TChain("cbmsim");

   string chainFileName = "./data/pulser/run_0043Sub.root";
   cout << "Opening file " << chainFileName << endl;
   runChain->Add(chainFileName.c_str());

   TTreeReader *reader = new TTreeReader(runChain);
   TTreeReaderValue<TClonesArray> *rawEventReader = new TTreeReaderValue<TClonesArray>(*reader, "AtRawEvent");

   AtTpcMap *tpcMap = new AtTpcMap();
   tpcMap->ParseXMLMap(TString(gSystem->Getenv("VMCWORKDIR")) + "/scripts/e12014_pad_map_size.xml");
   tpcMap->AddAuxPad({10, 0, 0, 0}, "MCP_US");
   tpcMap->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
   tpcMap->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
   tpcMap->AddAuxPad({10, 0, 2, 34}, "IC");
   tpcMap->GeneratePadPlane();

   int counts[10] = {};

   double Cin = 100e-15;
   double Vin = 1;

   AtPadReference padRef;

   AtRawEvent *background = new AtRawEvent();
   for (int cobo = 0; cobo < 10; cobo++) {
      padRef.cobo = cobo;
      for (int asad = 0; asad < 4; asad++) {
         padRef.asad = asad;
         for (int aget = 0; aget < 4; aget++) {
            padRef.aget = aget;
            for (int ch = 0; ch < 68; ch++) {
               padRef.ch = ch;
               if (ch != 11 && ch != 22 && ch != 45 && ch != 56) {
                  int padNum = tpcMap->GetPadNum(padRef);
                  AtPad *pad = new AtPad(padNum);
                  background->AddPad(*pad);
               } else {
                  // cout << "adding FPN " << cobo << " " << asad << " " << aget << " " << ch << endl;
                  background->AddFPN(padRef);
               }
            }
         }
      }
   }

   int events = runChain->GetEntries();

   cout << "Processing " << events << " events." << endl;

   for (int event = 0; event < events; event++) {
      // cout << "processing event number " << event << endl;

      reader->SetEntry(event);
      AtRawEvent *rawEventPtr = dynamic_cast<AtRawEvent *>((*rawEventReader)->At(0));

      for (int cobo = 0; cobo < 10; cobo++) {
         padRef.cobo = cobo;
         for (int asad = 0; asad < 4; asad++) {
            padRef.asad = asad;
            for (int aget = 0; aget < 4; aget++) {
               padRef.aget = aget;
               for (int ch = 0; ch < 68; ch++) {
                  padRef.ch = ch;
                  AtPad *pad;
                  AtPad *acPad;
                  if (ch != 11 && ch != 22 && ch != 45 && ch != 56) {
                     // cout << "processing " << cobo << " " << asad << " " << aget << " " << ch << endl;
                     pad = rawEventPtr->GetPad(tpcMap->GetPadNum(padRef));
                     acPad = background->GetPad(tpcMap->GetPadNum(padRef));
                  } else {
                     // cout << "using FPN " << cobo << " " << asad << " " << aget << " " << ch << endl;

                     pad = rawEventPtr->GetFpn(padRef);
                     acPad = background->GetFpn(padRef);
                  }
                  // cout << "pad " << cobo * 1000000 + asad * 10000 + aget * 100 + ch << endl;
                  // cout << "pad num " << tpcMap->GetPadNum(padRef) << endl;
                  if (pad == nullptr) {
                     // cout << "null pointer at " << cobo * 1000000 + asad * 10000 + aget * 100 + ch << endl;
                  } else {
                     for (int i = 0; i < 512; i++) {
                        double curVal = pad->GetRawADC(i);
                        auto prevVal = acPad->GetADC(i);
                        acPad->SetADC(i, curVal + prevVal);
                     }
                  }
               }
            }
         }
         counts[cobo]++;
      }
   }
   cout << "making pads" << endl;
   for (int cobo = 0; cobo < 10; cobo++) {
      padRef.cobo = cobo;
      for (int asad = 0; asad < 4; asad++) {
         padRef.asad = asad;
         for (int aget = 0; aget < 4; aget++) {
            padRef.aget = aget;
            for (int ch = 0; ch < 68; ch++) {
               padRef.ch = ch;
               AtPad *pad;
               if (ch != 11 && ch != 22 && ch != 45 && ch != 56) {
                  // cout << "making pad" << cobo * 1000000 + asad * 10000 + aget * 100 + ch << endl;
                  // cout << "making pad " << tpcMap->GetPadNum(padRef) << endl;
                  // cout << "pad made" << endl;
                  pad = background->GetPad(tpcMap->GetPadNum(padRef));
               } else {
                  pad = background->GetFpn(padRef);
               }
               for (int i = 0; i < 512; i++) {
                  pad->SetADC(i, pad->GetADC(i) / events);
               }
               // cout << "adding pad" << cobo * 1000000 + asad * 10000 + aget * 100 + ch << "  " << pad->GetPadNum()
               // << endl; cout << "pad added" << endl;
            }
         }
      }
   }
   TH1D *hist = new TH1D("hist", "hist", 512, 0, 512);
   auto pad = background->GetPad(2817);
   for (int i = 0; i < 512; i++) {
      hist->SetBinContent(i + 1, pad->GetADC(i));
   }
   hist->Draw();
   background->SetName("baseline");
   cout << "making root file" << endl;
   TFile *backFile = new TFile("./data/baseline.root", "recreate");
   cout << "writing file" << endl;
   background->Write();
   cout << "closing file" << endl;
   backFile->Close();
}
