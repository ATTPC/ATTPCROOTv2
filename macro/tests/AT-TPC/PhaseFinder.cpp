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

void PhaseFinder()
{
   cout << "starting macro" << endl;
   TChain *runChain = new TChain("cbmsim");

   string chainFileName = "./data/pulser/run_0043Sub.root";
   cout << "Opening file " << chainFileName << endl;
   runChain->Add(chainFileName.c_str());

   AtRawEvent *rawEventBkg;

   TFile *f1 = new TFile("./data/baseline.root");
   f1->GetObject("baseline", rawEventBkg);
   f1->Close();

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

   AtRawEvent *phase = new AtRawEvent();
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
                  phase->AddPad(*pad);
               } else {
                  phase->AddFPN(padRef);
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
                  AtPad *bkPad;
                  AtPad *acPad;
                  if (ch != 11 && ch != 22 && ch != 45 && ch != 56) {
                     pad = rawEventPtr->GetPad(tpcMap->GetPadNum(padRef));
                     bkPad = rawEventBkg->GetPad(tpcMap->GetPadNum(padRef));
                     acPad = phase->GetPad(tpcMap->GetPadNum(padRef));
                  } else {
                     pad = rawEventPtr->GetFpn(padRef);
                     bkPad = rawEventBkg->GetFpn(padRef);
                     acPad = phase->GetFpn(padRef);
                  }
                  // cout << "pad " << cobo * 1000000 + asad * 10000 + aget * 100 + ch << endl;
                  // cout << "pad num " << tpcMap->GetPadNum(padRef) << endl;
                  if (pad == nullptr) {
                     // cout << "null pointer at " << cobo * 1000000 + asad * 10000 + aget * 100 + ch << endl;
                  } else {
                     int lastCell = (dynamic_cast<AtPadValue *>(pad->GetAugment("lastCell")))->GetValue();
                     int zero = 512 - lastCell;
                     for (int i = 0; i < 512; i++) {
                        int shift = zero + i;
                        if (shift > 511) {
                           shift = shift - 512;
                        }
                        double curVal = 0;
                        if (i == lastCell || i == (lastCell + 1) % 512) {
                           curVal =
                              (pad->GetRawADC(1) - bkPad->GetADC(1) + pad->GetRawADC(510) - bkPad->GetADC(510)) / 2;
                        } else {
                           curVal = pad->GetRawADC(shift) - bkPad->GetADC(shift);
                        }
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
                  pad = phase->GetPad(tpcMap->GetPadNum(padRef));
               } else {
                  pad = phase->GetFpn(padRef);
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
   auto pad = phase->GetPad(2817);
   for (int i = 0; i < 512; i++) {
      hist->SetBinContent(i + 1, pad->GetADC(i));
   }
   hist->Draw();
   phase->SetName("phase");
   cout << "making root file" << endl;
   TFile *phaseFile = new TFile("./data/phase.root", "recreate");
   cout << "writing file" << endl;
   phase->Write();
   cout << "closing file" << endl;
   phaseFile->Close();
}
