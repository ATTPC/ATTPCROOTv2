#include "MCSrc.hh"
#include <ios>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <vector>

#include "TClonesArray.h"
#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TTreeReader.h"
#include "TTreePlayer.h"
#include "TTreeReaderValue.h"
#include "TSystem.h"
#include "TH1F.h"
#include "TCanvas.h"

#include "ATEvent.hh"
#include "ATPad.hh"
#include "ATHit.hh"
#include "ATHoughSpace.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpaceCircle.hh"

#include "FairRootManager.h"
#include "FairLogger.h"
#include "FairRun.h"
#include "FairRunAna.h"

Int_t main()
{

    gSystem->Load("libATTPCReco.so");

    FairRunAna* run = new FairRunAna(); //Forcing a dummy run

    TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "output";
    TString FilePath = workdir + "/macro/Unpack_GETDecoder2/";
    TString FileNameTail = ".root";
    TString FileName     = FilePath + FileNameHead + FileNameTail;

    std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    TTreeReaderValue<TClonesArray> houghArray(Reader1, "ATHough");

          while (Reader1.Next()) {


              ATEvent* event = (ATEvent*) eventArray->At(0);
              Int_t nHits = event->GetNumHits();
              std::vector<ATHit>* hitArray = event->GetHitArray(); //Not working!
              //hitArray->size();

              std::vector<ATHit*>* hitbuff = new std::vector<ATHit*>; // Working!

              //std::vector<ATEvent*> test;
              //test.push_back(event);

                    for(Int_t iHit=0; iHit<nHits; iHit++){
                      ATHit hit = event->GetHit(iHit);
                      TVector3 hitPos = hit.GetPosition();
                      hitbuff->push_back(&hit);


                    }

              //std::cout<<hitbuff->size()<<std::endl;

              ATHoughSpace* fHoughSpaceCircle  = dynamic_cast<ATHoughSpaceCircle*> (houghArray->At(0));
              //if(!fHoughSpaceCircle) std::cout<<" Warning : Failed casting "<<std::endl;
              fHoughSpaceCircle->GetXCenter();


          }

  //#pragma omp parallel for ordered schedule(dynamic,1)
  //for(Int_t i=0;i<100;i++)std::cout<<" Hello ATTPCer! "<<std::endl;
   return 0;

}
