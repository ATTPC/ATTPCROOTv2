#ifdef _OPENMP
#include <omp.h>
#endif

#include "MCSrc.hh"
#include "MCMinimization.hh"
#include "MCQMinimization.hh"
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
#include "TStopwatch.h"

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

int target_thread_num = 4;


Int_t main()
{

    gSystem->Load("libATTPCReco.so");
    //omp_set_num_threads(target_thread_num);

    TStopwatch timer;
    timer.Start();

    FairRunAna* run = new FairRunAna(); //Forcing a dummy run

    MCMinimization *min = new MCMinimization();
    min->ResetParameters();

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

    Double_t* parameter = new Double_t[8];

      //#pragma omp parallel for ordered schedule(dynamic,1)
      for(Int_t i=0;i<nEvents;i++){
          //while (Reader1.Next()) {

              Reader1.Next();

              ATEvent* event = (ATEvent*) eventArray->At(0);
              Int_t nHits = event->GetNumHits();
              std::vector<ATHit>* hitArray = event->GetHitArray();
              event->GetHitArrayObj();
              std::cout<<event->GetEventID()<<std::endl;
              hitArray->size();

              std::vector<ATHit*>* hitbuff = new std::vector<ATHit*>;


                    for(Int_t iHit=0; iHit<nHits; iHit++){
                      ATHit hit = event->GetHit(iHit);
                      TVector3 hitPos = hit.GetPosition();
                      hitbuff->push_back(&hit);


                    }
              //std::cout<<hitbuff->size()<<std::endl;

              ATHoughSpaceCircle* fHoughSpaceCircle  = dynamic_cast<ATHoughSpaceCircle*> (houghArray->At(0));
              //if(!fHoughSpaceCircle) std::cout<<" Warning : Failed casting "<<std::endl;
              //std::cout<<fHoughSpaceCircle->GetYCenter()<<std::endl;
              parameter = fHoughSpaceCircle->GetInitialParameters();
              std::pair<Double_t,Double_t>  fHoughLinePar =  fHoughSpaceCircle->GetHoughPar();
              Double_t HoughAngleDeg = fHoughLinePar.first*180.0/TMath::Pi();
              if (   HoughAngleDeg<90.0 && HoughAngleDeg>45.0 ) {

                  min->MinimizeOpt(parameter,event);
              }

          }

  //#pragma omp parallel for ordered schedule(dynamic,1)
  //for(Int_t i=0;i<100000;i++)std::cout<<" Hello ATTPCer! "<<std::endl;

  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  std::cout << std::endl << std::endl;
  std::cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << std::endl;
  std::cout << std::endl;
  return 0;

}
