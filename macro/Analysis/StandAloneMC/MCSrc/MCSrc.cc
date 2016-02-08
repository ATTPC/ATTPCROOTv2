#include "MCSrc.hh"
#include <ios>
#include <iostream>
#include <istream>
#include <limits>

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
#include "ATHoughSpace.hh"
#include "ATHoughSpaceLine.hh"

#include "FairRootManager.h"
#include "FairLogger.h"
#include "FairRun.h"
#include "FairRunAna.h"

Int_t main()
{

    gSystem->Load("libCling.so");
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
              ATHoughSpaceLine* fHoughSpaceLine_buff  = dynamic_cast<ATHoughSpaceLine*> (houghArray->At(0));


          }

  //#pragma omp parallel for ordered schedule(dynamic,1)
  //for(Int_t i=0;i<100;i++)std::cout<<" Hello ATTPCer! "<<std::endl;
   return 0;

}
