#include "SAMain.hh"

int main(int argc, char* argv[])
{

   gSystem->Load("libATTPCReco.so");
    
   TStopwatch timer;
   timer.Start();

   FairRunAna* run = new FairRunAna(); //Forcing a dummy run
   TString FileName = "attpcsim.root";
   std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
   TFile* file = new TFile(FileName.Data(),"READ");

   TTree* tree = (TTree*) file -> Get("cbmsim");
   Int_t nEvents = tree -> GetEntries();
   std::cout<<" Number of events : "<<nEvents<<std::endl;
   
  
   return 0;

}
