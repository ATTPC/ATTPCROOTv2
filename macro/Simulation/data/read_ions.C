//////////////////////////////////////////////////////////////////////
//   Macro that generates a list of ions from the ABLA root file    //
//   It can be limited to 1023 entries for visualization            //
//   purposes (problem with TGeoManager)                            //
//                                                                  //
//   Author: Y. Ayyad ayyadlim@nscl.msu.edu                         //
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include "TFile.h"
#include <fstream>
#include <map>
#include <sstream>
#include <string>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"



void read_ions(TString simfile ="240Cf.root"){

  ofstream ionlist;
  ionlist.open ("ionlist.txt");

  std::map<Int_t,std::pair<Int_t,Int_t>> ionsmap;




  Int_t Evnt;
  Int_t event;
  Int_t Aout[100],Zout[100],Ntrack;
  Float_t fOutPx[100],fOutPy[100],fOutPz[100];

  Int_t    ia1      = 0;
  Int_t    iz1      = 0;
  Int_t    key      = 0;

  TString dir = getenv("VMCWORKDIR");
  TString simfilepath = dir + "/macro/Simulation/data/"+ simfile;
  TFile *f = new TFile(simfilepath.Data());
  if(f->IsZombie()){
  std::cout<<cRED<<" ATTPCFissionGenerator: No simulation file found! Check VMCWORKDIR variable. Exiting... "<<cNORMAL<<std::endl;
  delete f;
  }else std::cout<<cGREEN<<" ATTPCFissionGenerator : Prototype geometry found in : "<<simfilepath.Data()<<cNORMAL<<std::endl;

  TTree* fTree = (TTree*) f-> Get("tree101");
  Int_t nEvents = fTree -> GetEntriesFast();
  std::cout<<" Number of events : "<<nEvents<<std::endl;
  fTree->SetBranchAddress("Evnt",&Evnt);
  fTree->SetBranchAddress("Ntrack",&Ntrack);
  fTree->SetBranchAddress("Aout",Aout);
  fTree->SetBranchAddress("Zout",Zout);

  for(Int_t i=0;i<nEvents;i++){

        fTree->GetEntry(i);


        for(Int_t j=0;j<Ntrack;j++){

            ia1=Aout[j];
            iz1=Zout[j];

            if(ia1>2 && iz1>2){

              std::stringstream oss;
              oss << ia1 << iz1;
              istringstream iss(oss.str());
              Int_t c;
              iss>>c;
              key =(Int_t)c;
              std::pair<std::map<Int_t,std::pair<Int_t,Int_t>>::iterator, Bool_t> ionmapiter = ionsmap.insert(  std::pair<Int_t,std::pair<Int_t,Int_t>>(key,std::make_pair(iz1,ia1))     );

              if (  !ionmapiter.second ) {
                  //std::cout << " Repeated key " <<std::endl;//  key << " already exists "<<std::endl;
                    //       << " with value " << (ionmapiter.first)->second << endl;
              } else {
                  //std::cout << "created key "<<std::endl;// << key << " with value " << value << endl;
                  ionlist<<iz1<<" "<<ia1<<std::endl;
                  //std::cout<<ia1<<" "<<iz1<<std::endl;
              }




            }

        }

        //std::cout<<cRED<<" Fission event : "<<i<<cNORMAL<<std::endl;

   }


}
