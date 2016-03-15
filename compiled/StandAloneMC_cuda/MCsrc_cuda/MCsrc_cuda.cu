#include "MCsrc_cuda.hh"
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


__global__ void test_GPU_function(int* a, int* b, Int_t N){
  int th = blockIdx.x*blockDim.x+threadIdx.x;
  if(th < N){
    b[th] = 0;
    for(int i=0;i<N;i++)
      b[th] += a[i]; 
  }
}


Int_t main()
{

    gSystem->Load("libATTPCReco.so");
    
    FairRunAna* run = new FairRunAna(); //Forcing a dummy run

    TString workdir = getenv("VMCWORKDIR");
  
    /***********************************************/
    /* Test GPU code                               */
    Int_t N = 100000;
    size_t size = N*sizeof(int);

    int* h_a = (int*)malloc(size);
    for(int i=0; i<N; i++){
      h_a[i] = 1;
    }
    int* h_b = (int*)malloc(size);
   
    int* d_a;
    cudaMalloc(&d_a,size);
    cudaMemcpy(d_a,h_a,size,cudaMemcpyHostToDevice);

    int* d_b;
    cudaMalloc(&d_b,size);

    int threadsPerBlock = 256;
    int numBlocks = (N + threadsPerBlock -1) / threadsPerBlock;
    test_GPU_function<<<numBlocks,threadsPerBlock>>>(d_a,d_b,N);

    cudaMemcpy(h_b,d_b,size,cudaMemcpyDeviceToHost);
  
    //std::vector<int> h_b_vec = h_b;
    for(int i=0;i<N;i++){
      std::cout << "h_b[" << i << "] = " << h_b[i]  << std::endl;
    }
    cudaFree(d_a);
    cudaFree(d_b);
    /**********************************************/


    /*TString FileNameHead = "output";
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
              event->GetHitArrayObj();
              //std::cout<<event->GetHitPadMult(0)<<std::endl;
              //std::cout<<event->GetEventID()<<std::endl;
              hitArray->size();

              std::vector<ATHit*>* hitbuff = new std::vector<ATHit*>; // Working!

              //std::vector<ATEvent*> test;
              //test.push_back(event);

                    for(Int_t iHit=0; iHit<nHits; iHit++){
                      ATHit hit = event->GetHit(iHit);
                      TVector3 hitPos = hit.GetPosition();
                      hitbuff->push_back(&hit);


                    }

              //std::cout<<hitbuff->size()<<std::endl;

              ATHoughSpaceCircle* fHoughSpaceCircle  = dynamic_cast<ATHoughSpaceCircle*> (houghArray->At(0));
              //if(!fHoughSpaceCircle) std::cout<<" Warning : Failed casting "<<std::endl;
              std::cout<<fHoughSpaceCircle->GetYCenter()<<std::endl;


          }

  //#pragma omp parallel for ordered schedule(dynamic,1)
  //for(Int_t i=0;i<100;i++)std::cout<<" Hello ATTPCer! "<<std::endl;

  */
   return 0;

}
