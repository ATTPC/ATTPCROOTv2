#include "ATTrackingAnalysis.hh"
#include "TMath.h"
#ifdef _OPENMP
#include <omp.h>
#endif
#include <memory>

ClassImp(ATTrackingAnalysis)

ATTrackingAnalysis::ATTrackingAnalysis()
{

}

ATTrackingAnalysis::~ATTrackingAnalysis()
{
}

void ATTrackingAnalysis::SetElossParameters(std::vector<Double_t> parE[10])                             {for(Int_t i=0;i<10;i++) fElossPar[i] = parE[i];}
void ATTrackingAnalysis::AddParticle(std::vector<std::pair<Int_t,Int_t>> ptcl)                          {fParticleAZ=ptcl;}


void ATTrackingAnalysis::Analyze(ATRANSACN::ATRansac *Ransac,ATTrackingEventAna *trackingEventAna)
{

  // Analysis function for a RANSAC event. An iterative loop over the tracks found by RANSAC is created.
  // A new minimization pointer has to be created for each independent track.
  // The algorithm will try to fit the track according to the possible particles added by the user from the macro/database.
  // For the moment the ELoss function is taken from this class but eventually the user can define their own parametrization.


  //TODO:: Only one function is considered for the moment

  if(fElossPar[0].size()>0 && fParticleAZ.size()>0){



                ATMCQMinimization *min = new ATMCQMinimization();
                min->ResetParameters();

                // Adding 1 function
                std::function<Double_t(Double_t,std::vector<Double_t>&)> ELossFunc = std::bind(GetEloss,std::placeholders::_1,std::placeholders::_2);
                min->AddELossFunc(ELossFunc);
                // Adding n-vectors of 11 parameters
                min->AddELossPar(fElossPar);
                // Adding n-particles
                min->AddParticle(fParticleAZ);


                delete min;


  }else{
    std::cout<<cYELLOW<<" ATTrackingAnalysis::Analyze - Warning! No Energy Loss or Particle parameters found! "<<cNORMAL<<std::endl;
  }


}

// Default ELoss function for protons in 20 torr Isobutane
Double_t ATTrackingAnalysis::GetEloss(Double_t c0,std::vector<Double_t>& par)
{
   if(par.size()==11){
   //return par[0]*(1./TMath::Power(c0,par[1]))*(1./(par[2]+par[3]/TMath::Power(c0,par[4])))+par[5]*TMath::Exp(-par[6]*TMath::Power((c0-par[7]),2));
   return par[0]*(1./TMath::Power(c0,par[1]))*(1./(par[2]+par[3]/TMath::Power(c0,par[4]))) + par[5]*TMath::Exp(par[6]*TMath::Power((c0+par[7]),2))
          + par[8]*TMath::Exp(par[9]*TMath::Power((c0+par[10]),2));
 }else{
   std::cerr<<cRED<<" ATHoughSpaceCircle::GetEloss -  Warning ! Wrong number of parameters."<<std::endl;
   return 0;
 }
}
