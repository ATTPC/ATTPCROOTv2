#ifndef ATTRACKINGANALYSIS_H
#define ATTRACKINGANALYSIS_H

#include "ATAnalysis.hh"
#include "ATRansac.hh"
#include "ATTrackingEventAna.hh"

#include "ATMCQMinimization.hh"

class ATTrackingAnalysis : public ATAnalysis{

      public:
	       ATTrackingAnalysis();
        ~ATTrackingAnalysis();

        void Analyze(ATRANSACN::ATRansac *Ransac,ATTrackingEventAna *trackingEventAna,TH2Poly* hPadPlane,const multiarray& PadCoord); // Analysis after RANSAC
        void SetElossParameters(std::vector<Double_t> (&parE)[10]);
        void SetEtoRParameters(std::vector<Double_t> (&parRtoE)[10]);
        void AddParticle(std::vector<std::pair<Int_t,Int_t>>& ptcl);


      private:

          static Double_t GetEloss(Double_t c0,std::vector<Double_t>& par);
          static Double_t GetEnergyFromRange(Double_t range,std::vector<Double_t>& par);
          std::vector<Double_t> fElossPar[10];
          std::vector<Double_t> fEtoRPar[10];
          std::vector<std::pair<Int_t,Int_t>> fParticleAZ;



          friend inline std::ostream& operator<<(std::ostream& os,const ATTrackingAnalysis& trackAna)
          {

              if(trackAna.fParticleAZ.size()>0){

                    for(Int_t i=0;i<trackAna.fParticleAZ.size();i++)
                        os<<" Particle "<<i<<" A : "<<trackAna.fParticleAZ.at(i).first<<"  Z : "<<trackAna.fParticleAZ.at(i).first<<std::endl;

                    return os;


              }else return os<<" No particles found !"<<std::endl;

              return os;

          }

          ClassDef(ATTrackingAnalysis, 1);


};

#endif
