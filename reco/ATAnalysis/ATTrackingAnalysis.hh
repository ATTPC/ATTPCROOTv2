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

        void Analyze(ATRANSACN::ATRansac *Ransac,ATTrackingEventAna *trackingEventAna); // Analysis after RANSAC
        void SetElossParameters(std::vector<Double_t> parE[10]);
        void AddParticle(std::vector<std::pair<Int_t,Int_t>> ptcl);


      private:

          static Double_t GetEloss(Double_t c0,std::vector<Double_t>& par);
          std::vector<Double_t> fElossPar[10];
          std::vector<std::pair<Int_t,Int_t>> fParticleAZ;

          ClassDef(ATTrackingAnalysis, 1);


};

#endif
