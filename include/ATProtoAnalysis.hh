#ifndef ATPROTOANALYSIS_H
#define ATPROTOANALYSIS_H

#include "ATAnalysis.hh"
#include "TF1.h"
#include "TGraph.h"

class ATProtoAnalysis : public ATAnalysis{

      public:
	       ATProtoAnalysis();
        ~ATProtoAnalysis();

        void Analyze(ATProtoEvent* protoevent,ATHoughSpaceLine* houghspace,TF1 *(&HoughFit)[4],TGraph *(&HitPatternFilter)[4],TF1 *(&FitResult)[4]);
        void SetHoughDist(Double_t value);
        Double_t fHoughDist;

        ClassDef(ATProtoAnalysis, 1);
      private:

        std::vector<Double_t> fPar0_fit;
        std::vector<Double_t> fPar1_fit;
        std::vector<Double_t> fAngle;
        std::vector<Double_t> fAngle_fit;
        std::vector<std::pair<Double_t,Double_t>> fHoughPar;

};

#endif
