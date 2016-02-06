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
        std::vector<Double_t>* GetAngleFit()               {return &fAngle_fit;}
        std::vector<Double_t>* GetAngle()                  {return &fAngle;}
        std::vector<Double_t>* GetPar0()                   {return &fPar0_fit;}
        std::vector<Double_t>* GetPar1()                   {return &fPar1_fit;}
        Double_t fHoughDist;


      private:

        std::vector<Double_t> fPar0_fit;
        std::vector<Double_t> fPar1_fit;
        std::vector<Double_t> fAngle;
        std::vector<Double_t> fAngle_fit;
        std::vector<std::pair<Double_t,Double_t>> fHoughPar;

          ClassDef(ATProtoAnalysis, 1);


};

#endif
