#ifndef AtPROTOANALYSIS_H
#define AtPROTOANALYSIS_H

#include "AtAnalysis.h"
#include "TF1.h"
#include "TGraph.h"

class AtProtoAnalysis : public AtAnalysis {

public:
   AtProtoAnalysis();
   ~AtProtoAnalysis();

   void Analyze(AtProtoEvent *protoevent, AtProtoEventAna *protoeventAna, AtHoughSpaceLine *houghspace,
                TF1 *(&HoughFit)[4], TGraph *(&HitPatternFilter)[4], TF1 *(&FitResult)[4]);
   void SetHoughDist(Double_t value);
   void SetUpperLimit(Double_t value);
   void SetLowerLimit(Double_t value);
   /*std::vector<Double_t>* GetAngleFit()                     {return &fAngle_fit;}
   std::vector<Double_t>* GetAngle()                        {return &fAngle;}
   std::vector<Double_t>* GetPar0()                         {return &fPar0_fit;}
   std::vector<Double_t>* GetPar1()                         {return &fPar1_fit;}
   std::vector<Double_t>* GetRange()                        {return &fRange;}
   std::vector<std::pair<Double_t,Double_t>>* GetHoughPar() {return &fHoughPar;}*/

   Double_t fHoughDist;
   Double_t fVertexDiff;
   Double_t fUpperLimit;
   Double_t fLowerLimit;

private:
   ClassDef(AtProtoAnalysis, 2);
};

#endif
