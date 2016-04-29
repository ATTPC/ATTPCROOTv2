#include "ATProtoAnalysis.hh"
#include "TMath.h"
#ifdef _OPENMP
#include <omp.h>
#endif
#include <memory>


ClassImp(ATProtoAnalysis)

ATProtoAnalysis::ATProtoAnalysis()
{
      gErrorIgnoreLevel=kFatal;
      fHoughDist=2.0;

}

ATProtoAnalysis::~ATProtoAnalysis()
{
}

void ATProtoAnalysis::SetHoughDist(Double_t value)   { fHoughDist=value; }

void ATProtoAnalysis::Analyze(ATProtoEvent* protoevent,ATProtoEventAna* protoeventAna,ATHoughSpaceLine* houghspace,TF1 *(&HoughFit)[4],TGraph *(&HitPatternFilter)[4],TF1 *(&FitResult)[4])
{


    std::vector<std::vector<std::pair<Double_t,Double_t>>> QELossHitPattern;


    fHoughPar = houghspace->GetHoughPar();

     for(Int_t i=0;i<fHoughPar.size();i++){

             fAngle.push_back(180-fHoughPar.at(i).first*180/TMath::Pi());
             HoughFit[i]->SetParameter(0,fHoughPar.at(i).first);
             HoughFit[i]->SetParameter(1,fHoughPar.at(i).second);

             /*std::cout<<" ------ Hough Parameters for Quadrant : "<<i<<std::endl;
             std::cout<<" Angle HIST : "<<180-fHoughPar.at(i).first*180/TMath::Pi()<<std::endl;
             std::cout<<" Distance HIST : "<<fHoughPar.at(i).second<<std::endl;*/
      }// HoughPar Loop


     for (Int_t i=0;i<4;i++){

        std::vector<std::pair<Double_t,Double_t>> ELossHitPattern;
        ATProtoQuadrant* quadrant;
        std::vector<Int_t> qNumHits;
   		  std::vector<Double_t> qRad;
        quadrant = &protoevent->GetQuadrantArray()->at(i);
        qNumHits.push_back(quadrant->GetNumHits());
        Int_t qNumHit = quadrant->GetNumHits();
        Double_t *rad_graph = new Double_t[qNumHit];
        Double_t *posz_graph = new Double_t[qNumHit];


        Double_t rad_max=0.0;
        Double_t charge_max=0.0;
        Double_t range=0.0;

            for(Int_t j=0;j<qNumHit;j++){
              ATHit* qhit = quadrant->GetHit(j);
              TVector3 position = qhit->GetPosition();
              Double_t charge = qhit->GetCharge();
              Double_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2) );
              Double_t distance = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2) + TMath::Power(position.Z(),2));
              if((qNumHit-j<5)&&(charge>charge_max)){
                charge_max=charge;
                range = distance;
              }
              if(radius>rad_max) rad_max=radius;
              rad_graph[j] = radius;
              posz_graph[j] = position.Z();

                Double_t geo_dist = TMath::Abs (TMath::Cos(fHoughPar.at(i).first)*radius  + TMath::Sin(fHoughPar.at(i).first)*position.Z()  - fHoughPar.at(i).second);
                if(geo_dist<fHoughDist) HitPatternFilter[i]->SetPoint(HitPatternFilter[i]->GetN(),radius,position.Z());

                    if(j<qNumHit-1){
                        ATHit* qhit_forw        = quadrant ->GetHit(j+1);
                        TVector3 position_forw  = qhit_forw->GetPosition();
                        Double_t radius_forw    = TMath::Sqrt( TMath::Power(position_forw.X(),2) + TMath::Power(position_forw.Y(),2) );
                        Double_t charge_forw    = qhit_forw->GetCharge();
                        std::pair<Double_t,Double_t> ELossPair;
                        ELossPair.first  = (charge + charge_forw)/2.0;
                        ELossPair.second = (radius + radius_forw)/2.0;
                        ELossHitPattern.push_back(ELossPair);

                    }


            } //Number of hits (j)

            Double_t par0=0.0;
            Double_t par1=0.0;
            Double_t afit=0.0;


            if(HitPatternFilter[i]->GetN()>3){
               HitPatternFilter[i]->Fit("pol1","FQ");
               FitResult[i] = HitPatternFilter[i]->GetFunction("pol1");


               if(FitResult[i]){
                 Bool_t IsValid = FitResult[i]->IsValid();
                 FitResult[i] ->SetName(Form("fitResult%i",i));
                 par0 = FitResult[i]->GetParameter(0);
                 par1 = FitResult[i]->GetParameter(1);

                 if(par1>=0) afit = TMath::ATan2(1,TMath::Abs(par1));
                 else if(par1<0)  afit = TMath::Pi()-TMath::ATan2(1,TMath::Abs(par1));

                       //std::cout<<" Par 0 : "<<par0<<std::endl;
                       //std::cout<<" Par 1 : "<<par1<<std::endl;
                       //if(gMinuit && debug) std::cout<<gMinuit->fCstatu<<std::endl;


                 }



            }//if qNumHit


            fPar0_fit.push_back(par0);
            fPar1_fit.push_back(par1);
            fAngle_fit.push_back(afit*180/TMath::Pi());
            fRange.push_back(range);

            if(i==0) protoeventAna->SetELHitPattern(ELossHitPattern);
            QELossHitPattern.push_back(ELossHitPattern);


      }// Quadrant loop (i)


          protoeventAna->SetAngle(fAngle);
          protoeventAna->SetAngleFit(fAngle_fit);
          protoeventAna->SetPar0(fPar0_fit);
          protoeventAna->SetPar1(fPar1_fit);
          protoeventAna->SetRange(fRange);
          protoeventAna->SetHoughPar(fHoughPar);
          protoeventAna->SetQELHitPattern(QELossHitPattern);





}
