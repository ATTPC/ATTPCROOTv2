/*******************************************************************
* Daughter class for Linear Hough Space transformation             *
* Log: Class started 28-04-2015                                    *
* Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
********************************************************************/

#ifndef ATHOUGHSPACELINE_H
#define ATHOUGHSPACELINE_H

#include "ATHoughSpace.hh"
#include "TH2F.h"
#include "TF1.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"


class ATHoughSpaceLine : public ATHoughSpace{

  public:
	 ATHoughSpaceLine();
  ~ATHoughSpaceLine();

  template <class GenHough>
  void  CalcGenHoughSpace(GenHough event);

	TH2F* GetHoughSpace(TString ProjPlane);
  void CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane);
  void CalcHoughSpace(ATEvent* event,TH2Poly* hPadPlane);
  void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4);
  void CalcHoughSpace(ATEvent* event,TH2Poly* hPadPlane,multiarray PadCoord);
  void CalcMultiHoughSpace(ATEvent* event);
  //TH2F* GetHoughQuadrant(Int_t index);
	std::pair<Double_t,Double_t> GetHoughParameters(TH2F* hist);
  std::vector<std::pair<Double_t,Double_t>> GetMultiHoughParameters(TH2F* hist);
  std::pair<Double_t,Double_t> GetHoughParameters(); //Overloaded version for std::map
	std::vector<std::pair<Double_t,Double_t>> GetHoughPar(TString opt="Hist");
  std::vector<Double_t> GetHoughMax();
  void FillHoughMap(Double_t ang, Double_t dist);
  void SetRadiusThreshold(Float_t value);


      protected:
        Int_t fThreshold;
        Int_t fRadThreshold;
        Double_t fDriftVelocity;
        Int_t fTBTime;
        Int_t fEntTB;
        Float_t fZk;
        Int_t fXbinRZ;
        Int_t fYbinRZ;

        std::map<std::vector<Float_t>,Int_t> HoughMap_XZ;
        TH2F *HistHoughXZ;
        TH2F *HistHoughRZ;
        //TH2F *HistHoughRZ[4]; //One per quadrant
      	std::vector<std::pair<Double_t,Double_t>> HoughPar;
        std::vector<std::pair<Double_t,Double_t>> HoughParSTD;
        std::map<ULong64_t,Int_t> HoughMap; //8 byte for the key, unsigned, no negative distance in Linear Hough space is expected for each quadrant (Radius and Z are the vairbales)
        std::vector<ULong64_t> HoughMapKey;
        std::vector<Double_t> HoughMax;

        struct maxpersecond
        {
            template <typename Lhs, typename Rhs>
              bool operator()(const Lhs& lhs, const Rhs& rhs) const
                {
                  return lhs.second < rhs.second;
                }
        };


        ClassDef(ATHoughSpaceLine, 1);

};

// Template that works for any AT container with Hits inside. It works on the X-Z projection for the time being
template <class GenHough>
void ATHoughSpaceLine::CalcGenHoughSpace(GenHough event)
{

  Int_t nHits = event->GetNumHits();
  Double_t drift_cal = fDriftVelocity*fTBTime/100.0;//mm
  //TH2F *rad_z = new TH2F("rad_z","rad_z",1000,-500,500,1000,0,1000);
  //TH2F *rad_z2 = new TH2F("rad_z2","rad_z2",1000,-500,500,1000,0,1000);
  //rad_z2->SetMarkerColor(kRed);
  //rad_z->SetMarkerSize(2.0);

  std::vector<ATHit*> HitBuffer;
  std::vector<ATHit*> HitBuffer2;

  for(Int_t iHit=0; iHit<nHits; iHit++){
        ATHit* hit = event->GetHit(iHit);
        Int_t PadNumHit = hit->GetHitPadNum();
        TVector3 position = hit->GetPosition();
        Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );


        Int_t itheta=0;
        for(itheta = 0; itheta <1023; itheta++){
              Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
              Float_t d0_RZ = (TMath::Cos(angle)*position.X())  +  (TMath::Sin(angle)*hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
              HistHoughRZ->Fill(angle,d0_RZ);

         }//Hough Angle loop

      }

    std::pair<Double_t,Double_t> HoughParBuff = GetHoughParameters(HistHoughRZ);
    HoughPar.push_back(HoughParBuff);


    for(Int_t iHit=0; iHit<nHits; iHit++){
        ATHit* hit = event->GetHit(iHit);
        Int_t PadNumHit = hit->GetHitPadNum();
        TVector3 position = hit->GetPosition();
        Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );
        Double_t geo_dist = TMath::Abs (TMath::Cos(HoughParBuff.first)*position.X()  + TMath::Sin(HoughParBuff.first)*hit->GetTimeStamp()  - HoughParBuff.second);
        //rad_z2->Fill(hit->GetTimeStamp(),position.X());
         if(geo_dist>fHoughDist){
           HitBuffer.push_back(hit);
           //rad_z->Fill(hit->GetTimeStamp(),position.X());
         }
    }

    //rad_z2->Draw();
    //rad_z->Draw("SAMES");


    //******************************************** Recursive Hough Space calculation ***********************************

  //std::cout<<HitBuffer.size()<<std::endl;

  for(Int_t i=0;i<3;i++){

    HistHoughRZ->Reset();
    //rad_z->Reset();

      for(Int_t iHit=0; iHit<HitBuffer.size(); iHit++){
          ATHit* hit = HitBuffer.at(iHit);
          Int_t PadNumHit = hit->GetHitPadNum();
          TVector3 position = hit->GetPosition();
          Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );

          Int_t itheta=0;
          //#pragma omp parallel for ordered schedule(dynamic) private(itheta) //TOOD: Check performance
          for(itheta = 0; itheta <1023; itheta++){
                Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                Float_t d0_RZ = (TMath::Cos(angle)*position.X())  +  (TMath::Sin(angle)*hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
                //#pragma omp ordered
                HistHoughRZ->Fill(angle,d0_RZ);
                //rad_z->Fill(hit->GetTimeStamp(),radius);


           }//Hough Angle loop
        }


       HoughParBuff = GetHoughParameters(HistHoughRZ);
       HoughPar.push_back(HoughParBuff);


      //std::cout<<"  Iterative Hough Space  "<<i<<std::endl;
      //std::cout<<"  Hough Parameter Angle : "<<HoughPar.first<<std::endl;
     //std::cout<<"  Hough Parameter Distance : "<<HoughPar.second<<std::endl;

      //fHoughLinearFit->SetParameter(0,TMath::Pi()/2.0-HoughPar.first);
      //fHoughLinearFit->SetParameter(1,HoughPar.second);

       for(Int_t iHit=0; iHit<HitBuffer.size(); iHit++){
           ATHit* hit = HitBuffer.at(iHit);
           Int_t PadNumHit = hit->GetHitPadNum();
           TVector3 position = hit->GetPosition();
           Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );
           Double_t geo_dist = TMath::Abs (TMath::Cos(HoughParBuff.first)*position.X()  + TMath::Sin(HoughParBuff.first)*hit->GetTimeStamp()  - HoughParBuff.second);
            if(geo_dist>fHoughDist){
              //std::cout<<geo_dist<<" Time Bucket : "<<hit->GetTimeStamp()<<std::endl;
              HitBuffer2.push_back(hit);
              //rad_z->Fill(hit->GetTimeStamp(),radius);
            }
       }

       HitBuffer.clear();
       HitBuffer=HitBuffer2;
       HitBuffer2.clear();

     }

    //rad_z->Draw();
    //fHoughLinearFit->Draw("SAME");


    for(Int_t iHit=0; iHit<nHits; iHit++){
          ATHit* hit = event->GetHit(iHit);
          Int_t PadNumHit = hit->GetHitPadNum();
          TVector3 position = hit->GetPosition();
          Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );

          Int_t itheta=0;

          for(itheta = 0; itheta <1023; itheta++){
                Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                Float_t d0_RZ = (TMath::Cos(angle)*position.X())  +  (TMath::Sin(angle)*hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
                //#pragma omp ordered
                HistHoughRZ->Fill(angle,d0_RZ);



           }//Hough Angle loop
          //}
        }






}


#endif
