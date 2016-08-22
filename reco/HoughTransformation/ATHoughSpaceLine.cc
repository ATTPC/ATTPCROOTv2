#include "ATHoughSpaceLine.hh"
#include "TCanvas.h"
#ifdef _OPENMP
#include <omp.h>
#endif

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATHoughSpaceLine)

ATHoughSpaceLine::ATHoughSpaceLine()
{

    fXbinRZ = 500.0;
    fYbinRZ = 500.0;
    //fHoughDist=5.0;
    HistHoughXZ = new TH2F("HistHoughXZ","HistHoughXZ",500,0,3.15,500,0,300);
    HistHoughRZ = new TH2F("HistHoughRZ","HistHoughRZ",fXbinRZ,0,3.15,fYbinRZ,-300,300);
    fRadThreshold = 0.0;

    /*Char_t HoughQuadHistName[256];
    for(Int_t i=0;i<4;i++){
     sprintf(HoughQuadHistName,"HoughQuad_%d",i);
     HistHoughRZ[i] = new TH2F(HoughQuadHistName,HoughQuadHistName,500,0,3.15,2500,0,1000);
    }*/

    FairLogger *fLogger=FairLogger::GetLogger();
    ATDigiPar *fPar;

    FairRun *run = FairRun::Instance();
    if (!run)
      fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

    FairRuntimeDb *db = run -> GetRuntimeDb();
    if (!db)
      fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

    fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
    if (!fPar)
      fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");

    fZk = 500.0; //Prototype
    fDriftVelocity = fPar -> GetDriftVelocity();
    fEntTB   = (Int_t) fPar->GetTBEntrance();
    fTBTime = fPar -> GetTBTime();
    //std::cout<<cRED<< fDriftVelocity<<"          "<<fTBTime<<cNORMAL<<std::endl;

    HoughPar.clear();
    HoughParSTD.clear();
    HoughMax.clear();



}

ATHoughSpaceLine::~ATHoughSpaceLine()
{
      delete HistHoughXZ;
      delete HistHoughRZ;
  //for(Int_t i=0;i<4;i++) delete HistHoughRZ[i];
}

void ATHoughSpaceLine::SetRadiusThreshold(Float_t value) {fRadThreshold=value;}
std::vector<Double_t> ATHoughSpaceLine::GetHoughMax()    {return HoughMax;}

TH2F* ATHoughSpaceLine::GetHoughSpace(TString ProjPlane) {return HistHoughRZ;}
//TH2F* ATHoughSpaceLine::GetHoughQuadrant(Int_t index) {return HistHoughRZ[index];}
std::vector<std::pair<Double_t,Double_t>> ATHoughSpaceLine::GetHoughPar(TString opt)
{
  if(opt.EqualTo("Hist")) return HoughPar;
  else if(opt.EqualTo("STD")) return HoughParSTD;

}

void ATHoughSpaceLine::CalcHoughSpace(ATEvent* event,TH2Poly* hPadPlane)
{


}

void ATHoughSpaceLine::CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane){

	Int_t nHits = event->GetNumHits();


		for(Int_t iHit=0; iHit<nHits; iHit++){
			ATHit hit = event->GetHitArray()->at(iHit);
    	Int_t PadNumHit = hit.GetHitPadNum();
			if(hit.GetCharge()<fThreshold) continue;
   		        TVector3 position = hit.GetPosition();

			for(Int_t itheta = 0; itheta <1023; itheta++){
	 				Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
               	    		 if(XZplane){
                      	 		 Float_t d0_XZ = (TMath::Cos(angle)*position.X())  +  (TMath::Sin(angle)*position.Z());
                      			 HistHoughXZ->Fill(angle,d0_XZ);

                   		 }

			}

	  }// Hits loop

}

void ATHoughSpaceLine::CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4){


      Int_t nQuads = protoevent->GetNumQuadrants();
      std::vector<ATProtoQuadrant> quadrantArray;
      ATProtoQuadrant* quadrant;
      Double_t drift_cal = fDriftVelocity*fTBTime/100.0;//mm

	      TH2F *HistHoughRZ[4]; //One per quadrant
        Char_t HoughQuadHistName[256];
        for(Int_t i=0;i<4;i++){
           sprintf(HoughQuadHistName,"HoughQuad_%d",i);
          // HistHoughRZ[i] = new TH2F(HoughQuadHistName,HoughQuadHistName,200,0,3.15,1000,0,600);//Was (500,0,3.15,....
           HistHoughRZ[i] = new TH2F(HoughQuadHistName,HoughQuadHistName,500,0,3.15,1000,0,600);//Was (500,0,3.15,....
	}

      if(nQuads<5){

        for(Int_t iQ=0; iQ<nQuads; iQ++)
        {


            HoughMap.clear();
            //std::cout<<" =========  Quadrant : "<<iQ<<std::endl;
            quadrant = &protoevent->GetQuadrantArray()->at(iQ);
            Int_t qNumHits = quadrant->GetNumHits();


                    	for(Int_t iHit=0; iHit<qNumHits; iHit++){
                            ATHit* hit = quadrant->GetHit(iHit);
                            Int_t PadNumHit = hit->GetHitPadNum();
                            TVector3 position = hit->GetPosition();
                            Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );
                            Double_t posZCal     = fZk - (fEntTB - hit->GetTimeStamp())*drift_cal; // Recalibrating Z position from Time Bucket
                            Double_t posZCalCorr = fZk - (fEntTB - hit->GetTimeStampCorr())*drift_cal; // Recalibrating Z position from Time Bucket


                            //std::cout<<" Pos Z Vector : "<<position.Z()<<" posZCal : "<<posZCal<<" fEntTB : "<<fEntTB<<" Time Stamp : "<<hit->GetTimeStamp()<<" Drift cal : "<<drift_cal<<std::endl;


                            if(radius>fRadThreshold){
                              Int_t itheta=0;
			                        //#pragma omp parallel for ordered schedule(dynamic) private(itheta) //TOOD: Check performance
                              for(itheta = 0; itheta <1023; itheta++){
                                    Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                                    Float_t d0_RZ = (TMath::Cos(angle)*radius)  +  (TMath::Sin(angle)*posZCalCorr); // posZCal can be replaced anytime by position.Z()
                                    //#pragma omp ordered
                                    HistHoughRZ[iQ]->Fill(angle,d0_RZ);
                                    //#pragma omp ordered
                                    //FillHoughMap(angle,d0_RZ);

                               }//Hough Angle loop
                            }

                      }// Hit loop


      //HoughMapKey.push_back(GetHoughParameters());
       		HoughPar.push_back(GetHoughParameters(HistHoughRZ[iQ]));
      //HoughParSTD.push_back(GetHoughParameters());

         }// Quadrant loop
       }

	for(Int_t i=0;i<4;i++) delete HistHoughRZ[i];
	/*	TCanvas *c1=new TCanvas();
		c1->Divide(2,2);
		c1->cd(1);
		HistHoughRZ[0]->Draw("zcol");
		c1->cd(2);
		HistHoughRZ[1]->Draw("zcol");
		c1->cd(3);
		HistHoughRZ[2]->Draw("zcol");
		c1->cd(4);
		HistHoughRZ[3]->Draw("zcol");*/

}

void ATHoughSpaceLine::CalcHoughSpace(ATEvent* event,TH2Poly* hPadPlane,multiarray PadCoord)
{


}


void ATHoughSpaceLine::CalcMultiHoughSpace(ATEvent* event)
{

  Int_t nHits = event->GetNumHits();
  Double_t drift_cal = fDriftVelocity*fTBTime/100.0;//mm

  std::vector<ATHit*> HitBuffer;
  std::vector<ATHit*> HitBuffer2;

  //HistHoughRZ = new TH2F("HistHoughRZ","HistHoughRZ",500,0,3.15,500,-300,300);

  //TH2F *rad_z = new TH2F("rad_z","rad_z",1000,-500,500,1000,0,1000);

  //TF1* fHoughLinearFit = new TF1("HoughLinearFit"," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])",0,500);

  //TF1 *fHoughLinearFit;
  //fHoughLinearFit = new TF1("HoughLinearFit"," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])",0,500);

  for(Int_t iHit=0; iHit<nHits; iHit++){
        ATHit* hit = event->GetHit(iHit);
        Int_t PadNumHit = hit->GetHitPadNum();
        TVector3 position = hit->GetPosition();
        Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );
        //Double_t posZCal     = fZk - (fEntTB - hit->GetTimeStamp())*drift_cal; // Recalibrating Z position from Time Bucket
        //Double_t posZCalCorr = fZk - (fEntTB - hit->GetTimeStampCorr())*drift_cal; // Recalibrating Z position from Time Bucket


        Int_t itheta=0;
        //if(hit->GetTimeStamp()>130){
        //#pragma omp parallel for ordered schedule(dynamic) private(itheta) //TOOD: Check performance
        for(itheta = 0; itheta <1023; itheta++){
              Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
              Float_t d0_RZ = (TMath::Cos(angle)*radius)  +  (TMath::Sin(angle)*hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
              //#pragma omp ordered
              HistHoughRZ->Fill(angle,d0_RZ);
              //rad_z->Fill(hit->GetTimeStamp(),radius);


         }//Hough Angle loop
        //}
      }

      //delete HistHoughRZ;
      //HistHoughRZ->Draw("zcol");
      //rad_z->Draw();

      std::pair<Double_t,Double_t> HoughParBuff = GetHoughParameters(HistHoughRZ);
      HoughPar.push_back(HoughParBuff);


      /*std::cout<<"  ++++++++++++++++++++++++++++++++++++++ "<<std::endl;
      std::cout<<"  First Hough Space :  "<<std::endl;
      std::cout<<"  Hough Parameter Distance : "<<HoughPar.first<<std::endl;
  		std::cout<<"  Hough Parameter Angle : "<<HoughPar.second<<std::endl;*/

      //fHoughLinearFit->SetParameter(0,TMath::Pi()/2.0-HoughPar.first);
      //fHoughLinearFit->SetParameter(1,HoughPar.second);

    //rad_z->clear();

      for(Int_t iHit=0; iHit<nHits; iHit++){
          ATHit* hit = event->GetHit(iHit);
          Int_t PadNumHit = hit->GetHitPadNum();
          TVector3 position = hit->GetPosition();
          Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );
          Double_t geo_dist = TMath::Abs (TMath::Cos(HoughParBuff.first)*radius  + TMath::Sin(HoughParBuff.first)*hit->GetTimeStamp()  - HoughParBuff.second);
           if(geo_dist>fHoughDist){
             HitBuffer.push_back(hit);
             //rad_z->Fill(hit->GetTimeStamp(),radius);
           }
      }

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
                  Float_t d0_RZ = (TMath::Cos(angle)*radius)  +  (TMath::Sin(angle)*hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
                  //#pragma omp ordered
                  HistHoughRZ->Fill(angle,d0_RZ);
                  //rad_z->Fill(hit->GetTimeStamp(),radius);


             }//Hough Angle loop
          }


         HoughParBuff = GetHoughParameters(HistHoughRZ);
         HoughPar.push_back(HoughParBuff);


      /*  std::cout<<"  Iterative Hough Space  "<<i<<std::endl;
        std::cout<<"  Hough Parameter Angle : "<<HoughPar.first<<std::endl;
     		std::cout<<"  Hough Parameter Distance : "<<HoughPar.second<<std::endl;*/

        //fHoughLinearFit->SetParameter(0,TMath::Pi()/2.0-HoughPar.first);
        //fHoughLinearFit->SetParameter(1,HoughPar.second);

         for(Int_t iHit=0; iHit<HitBuffer.size(); iHit++){
             ATHit* hit = HitBuffer.at(iHit);
             Int_t PadNumHit = hit->GetHitPadNum();
             TVector3 position = hit->GetPosition();
             Float_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2)  );
             Double_t geo_dist = TMath::Abs (TMath::Cos(HoughParBuff.first)*radius  + TMath::Sin(HoughParBuff.first)*hit->GetTimeStamp()  - HoughParBuff.second);
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
                  Float_t d0_RZ = (TMath::Cos(angle)*radius)  +  (TMath::Sin(angle)*hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
                  //#pragma omp ordered
                  HistHoughRZ->Fill(angle,d0_RZ);



             }//Hough Angle loop
            //}
          }





}


std::pair<Double_t,Double_t> ATHoughSpaceLine::GetHoughParameters(TH2F* hist){

		std::pair<Double_t,Double_t> HoughParBuff;
    Double_t xpos=0.0;
    Double_t ypos=0.0;
		Int_t locmaxx,locmaxy,locmaxz;
    hist->GetMaximumBin(locmaxx,locmaxy,locmaxz);
    xpos = hist->GetXaxis()->GetBinCenter(locmaxx);
    ypos = hist->GetYaxis()->GetBinCenter(locmaxy);
    HoughMax.push_back(hist->GetBinContent(locmaxx,locmaxy));
		//std::cout<<" X Hough Position : "<<180-xpos*180/TMath::Pi()<<std::endl;
    //std::cout<<" X Hough Position : "<<xpos<<std::endl;
		//std::cout<<" Y Hough Position : "<<ypos<<std::endl;
		HoughParBuff.first= xpos;
		HoughParBuff.second= ypos;
		return HoughParBuff;

}


std::vector<std::pair<Double_t,Double_t>> ATHoughSpaceLine::GetMultiHoughParameters(TH2F* hist)
{

      std::vector<std::pair<Double_t,Double_t>> HoughPar;
      //Int_t nBins = hist->GetSize();
      Int_t maxBin=0;
      Int_t maxBin2=0;
      Int_t buff=0;
      Double_t xpos1=0.0;
      Double_t ypos1=0.0;

        for(Int_t binx=1;binx<fXbinRZ;binx++){
          for(Int_t biny=1;biny<fYbinRZ;biny++){
              buff = hist->GetBinContent(binx,biny);
                if(buff>maxBin){
                  maxBin=buff;
                  xpos1 = hist->GetXaxis()->GetBinCenter(binx);
                  ypos1 = hist->GetYaxis()->GetBinCenter(biny);
                }
          }
        }

    //  std::cout<<maxBin<<std::endl;
    //  std::cout<<" X Hough Position 1: "<<xpos1<<std::endl;
  	//	std::cout<<" Y Hough Position 1: "<<ypos1<<std::endl;

      return HoughPar;

}


std::pair<Double_t,Double_t> ATHoughSpaceLine::GetHoughParameters()
{

    std::pair<Double_t,Double_t> HoughParBuff;
    // Method 1:
    /*  using pair_type = decltype(HoughMap)::value_type;

       auto pr = std::max_element
        (
            std::begin(HoughMap), std::end(HoughMap),
            [] (const pair_type & p1, const pair_type & p2) {
              return p1.second < p2.second;
            }
         );

          std::cout << "pr->first : " << pr->first<< '\n';
          std::cout << "pr->second : " << pr->second<< '\n';
          std::cout << " pr->first&0xFFFFFFFF " << (pr->first&0xFFFFFFFF) << '\n';
          std::cout << " ((pr->first>>32)&0xFFFFFFFF) " << ((pr->first>>32)&0xFFFFFFFF) << '\n';*/


     //Method 2:
           std::map<ULong64_t,Int_t>::iterator maximum = std::max_element(HoughMap.begin(), HoughMap.end(), maxpersecond());
           HoughParBuff.first=(maximum->first&0xFFFFFFFF)/159.0; //Binning of the associated histogram
           HoughParBuff.second=((maximum->first&0xFFFFFFFF00000000)>>32)/17;
          // std::cout<<" Size of First Element in Hough Map (byte) :"<<sizeof(maximum->first)<<std::endl;
          // std::cout<<" Size of Second Element of Houg Map (byte) :"<<sizeof(maximum->second)<<std::endl;
          // std::cout<<" Angle : "<<(maximum->first&0xFFFFFFFF)/159.0<<std::endl;
           //std::cout<<" Distance : "<<((maximum->first&0xFFFFFFFF00000000)>>32)/17<<std::endl;
          // std::cout<<" Number of Maximum Votes : "<<maximum->second<<std::endl;
          return HoughParBuff;

}

void ATHoughSpaceLine::FillHoughMap(Double_t ang, Double_t dist)
{


      ULong64_t MapKey;

      //if(ang<0 || dist<0)  std::cout<<" -I- ATHougSpaceLine : Negative values for Hough Space parameters "<<std::endl;

      if(ang>0 || dist>0){
      Int_t dist_enc = static_cast<Int_t>(round(dist*17));
      Int_t ang_enc = static_cast<Int_t>(round(ang*159.0));
      /*std::cout<<" Filling std::map for the Hough Space "<<std::endl;
      std::cout<<" Hough Angle : "<<ang<<" Hough Dist : "<<dist<<std::endl;
      std::cout<<" Encoded Hough Angle : "<<ang_enc<<" Encoded Hough Dist : "<<dist_enc<<std::endl;*/

      MapKey = dist_enc&0xFFFFFFFF;
      MapKey = (MapKey<<32) + (ang_enc&0xFFFFFFFF);
      HoughMap[MapKey]++;

    /*  std::cout<<" Map Key : "<<MapKey<<std::endl;
      std::cout<<" Map Key Dist "<<(MapKey&0xFFFFFFFF)<<std::endl;
      std::cout<<" Map Key Ang "<<((MapKey>>32)&(0xFFFFFFFF))<<std::endl;*/

      }


}
