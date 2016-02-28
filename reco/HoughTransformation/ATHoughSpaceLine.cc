#include "ATHoughSpaceLine.hh"
#include "TCanvas.h"
#ifdef _OPENMP
#include <omp.h>
#endif

ClassImp(ATHoughSpaceLine)

ATHoughSpaceLine::ATHoughSpaceLine()
{
    HistHoughXZ = new TH2F("HistHoughXZ","HistHoughXZ",500,0,3.15,500,0,300);
    fRadThreshold = 0.0;

    /*Char_t HoughQuadHistName[256];
    for(Int_t i=0;i<4;i++){
     sprintf(HoughQuadHistName,"HoughQuad_%d",i);
     HistHoughRZ[i] = new TH2F(HoughQuadHistName,HoughQuadHistName,500,0,3.15,2500,0,1000);
    }*/

}

ATHoughSpaceLine::~ATHoughSpaceLine()
{
      delete HistHoughXZ;
  //for(Int_t i=0;i<4;i++) delete HistHoughRZ[i];
}

void ATHoughSpaceLine::SetRadiusThreshold(Float_t value) {fRadThreshold=value;}

TH2F* ATHoughSpaceLine::GetHoughSpace(TString ProjPlane) {return HistHoughXZ;}
//TH2F* ATHoughSpaceLine::GetHoughQuadrant(Int_t index) {return HistHoughRZ[index];}
std::vector<std::pair<Double_t,Double_t>> ATHoughSpaceLine::GetHoughPar(TString opt)
{
  if(opt.EqualTo("Hist")) return HoughPar;
  else if(opt.EqualTo("STD")) return HoughParSTD;

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

	        }

}

void ATHoughSpaceLine::CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4){

      Int_t nQuads = protoevent->GetNumQuadrants();
      std::vector<ATProtoQuadrant> quadrantArray;
      ATProtoQuadrant* quadrant;
      //std::cout<<"  ========================================== "<<std::endl;

	TH2F *HistHoughRZ[4]; //One per quadrant
        Char_t HoughQuadHistName[256];
        for(Int_t i=0;i<4;i++){
           sprintf(HoughQuadHistName,"HoughQuad_%d",i);
           HistHoughRZ[i] = new TH2F(HoughQuadHistName,HoughQuadHistName,200,0,3.15,1000,0,600);//Was (500,0,3.15,....
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

                            if(radius>fRadThreshold){
                              Int_t itheta=0;
			                        //#pragma omp parallel for ordered schedule(dynamic) private(itheta) //TOOD: Check performance
                              for(itheta = 0; itheta <1023; itheta++){
                                    Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                                    Float_t d0_RZ = (TMath::Cos(angle)*radius)  +  (TMath::Sin(angle)*position.Z());
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


std::pair<Double_t,Double_t> ATHoughSpaceLine::GetHoughParameters(TH2F* hist){

		std::pair<Double_t,Double_t> HoughParBuff;
		Int_t locmaxx,locmaxy,locmaxz;
    hist->GetMaximumBin(locmaxx,locmaxy,locmaxz);
    Double_t xpos = hist->GetXaxis()->GetBinCenter(locmaxx);
    Double_t ypos = hist->GetYaxis()->GetBinCenter(locmaxy);
		//std::cout<<" X Hough Position : "<<180-xpos*180/TMath::Pi()<<std::endl;
  //  std::cout<<" X Hough Position : "<<xpos<<std::endl;
	//	std::cout<<" Y Hough Position : "<<ypos<<std::endl;
		HoughParBuff.first= xpos;
		HoughParBuff.second= ypos;
		return HoughParBuff;

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
