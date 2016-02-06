#include "ATPhiReco.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

// STL
#include <algorithm>


ClassImp(ATPhiReco)

ATPhiReco::ATPhiReco()
{
  
    //TODO: IS THIS NEEDED??? 
 /* fLogger = FairLogger::GetLogger();

  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");*/

 
}

ATPhiReco::~ATPhiReco()
{
}

/*Double_t ATPhiReco::PhiCalc(ATProtoQuadrant *quadrant)
{

     
     Double_t a=0.5; //Small size of the strip
     Double_t da; //
  
     // type A: a + da - Phi/90
     // type B: a + Phi/90
  
     // TODO This function is tuned for prototype micromegas, 
       Int_t nHits = quadrant->GetNumHits();
		if(nHits>1){
		      //std::cout<<" Number of hits in quadrant  : "<<nHits<<std::endl;
		       for(Int_t iHit=0; iHit<nHits-1; iHit++){
			   Double_t phi=0.0;
			  ATHit qhit_f = quadrant->GetHitArray()->at(iHit); //First strip
			  ATHit qhit_s = quadrant->GetHitArray()->at(iHit+1); //Second strip
			  Int_t PadNum_qf = qhit_f.GetHitPadNum();
			  Int_t PadNum_qs = qhit_s.GetHitPadNum();
			  Double_t Q_f = qhit_f.GetCharge(); //TODO: For the moment we asume the charge to be the amplitude
			  Double_t Q_s = qhit_s.GetCharge(); 
                         	 if(PadNum_qf<11) da=1.0-a;
			  	 else da=3.2-a;
                                 if(PadNum_qs<11) da=1.0-a;
			  	 else da=3.2-a;
                             

			        if(PadNum_qs==PadNum_qf+1){


					if(quadrant->GetQuadrantID()%2==1){ 
						//std::cout<<" "<<std::endl;
						//std::cout<<" Inside an odd quadrant ID : "<<quadrant->GetQuadrantID()<<std::endl;
						phi= (1.0 - (Q_f-Q_s)/(Q_f+Q_s)*2.0*a/da - (Q_f-Q_s)/(Q_f+Q_s) )*45 ;
						if(PadNum_qf%2==1 && PadNum_qs%2==0) phi= (1.0 - (Q_f-Q_s)/(Q_f+Q_s)*2.0*a/da - (Q_f-Q_s)/(Q_f+Q_s) )*45 ;
						else if(PadNum_qf%2==0 && PadNum_qs%2==1) phi = 90.0-phi;
						else{ std::cout<<" ======================================================================= "<<std::endl;
						      std::cout<<" Warning, even-odd sttagering not found. "<<std::endl;
						}


					}

					else if(quadrant->GetQuadrantID()%2==0){

						//std::cout<<" Inside an even quadrant ID : "<<quadrant->GetQuadrantID()<<std::endl;
                                                phi= (1.0 - (Q_f-Q_s)/(Q_f+Q_s)*2.0*a/da - (Q_f-Q_s)/(Q_f+Q_s) )*45 ;
						if(PadNum_qf%2==0 && PadNum_qs%2==1) phi= (1.0 - (Q_f-Q_s)/(Q_f+Q_s)*2.0*a/da - (Q_f-Q_s)/(Q_f+Q_s) )*45 ;
						else if(PadNum_qf%2==1 && PadNum_qs%2==0) phi = 90.0-phi;
						else{ std::cout<<" ======================================================================= "<<std::endl;
						      std::cout<<" Warning, even-odd sttagering not found. "<<std::endl;
						}
						

					}else std::cout<<" Invalid Quadrant ID : "<<quadrant->GetQuadrantID()<<std::endl;


				}else{
					//std::cout<<" ======================================================================= "<<std::endl;
					//std::cout<<" Warning, Pads are not adjacent. "<<std::endl;
					phi=0.0;

				 } //Adjacent strips
	                               
			        // std::cout<<" ======================================================================= "<<std::endl;
				// std::cout<<" Prototype quadrant : "<<quadrant->GetQuadrantID()<<std::endl;
                     		// std::cout<<" First Hit Pad : "<<PadNum_qf<<" First Pad Charge : "<<Q_f<<std::endl;
                                // std::cout<<" Second Hit Pad : "<<PadNum_qs<<" Second Pad Charge : "<<Q_s<<std::endl;
				// std::cout<<" Phi : "<<phi<<std::endl;
 
		       }
                 }//nHits>1

             

}
*/













