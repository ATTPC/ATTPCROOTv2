#include "ATPSAProto.hh"
#include "TH1.h"

// STL
#include <cmath>
#include <map>
#ifdef _OPENMP
#include <omp.h>
#endif

ClassImp(ATPSAProto)

ATPSAProto::ATPSAProto()
{
  //fPeakFinder = new TSpectrum();
  //HPeak = new TH1F("HPeak","HPeak",512,0,511);
}

ATPSAProto::~ATPSAProto()
{
}

void
ATPSAProto::Analyze(ATRawEvent *rawEvent, ATEvent *event)
{
  Int_t numPads = rawEvent -> GetNumPads();
  Int_t hitNum = 0;
  Double_t QEventTot=0.0;
  Double_t RhoVariance = 0.0;
  Double_t RhoMean = 0.0;
  Double_t Rho2 = 0.0;
  std::map<Int_t,Int_t> PadMultiplicity;
  Float_t mesh[512] = {0};

  Int_t iPad = 0;

  //#pragma omp parallel for ordered schedule(dynamic,1) private(iPad)
  for (iPad = 0; iPad < numPads; iPad++) {


      ATPad *pad = rawEvent -> GetPad(iPad);
      Int_t PadNum = pad->GetPadNum();
      Double_t QHitTot = 0.0;
      Int_t PadHitNum = 0;
      TVector3 HitPos;
      Bool_t fValidBuff = kTRUE;
      Bool_t fValidThreshold = kTRUE;

      Double_t xPos     = pad -> GetPadXCoord();
      Double_t yPos     = pad -> GetPadYCoord();
      Double_t zPos     = 0;
      Double_t zPosCorr = 0.0;
      Double_t charge   = 0;

      if((xPos<-9000 || yPos<-9000) && !pad->IsAux() )
      {
          //std::cout<<" Is Auxiliary? "<<pad->IsAux()<<" Pad Num "<<PadNum<<"\n";
          continue; //Skip invalid pads that are not
      }else if(pad->IsAux()){

        //std::cout<<" Is Auxiliary 2? "<<pad->IsAux()<<" Pad Num "<<PadNum<<"\n";
        event->AddAuxPad(pad);
        continue;
      }   

    if (!(pad -> IsPedestalSubtracted())) {
      fLogger -> Error(MESSAGE_ORIGIN, "Pedestal should be subtracted to use this class!");

      //return;
    }

    Double_t *adc = pad -> GetADC();
    Double_t floatADC[512] = {0};
    Double_t dummy[512] = {0};
    Int_t zeroPeak = 0;


      for (Int_t iTb = 0; iTb < fNumTbs; iTb++){
          floatADC[iTb] = adc[iTb];
          QHitTot+=adc[iTb];
      }

      TSpectrum *PeakFinder = new TSpectrum;
      Int_t numPeaks=0;
      //#pragma omp ordered
      numPeaks = PeakFinder -> SearchHighRes(floatADC, dummy, fNumTbs, 4.7, 5, fBackGroundSuppression, 3, kTRUE, 3);

      if(numPeaks==0) zeroPeak=1;
      else zeroPeak=0;
      //#pragma omp ordered
      //std::cout<<" iPad : "<<iPad<<" Num Peaks : "<<numPeaks<<std::endl;

    //if (numPeaks == 0) fValidBuff = kFALSE;
       //continue;


   if(fValidBuff){
    for (Int_t iPeak = 0; iPeak < numPeaks+zeroPeak; iPeak++) {
       Int_t maxAdcIdx=0;
       Float_t max=0.0;
       Float_t min=0.0;
       Int_t maxTime = 0;



      if(numPeaks>0) maxAdcIdx = (Int_t)(ceil((PeakFinder -> GetPositionX())[iPeak]));
      else if(numPeaks==0){

            //if no peaks are found, find the maximum value
            for (Int_t ij = 20; ij < 500; ij++) //Excluding first and last 10 Time Buckets
            {
              if (floatADC[ij] > max)
               {
                 max = floatADC[ij];
                 maxTime = ij;
              }
            }

            maxAdcIdx = maxTime;

      }//if numPeaks

              // Time Correction by Center of Gravity
              Double_t TBCorr=0.0;
              Double_t TB_TotQ = 0.0;
        
              if(maxAdcIdx>11){
                for(Int_t i=0;i<11;i++){

                    TBCorr  += floatADC[maxAdcIdx-i+5]*(maxAdcIdx-i+5);
                    TB_TotQ += floatADC[maxAdcIdx-i+5];

                }
              }

              TBCorr = TBCorr/TB_TotQ;

      zPos     = CalculateZGeo(maxAdcIdx);
      zPosCorr = CalculateZGeo(TBCorr);
      charge = adc[maxAdcIdx];
      //std::cout<<zPos<<"    "<<zPosCorr<<std::endl;

      //#pragma omp ordered
      //std::cout<<" Pad Num : "<<PadNum<<" Peak : "<<iPeak<<"/"<<numPeaks<<" - Charge : "<<charge<<" - zPos : "<<zPos<<std::endl;
      //std::cout<<" xPos : "<<xPos<<" yPos : "<<yPos<<"\n";

        if (fThreshold > 0 && charge < fThreshold )// TODO: Does this work when the polarity is negative??
            fValidThreshold = kFALSE;

      //if (zPos > 0 || zPos < -fMaxDriftLength)
      //if (zPos < 0 || zPos > fMaxDriftLength)
       // continue;

     if(fValidThreshold){
      
          if(iPeak==0) 
              for (Int_t iTb = fIniTB; iTb < fEndTB; iTb++) 
                    if(adc[iTb]>fThreshold) QEventTot+=adc[iTb]; //Sum only if Hit is valid - We only sum once (iPeak==0) to account for the whole spectrum.
        //std::cout<<" maxAdcIdx : "<<maxAdcIdx<<" TBCorr : "<<TBCorr<<std::endl;
        //std::cout<<zPos<<"    "<<zPosCorr<<std::endl;
      ATHit *hit = new ATHit(PadNum,hitNum, xPos, yPos, zPos, charge);
      PadHitNum++;
      hit->SetQHit(QHitTot); // TODO: The charge of each hit is the total charge of the spectrum, so for double structures this is unrealistic.
      hit->SetTimeStamp(maxAdcIdx);
      hit->SetTimeStampCorr(TBCorr);
      hit->SetPositionCorr(xPos, yPos, zPosCorr); //Only Z is corrected here
      HitPos =  hit->GetPosition();
      Rho2+= HitPos.Mag2();
      RhoMean+=HitPos.Mag();
      if((xPos<-9000 || yPos<-9000) && pad->GetPadNum()!=-1) std::cout<<" ATPSAProto::Analysis Warning! Wrong Coordinates for Pad : "<<pad->GetPadNum()<<std::endl;
      //std::cout<<"  =============== Next Hit Variance Info  =============== "<<std::endl;
      //std::cout<<" Hit Num : "<<hitNum<<"  - Hit Pos Rho2 : "<<HitPos.Mag2()<<"  - Hit Pos Rho : "<<HitPos.Mag()<<std::endl;
      //std::cout<<" Hit Coordinates : "<<xPos<<"  -  "<<yPos<<" - "<<zPos<<"  -  "<<std::endl;
      //std::cout<<" Is Pad"<<pad->GetPadNum()<<" Valid? "<<pad->GetValidPad()<<std::endl;
      //#pragma omp ordered
      event -> AddHit(hit);
      delete hit;
      //#pragma omp ordered
      hitNum++;

                if(PadHitNum==1){ //Construct mesh signal only once per PAD
                	for (Int_t iTb = 0; iTb < fNumTbs; iTb++){
                		if(adc[iTb]>fThreshold) mesh[iTb]+=floatADC[iTb];
                	}
                }

      }//Valid Threshold

    }// Peak loop


     //#pragma omp ordered
     PadMultiplicity.insert(std::pair<Int_t,Int_t>(PadNum,PadHitNum));



   }//if Valid Num Peaks (ValidBuff)

   delete PeakFinder;

  }// Pad loop
    // std::cout<<"  --------------------------------- "<<std::endl;
     std::cout<<" Rho2 : "<<Rho2<<" - RhoMean : "<<RhoMean<<" Num of Hits : "<<event->GetNumHits()<<std::endl;
     RhoVariance = Rho2 - ( pow(RhoMean,2)/(event->GetNumHits()) );
     RhoVariance = Rho2 - ( event->GetNumHits()*pow((RhoMean/event->GetNumHits()),2) ) ;
     //std::cout<<" Rho Variance : "<<RhoVariance<<std::endl;
     for (Int_t iTb = 0; iTb < fNumTbs; iTb++) event -> SetMeshSignal(iTb,mesh[iTb]);
     event -> SetMultiplicityMap(PadMultiplicity);
     event -> SetRhoVariance(RhoVariance);
     event -> SetEventCharge(QEventTot);

}
