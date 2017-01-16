#include "ATPSAFilter.hh"
#include "TH1F.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TH2F.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

// STL
#include <cmath>
#include <map>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <algorithm>


ClassImp(ATPSAFilter)

ATPSAFilter::ATPSAFilter()
{

  fMeanK = 10;
  fStdDev = 0.01;

}

ATPSAFilter::~ATPSAFilter()
{
}

void
ATPSAFilter::Analyze(ATRawEvent *rawEvent, ATEvent *event)
{

  Int_t numPads = rawEvent -> GetNumPads();
  Int_t hitNum = 0;
  Double_t QEventTot=0.0;
  Double_t RhoVariance = 0.0;
  Double_t RhoMean = 0.0;
  Double_t Rho2 = 0.0;
  std::map<Int_t,Int_t> PadMultiplicity;
  Float_t mesh[512] = {0};
  std::vector<ATHit> hitBuff;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud->points.resize(numPads);


  /*TH2F* cloud_ini = new TH2F("cloud_ini","cloud_ini",1000,-250,250,1000,-250,250);
  cloud_ini->SetMarkerColor(kRed);
  cloud_ini->SetMarkerStyle(20);
  cloud_ini->SetMarkerSize(1.0);

  TH2F* cloud_fil = new TH2F("cloud_fil","cloud_fil",1000,-250,250,1000,-250,250);
  cloud_fil->SetMarkerColor(kBlack);
  cloud_fil->SetMarkerStyle(20);
  cloud_fil->SetMarkerSize(1.0);*/


  Int_t iPad=0;
  //#pragma omp parallel for ordered schedule(dynamic,1) private(iPad)
  for (iPad = 0; iPad < numPads; iPad++) {


      ATPad *pad = rawEvent -> GetPad(iPad);
      Int_t PadNum = pad->GetPadNum();
      Double_t QHitTot = 0.0;
      Int_t PadHitNum = 0;
      TVector3 HitPos;
      TVector3 HitPosRot;
      Bool_t fValidBuff = kTRUE;
      Bool_t fValidThreshold = kTRUE;
      Bool_t fValidDerivative = kTRUE;
      TRotation r;
      TRotation ry;
      TRotation rx;
      r.RotateZ(272.0*TMath::Pi()/180.0);
      ry.RotateY(180.0*TMath::Pi()/180.0);
      rx.RotateX(6.0*TMath::Pi()/180.0);

      Double_t xPos = pad -> GetPadXCoord();
      Double_t yPos = pad -> GetPadYCoord();
      Double_t zPos = 0;
      Double_t xPosRot = 0;
      Double_t yPosRot = 0;
      Double_t zPosRot = 0;
      Double_t xPosCorr = 0;
      Double_t yPosCorr = 0;
      Double_t zPosCorr = 0;
      Double_t charge = 0;
      Int_t maxAdcIdx=0;
      Int_t numPeaks=0;

    CalcLorentzVector();

      if (!(pad -> IsPedestalSubtracted())) {
      fLogger -> Error(MESSAGE_ORIGIN, "Pedestal should be subtracted to use this class!");

      //return;
      }

      Double_t *adc = pad -> GetADC();
      Double_t floatADC[512] = {0};
      Double_t dummy[512] = {0};


      for (Int_t iTb = 0; iTb < fNumTbs; iTb++){
          floatADC[iTb] = adc[iTb];
          QHitTot+=adc[iTb];

      }

  TSpectrum *PeakFinder = new TSpectrum;
  if(fIsPeakFinder) numPeaks = PeakFinder -> SearchHighRes(floatADC, dummy, fNumTbs, 4.7, 5, fBackGroundSuppression, 3, kTRUE, 3);
  if(fIsMaxFinder) numPeaks = 1;

  if (numPeaks == 0) fValidBuff = kFALSE;
       //continue;

   if(fValidBuff){

    for (Int_t iPeak = 0; iPeak < numPeaks; iPeak++) {

      Float_t max=0.0;
      Float_t min=0.0;
      Int_t maxTime = 0;

      if(fIsPeakFinder) maxAdcIdx = (Int_t)(ceil((PeakFinder -> GetPositionX())[iPeak]));

    //  Int_t maxAdcIdx = *std::max_element(floatADC,floatADC+512);

    if(fIsMaxFinder){
      for (Int_t ij = 20; ij < 500; ij++) //Excluding first and last 12 Time Buckets
      {
       //if(PadNum==9788) std::cout<<" Time Bucket "<<i<<" floatADC "<<floatADC[i]<<std::endl;

        if (floatADC[ij] > max)
         {
           max = floatADC[ij];
           maxTime = ij;

        }
      }

      maxAdcIdx = maxTime;

    }


      //Charge Correction due to mesh induction (base line)

      Double_t basecorr=0.0;
      Double_t slope = 0.0;
      Int_t slope_cnt =0;

        if(maxAdcIdx>20) for (Int_t i = 0; i < 10; i++){

           basecorr+=floatADC[maxAdcIdx-8-i];

           if(i<5){
             slope = (floatADC[maxAdcIdx-i] - floatADC[maxAdcIdx-i-1]); //Derivate for 5 Timebuckets
              //if(slope<0 && floatADC[maxAdcIdx]<3500 && fIsBaseCorr && fIsMaxFinder)
               //fValidDerivative = kFALSE; //3500 condition to avoid killing saturated pads
              if(slope<0 && fIsBaseCorr && fIsMaxFinder) slope_cnt++;
           }


         }

      //Calculation of the mean value of the peak time by interpolating the pulse

      Double_t timemax = 0.5*(floatADC[maxAdcIdx-1] - floatADC[maxAdcIdx+1])  /  (floatADC[maxAdcIdx-1] + floatADC[maxAdcIdx+1] - 2*floatADC[maxAdcIdx]);

      // Time Correction by Center of Gravity
      Double_t TBCorr=0.0;
      Double_t TB_TotQ = 0.0;


      if(maxAdcIdx>11){
        for(Int_t i=0;i<11;i++){

          TBCorr  += (floatADC[maxAdcIdx-i+5] - basecorr/10.0)*(maxAdcIdx-i+5); //Substract the baseline correction
          TB_TotQ += floatADC[maxAdcIdx-i+5] - basecorr/10.0;

       }
     }

    TBCorr = TBCorr/TB_TotQ;



      if(fIsBaseCorr) charge = adc[maxAdcIdx] - basecorr/10.0; //Number of timebuckets taken into account
      else charge = adc[maxAdcIdx];


      if(fIsTimeCorr) zPos = CalculateZGeo(TBCorr);
      else zPos = CalculateZGeo(maxAdcIdx);
      //std::cout<<" zPos : "<<zPos<<" maxAdcIdx : "<<maxAdcIdx<<" timemax : "<<timemax<<std::endl;

    if(fIsMaxFinder){
      if ((fThreshold > 0 && charge < fThreshold ) || maxTime<20 || maxTime>500)// TODO: Does this work when the polarity is negative??
             fValidThreshold = kFALSE;
    }

    if(fIsPeakFinder) {
        if (fThreshold > 0 && charge < fThreshold)
         fValidThreshold = kFALSE;
    }



    if(fValidThreshold && fValidDerivative){

      if(iPeak==0) QEventTot+=QHitTot; //Sum only if Hit is valid - We only sum once (iPeak==0) to account for the whole spectrum.


      TVector3 posRot =  RotateDetector(xPos,yPos,zPos,maxAdcIdx);

      ATHit *hit = new ATHit(PadNum,hitNum, xPos, yPos, zPos, charge);
      cloud->points[hitNum].x   = xPos;
      cloud->points[hitNum].y   = yPos;
      cloud->points[hitNum].z   = zPos;
      cloud->points[hitNum].rgb = hitNum;

      //cloud_ini->Fill(xPos,yPos);


      hit->SetPositionCorr(posRot.X(),posRot.Y(), posRot.Z());
      hit->SetTimeStamp(maxAdcIdx);
      hit->SetTimeStampCorr(TBCorr);
      hit->SetTimeStampCorrInter(timemax);
      hit->SetBaseCorr(basecorr/10.0);
      hit->SetSlopeCnt(slope_cnt);
      PadHitNum++;
      hit->SetQHit(QHitTot); // TODO: The charge of each hit is the total charge of the spectrum, so for double structures this is unrealistic.
      HitPos =  hit->GetPosition();
      Rho2+= HitPos.Mag2();
      RhoMean+=HitPos.Mag();
      if((xPos<-9000 || yPos<-9000) && pad->GetPadNum()!=-1 ) std::cout<<" ATPSASimple2::Analysis Warning! Wrong Coordinates for Pad : "<<pad->GetPadNum()<<std::endl;


      //#pragma omp ordered
      //event -> AddHit(hit);
      hitBuff.push_back(*hit);
      delete hit;

      hitNum++;


        for (Int_t iTb = 0; iTb < fNumTbs; iTb++){
          mesh[iTb]+=floatADC[iTb];
        }

       }//Valid Threshold

      }//Peak Loop

      //#pragma omp ordered
      PadMultiplicity.insert(std::pair<Int_t,Int_t>(PadNum,PadHitNum));

    }//if Valid Num Peaks

    delete PeakFinder;

 }//Pad Loop

      //Inliers
      cloud->points.resize(hitNum);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(fMeanK);
      sor.setStddevMulThresh(fStdDev);
      sor.filter(*cloud_filtered);
      //Outliers
      //sor.setNegative(true);
      //sor.filter(*cloud_filtered);


          for(Int_t pc = 0;pc<cloud_filtered->points.size();pc++)
          {
            //cloud_fil->Fill(cloud_filtered->points[pc].x,cloud_filtered->points[pc].y);
            event -> AddHit(&hitBuff.at(cloud_filtered->points[pc].rgb));

          }


     RhoVariance = Rho2 - ( pow(RhoMean,2)/(event->GetNumHits()) );
     RhoVariance = Rho2 - ( event->GetNumHits()*pow((RhoMean/event->GetNumHits()),2) ) ;

     for (Int_t iTb = 0; iTb < fNumTbs; iTb++) event -> SetMeshSignal(iTb,mesh[iTb]);
     event -> SortHitArrayTime();
     event -> SetMultiplicityMap(PadMultiplicity);
     event -> SetRhoVariance(RhoVariance);
     event -> SetEventCharge(QEventTot);

     //cloud_ini->Draw();
     //cloud_fil->Draw("SAME");

}
