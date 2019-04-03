#include "FairRootManager.h"

#include "ATEventDrawTaskProto.hh"


#include "TEveManager.h"
#include "TEveGeoShape.h"
#include "TEveTrans.h"
#include "TGeoSphere.h"
#include "TEveTrans.h"
#include "TPaletteAxis.h"
#include "TStyle.h"
#include "TRandom.h"
#include "TColor.h"

#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "TH2Poly.h"
#include "TF1.h"

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

#include <iostream>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"


using namespace std;

ClassImp(ATEventDrawTaskProto);


ATEventDrawTaskProto::ATEventDrawTaskProto():
fHitColor(kPink),
fHitSize(1),
fHitStyle(kFullDotMedium),
fHitSet(0),
fSaveTextData(0),
fhitBoxSet(0)
{
  Char_t padhistname[256];
  fMultiHit=10;

  for(Int_t i=0;i<2015;i++){ 
      sprintf(padhistname,"pad_%d",i);
        fPadAll[i] = new TH1I(padhistname,padhistname,512,0,511);
  }

  Char_t phihistname[256];

          for(Int_t i=0;i<5;i++){
           sprintf(phihistname,"PhiDistr_%d",i);
           fPhiDistr[i] = new TH1D(phihistname,phihistname,180.0,-180.0,180.0);
           if(i==0) fPhiDistr[i]->SetLineColor(kRed);
           else if(i==1) fPhiDistr[i]->SetLineColor(kBlue);
           else if(i==2) fPhiDistr[i]->SetLineColor(kGreen);
           else if(i==3) fPhiDistr[i]->SetLineColor(kCyan);
                 else if(i==4) fPhiDistr[i]->SetLineColor(kMagenta);
                 fPhiDistr[i]->SetLineWidth(2);
                 fPhiDistr[i]->GetYaxis()->SetRangeUser(0., 20.);
          }

          fIsCircularHough=kFALSE;
          fIsLinearHough=kTRUE;
          fIsRawData=kFALSE;
          kIsPRDrawn=kFALSE;
          fHoughLinearFit =new TF1("HoughLinearFit"," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])",0,500);

          for (Int_t i=0;i<4;i++){
						fHoughFit[i] = new TF1(Form("HoughFit%i",i)," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])",0,120);
						fHoughFit[i] -> SetLineColor(kRed);
            fFit[i]      = new TF1(  Form("HoughFit%i",i), "pol1",0,120    );
            fFit[i]      -> SetLineColor(kBlue);

          }

          f3DHitStyle=1;


}

ATEventDrawTaskProto::~ATEventDrawTaskProto()
{

    delete fHoughLinearFit;

}

InitStatus
ATEventDrawTaskProto::Init()
{


  std::cout<<" =====  ATEventDrawTaskProto::Init ====="<<std::endl;

  gROOT->Reset();
  FairRootManager* ioMan = FairRootManager::Instance();
  fEventManager = ATEventManagerProto::Instance();
  fDetmap  =  new AtTpcProtoMap();
  fDetmap -> SetProtoMap(fMap.Data());
  fDetmap -> SetGeoFile("proto20181201_geo_hires.root");
  fDetmap -> SetName("fMap");
  gROOT->GetListOfSpecials()->Add(fDetmap);

  fHitArray = (TClonesArray*) ioMan->GetObject("ATEventH"); // TODO: Why this confusing name? It should be fEventArray
  if(fHitArray) LOG(INFO)<<cGREEN<<"Hit Array Found."<<cNORMAL<<FairLogger::endl;

  fRawEventArray = (TClonesArray*) ioMan->GetObject("ATRawEvent");
  if(fRawEventArray){
       LOG(INFO)<<cGREEN<<"Raw Event Array  Found."<<cNORMAL<<FairLogger::endl;
       fIsRawData=kTRUE;
  }

  fPatternEventArray = (TClonesArray*) ioMan->GetObject("ATPatternEvent");
    if(fPatternEventArray) LOG(INFO)<<cGREEN<<"Pattern Event Array Found."<<cNORMAL<<FairLogger::endl;

  //fHoughSpaceArray =  (TClonesArray*) ioMan->GetObject("ATHough");
  //if(fHoughSpaceArray) LOG(INFO)<<cGREEN<<"Hough Array Found."<<cNORMAL<<FairLogger::endl;


  //fProtoEventArray =  (TClonesArray*) ioMan->GetObject("ATProtoEvent");
  //if(fProtoEventArray) LOG(INFO)<<cGREEN<<"Prototype Event Array Found."<<cNORMAL<<FairLogger::endl;

  fProtoEventAnaArray =  (TClonesArray*) ioMan->GetObject("ATProtoEventAna");
   if(fProtoEventAnaArray) LOG(INFO)<<cGREEN<<"Prototype Event Analysis Array Found."<<cNORMAL<<FairLogger::endl;

  

  //Drawing histograms

  gStyle -> SetPalette(55);
  fCvsPadWave = fEventManager->GetCvsPadWave();
  fCvsPadWave->SetName("fCvsPadWave");
  gROOT->GetListOfSpecials()->Add(fCvsPadWave);
  DrawPadWave();
  fCvsPadPlane = fEventManager->GetCvsPadPlane();// There is a problem if the pad plane is drawn first
  fCvsPadPlane -> ToggleEventStatus();
  fCvsPadPlane->AddExec("ex","ATEventDrawTaskProto::SelectPad(\"fRawEvent\")");
  DrawPadPlane();
  fCvsPadAll = fEventManager->GetCvsPadAll();
  DrawPadAll();
  fCvsMesh = fEventManager->GetCvsMesh();
  DrawMesh();
  /*fCvsQuadrant1 = fEventManager->GetCvsQuadrant1();
  fCvsQuadrant2 = fEventManager->GetCvsQuadrant2();
  fCvsQuadrant3 = fEventManager->GetCvsQuadrant3();
  fCvsQuadrant4 = fEventManager->GetCvsQuadrant4();
  DrawProtoSpace();
  fCvsELQuadrant1 = fEventManager->GetCvsELQuadrant1();
  fCvsELQuadrant2 = fEventManager->GetCvsELQuadrant2();
  fCvsELQuadrant3 = fEventManager->GetCvsELQuadrant3();
  fCvsELQuadrant4 = fEventManager->GetCvsELQuadrant4();
  DrawProtoEL();
  //if(fProtoEventAnaArray) DrawProtoELAna();
  fCvsVertex =  fEventManager->GetCvsVertex();
  fCvsKineAA =  fEventManager->GetCvsKineAA();
  DrawProtoVertex();
  DrawProtoKine();*/
  fCvsAux   =   fEventManager->GetCvsAux();
  DrawProtoAux();



}

void
ATEventDrawTaskProto::Exec(Option_t* option)
{
    Reset();
  
    if(fHitArray)           DrawHitPoints();
    //if(fProtoEventArray)    DrawProtoPattern();
    //if(fHoughSpaceArray)    DrawProtoHough();
    //if(fProtoEventAnaArray) DrawProtoPatternAna();

    gEve -> Redraw3D(kFALSE);

    UpdateCvsPadWave();
    UpdateCvsPadPlane();
    UpdateCvsPadAll();
    UpdateCvsMesh();
    //UpdateCvsProtoQ();
    //UpdateCvsProtoEL();
    //UpdateCvsProtoVertex();
    //UpdateCvsProtoKine();
    UpdateCvsProtoAux();

}

void
ATEventDrawTaskProto::Reset()
{

  if(fHitSet) {
    fHitSet->Reset();
    gEve->RemoveElement(fHitSet, fEventManager);

  }

   if(fhitBoxSet) {
    fhitBoxSet->Reset();
    gEve->RemoveElement(fhitBoxSet, fEventManager);

  }

  if(fEventManager->GetDrawPatternRecognition()){


        if(fPatternEventArray!=NULL & kIsPRDrawn==kTRUE){

                if(fLineNum>0){
                    for(Int_t i=0;i<fLineNum;i++){
                        if(fHitSetPR[i]!=NULL){
                            gEve -> RemoveElement(fHitSetPR[i],fEventManager);
                        }
                    }
                }

        }

        kIsPRDrawn = kFALSE;


  }  

  if(fPadPlane!=NULL)
    fPadPlane->Reset(0);


}

// Filling functions

void
ATEventDrawTaskProto::DrawHitPoints()
{

  Float_t *MeshArray;
  fMesh->Reset(0);
  for(Int_t i=0;i<9;i++) fAuxChannels[i]->Reset(0);
  //f3DHist->Reset(0);
  //TRandom r(0);

    for(Int_t i=0;i<2015;i++)
        fPadAll[i]->Reset(0);


  std::ofstream dumpEvent;
  dumpEvent.open ("event.dat");

  std::vector<Double_t> fPosXMin;
  std::vector<Double_t> fPosYMin;
  std::vector<Double_t> fPosZMin;


  //fQEventHist_H->Reset(0);
  ATEvent* event = (ATEvent*) fHitArray->At(0); // TODO: Why this confusing name? It should be fEventArray
  Int_t nHits=0;

  if(event!=NULL)
  {
    Double_t Qevent=event->GetEventCharge();
    Double_t RhoVariance=event->GetRhoVariance();
    MeshArray = event->GetMesh();
    Int_t eventID=event->GetEventID();
    nHits = event->GetNumHits();
    TString TSevt =" Event ID : ";
    TString TSpad =" Pad ID : ";
    dumpEvent<<TSevt<<eventID<<std::endl;

     /* if(fEventManager->GetEraseQEvent()){
    	 fQEventHist->Reset();
       fRhoVariance->Reset();
      }

      fQEventHist->Fill(Qevent);
      fQEventHist_H->Fill(Qevent);
      fRhoVariance->Fill(RhoVariance);*/

        for(Int_t i=0;i<512;i++){

    		fMesh->SetBinContent(i,MeshArray[i]);

    	}
    }//if(event=!NULL) 

          fHitSet = new TEvePointSet("Hit",nHits, TEvePointSelectorConsumer::kTVT_XYZ);
          fHitSet->SetOwnIds(kTRUE);
          fHitSet->SetMarkerColor(fHitColor);
          fHitSet->SetMarkerSize(fHitSize);
          fHitSet->SetMarkerStyle(fHitStyle);
          std::cout<<cYELLOW<<" Number of hits : "<<nHits<<cNORMAL<<std::endl;


          //////////////////////////////////////////////

          fhitBoxSet = new TEveBoxSet("hitBox");
          fhitBoxSet->Reset(TEveBoxSet::kBT_AABox, kTRUE, 64);

          for(Int_t iHit=0; iHit<nHits; iHit++)
          {

            ATHit hit = event->GetHitArray()->at(iHit);
            Int_t PadNumHit = hit.GetHitPadNum();
            Int_t PadMultHit = event->GetHitPadMult(PadNumHit);
            Double_t BaseCorr = hit.GetBaseCorr();
            Int_t Atbin = -1;



            //if(hit.GetCharge()<fThreshold) continue;
            //if(PadMultHit>fMultiHit) continue;
            TVector3 position = hit.GetPosition();
            TVector3 positioncorr = hit.GetPositionCorr();


            fHitSet->SetMarkerColor(fHitColor);
            fHitSet->SetNextPoint(position.X()/10.,position.Y()/10.,position.Z()/10.); // Convert into cm
            fHitSet->SetPointId(new TNamed(Form("Hit %d",iHit),""));
            Atbin = fPadPlane->Fill(position.X(), position.Y(), hit.GetCharge());

            Bool_t fValidPad;


            
            //if(fSaveTextData)
              //dumpEvent<<position.X()<<" "<<position.Y()<<" "<<position.Z()<<" "<<hit.GetTimeStamp()<<" "<<hit.GetCharge()<<std::endl;

          }
            //////////////////////// Colored Box Drawing ////////////////

            //fPadPlane -> Draw("zcol");
            gPad ->Update();
            fPadPlanePal
            = (TPaletteAxis *) fPadPlane->GetListOfFunctions()->FindObject("palette");





            for(Int_t iHit=0; iHit<nHits; iHit++)
            {

                ATHit hit = event->GetHitArray()->at(iHit);
                TVector3 position = hit.GetPosition();
                TVector3 positioncorr = hit.GetPositionCorr();

                if(f3DHitStyle==0){

                  Float_t HitBoxYDim = hit.GetCharge()*0.001;
                  Float_t HitBoxZDim = 0.05;
                  Float_t HitBoxXDim = 0.05;


                  fhitBoxSet->AddBox(position.X()/10. - HitBoxXDim/2.0, position.Y()/10., position.Z()/10. - HitBoxZDim/2.0,
                  HitBoxXDim,HitBoxYDim,HitBoxZDim); //This coordinates are x,y,z in our system


                }else if(f3DHitStyle==1){

                 Float_t HitBoxYDim = hit.GetCharge()*0.0002;
                 Float_t HitBoxZDim = hit.GetCharge()*0.0002;
                 Float_t HitBoxXDim = hit.GetCharge()*0.0002;



                 fhitBoxSet->AddBox(position.X()/10. - HitBoxXDim/2.0, position.Y()/10. - HitBoxYDim/2.0, position.Z()/10. - HitBoxZDim/2.0,
                          HitBoxXDim,HitBoxYDim,HitBoxZDim); //This coordinates are x,y,z in our system


              }

            Float_t xrgb=255,yrgb=0,zrgb=0;
            if(fPadPlanePal){

                Int_t cHit = fPadPlanePal->GetValueColor(hit.GetCharge());
                TColor *hitBoxColor = gROOT->GetColor(cHit);
                hitBoxColor->GetRGB(xrgb,yrgb,zrgb);

            }

                 fhitBoxSet->DigitColor(xrgb*255,yrgb*255,zrgb*255, 0);

                 dumpEvent<<position.X()<<" "<<position.Y()<<" "<<position.Z()<<" "<<hit.GetTimeStamp()<<" "<<hit.GetCharge()<<std::endl;

            }

             /////////////////////// End of colored box drawing ////////////////////////////

            fhitBoxSet->RefitPlex();
            TEveTrans& tHitBoxPos = fhitBoxSet->RefMainTrans();
            tHitBoxPos.SetPos(0.0, 0.0, 0.0);

           //for(Int_t i=0;i<hitSphereArray.size();i++) gEve->AddElement(hitSphereArray[i]);


  if(fIsRawData){

      fRawevent = (ATRawEvent*) fRawEventArray->At(0);

      if(fRawevent){
          fRawevent->SetName("fRawEvent");
          gROOT->GetListOfSpecials()->Add(fRawevent);

                  Int_t aux_cnt=0;

                  std::vector<ATPad>* PadArray = fRawevent->GetPads();
                  for(Int_t i=0;i<PadArray->size();i++){
                    ATPad Pad = PadArray->at(i);
                      if(Pad.IsAux()){
                        if(aux_cnt<9){
                            std::cout<<cYELLOW<<" Auxiliary Channel "<<aux_cnt<<" - Name "<<Pad.GetAuxName()<<cNORMAL<<"\n";
                            Int_t *rawadc = Pad.GetRawADC();
                            for(Int_t j=0;j<512;j++) fAuxChannels[aux_cnt]->SetBinContent(j,rawadc[j]);
                            aux_cnt++;
                        }else std::cout<<cYELLOW<<" Warning : More auxiliary external channels than expected (max. 9)"<<cNORMAL<<std::endl;
                      }


                  }

       }//if raw event           
  }

    /*if(fIsRawData){
    ATPad *RawPad = fRawevent->GetPad(PadNumHit,fValidPad);
    Double_t *adc = RawPad->GetADC();
        for(Int_t i=0;i<512;i++){

            //f3DThreshold = fEventManager->Get3DThreshold();
            //if(adc[i]>f3DThreshold)
            //f3DHist->Fill(position.X()/10.,position.Y()/10.,i,adc[i]);

          }
    }*/


  if(fIsRawData){

    if(fRawevent!=NULL){

        Int_t nPads = fRawevent->GetNumPads();
        std::cout<<"Num of pads : "<<nPads<<std::endl;

            for(Int_t iPad = 0;iPad<nPads;iPad++){

                ATPad *fPad = fRawevent->GetPad(iPad);
                //std::cout<<"Pad num : "<<iPad<<" Is Valid? : "<<fPad->GetValidPad()<<" Pad num in pad object :"<<fPad->GetPadNum()<<std::endl;

                 if(fPad!=NULL){
                      Int_t *rawadc = fPad->GetRawADC();
                      Double_t *adc = fPad->GetADC();
                      Int_t PadNum_temp = fPad->GetPadNum();
          	          // dumpEvent<<TSpad<<fPad->GetPadNum()<<std::endl;
                      if (fPad->GetValidPad() && PadNum_temp<2015 && PadNum_temp>-1){

                         for(Int_t j=0;j<512;j++){                          
                              fPadAll[PadNum_temp]->SetBinContent(j,adc[j]);
                          }

                      }
                  }    

            }//Pads

      }//NULL      

  }//Rawdata


    if(fEventManager->GetDrawPatternRecognition())
    {

        for(Int_t i=0;i<10;i++) fLineArray[i] = new TEveLine();
        int n = 100;
        double t0 = 0;
        double dt = 2000;
        std::vector<ATTrack> TrackCand;



        if(fPatternEventArray){

            ATPatternEvent* patternEvent = dynamic_cast<ATPatternEvent*> (fPatternEventArray->At(0));

           if(patternEvent!=NULL)
           {
            TrackCand = patternEvent->GetTrackCand();
            for(Int_t i=0;i<10;i++) fHitSetPR[i] = 0;

            fLineNum = TrackCand.size();
            std::cout<<cRED<<" Found "<<TrackCand.size()<<" track candidates "<<cNORMAL<<std::endl;

              if(TrackCand.size()<10){
                    for(Int_t i=0;i<TrackCand.size();i++)
                    {

                        ATTrack track = TrackCand.at(i);
                        std::vector<ATHit>* trackHits =  track.GetHitArray();
                        int nHitsMin = trackHits->size();

                        fHitSetPR[i] = new TEvePointSet(Form("HitPR_%d",i),nHitsMin, TEvePointSelectorConsumer::kTVT_XYZ);
                        if(track.GetIsNoise()) fHitSetPR[i]->SetMarkerColor(kRed);
                        else fHitSetPR[i]->SetMarkerColor(GetTrackColor(i)+1);
                        fHitSetPR[i]->SetMarkerSize(fHitSize);
                        fHitSetPR[i]->SetMarkerStyle(fHitStyle);


                          for(int j =0;j<trackHits->size();++j){
                            TVector3 position = trackHits->at(j).GetPosition();
                            fHitSetPR[i]->SetNextPoint(position.X()/10.,position.Y()/10.,position.Z()/10.);

                          }

                      if(fEventManager->GetDrawPatternRecognition()) gEve -> AddElement(fHitSetPR[i]);
                      kIsPRDrawn=kTRUE;

                    }
               }
            }
      }    


    } 


    if(!fEventManager->GetDrawPatternRecognition()){

      gEve -> AddElement(fHitSet);
      gEve -> AddElement(fhitBoxSet);

     }else if(fEventManager->GetDrawPatternRecognition()){
     

       //if(fPatternEventArray)
            // if(fLineNum>0) for(Int_t i=0;i<fLineNum;i++) gEve -> AddElement(fHitSetPR[i]);

     }   

     dumpEvent.close();      

}

void
ATEventDrawTaskProto::DrawProtoPattern()
{

    for(Int_t i=0;i<4;i++){
       fQHitPattern[i]  -> Set(0);
       fQELossPattern[i]-> Set(0);
     }
    ATProtoEvent* protoevent = (ATProtoEvent*) fProtoEventArray->At(0);
    Int_t nQuads = protoevent->GetNumQuadrants();
    std::vector<ATProtoQuadrant> quadrantArray;

    //fHoughSpaceLine  = dynamic_cast<ATHoughSpaceLine*> (fHoughSpaceArray->At(0));
    //std::vector<std::pair<Double_t,Double_t>> HoughPar = fHoughSpaceLine->GetHoughPar();

   if(nQuads<5){
    for(Int_t iQ=0; iQ<nQuads; iQ++)
 	  {

	   ATProtoQuadrant quadrant = protoevent->GetQuadrantArray()->at(iQ);
	   quadrantArray.push_back(protoevent->GetQuadrantArray()->at(iQ));
      std::vector<Double_t> *PhiArray =quadrantArray[iQ].GetPhiArray();

        for(Int_t pval=0;pval<PhiArray->size();pval++){
            fPhiDistr[iQ]->Fill(PhiArray->at(pval));
			   }
	          PhiArray->clear();

              Int_t qNumHit = quadrant.GetNumHits();

                  for(Int_t j=0;j<qNumHit;j++){

                    ATHit* qhit = quadrant.GetHit(j);
                    TVector3 position = qhit->GetPosition();
                    TVector3 positionCorr = qhit->GetPositionCorr();
                    Double_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2) );
                    //fQHitPattern[iQ]   ->SetPoint(fQHitPattern[iQ]->GetN(),radius,position.Z());//
                    fQHitPattern[iQ]   ->SetPoint(fQHitPattern[iQ]->GetN(),radius,positionCorr.Z());
                    fQELossPattern[iQ] ->SetPoint(fQELossPattern[iQ]->GetN(),radius,qhit->GetCharge());

                  }


  	   }
    }



}

void
ATEventDrawTaskProto::DrawProtoHough()
{

  fHoughSpaceLine  = dynamic_cast<ATHoughSpaceLine*> (fHoughSpaceArray->At(0));
  std::vector<std::pair<Double_t,Double_t>> HoughPar = fHoughSpaceLine->GetHoughPar();

      std::vector<Double_t> par0_fit;
      std::vector<Double_t> par1_fit;
      std::vector<Double_t> Angle;
      std::vector<Double_t> Angle_fit;

      Double_t tHoughPar0[4]={0};
		  Double_t tHoughPar1[4]={0};
	   	Double_t tFitPar0[4]={0};
	  	Double_t tFitPar1[4]={0};
	  	Double_t tAngleHough[4]={0};
		  Double_t tAngleFit[4]={0};

      for(Int_t i=0;i<HoughPar.size();i++){

           Angle.push_back(180-HoughPar.at(i).first*180/TMath::Pi());

           tHoughPar0[i]=HoughPar.at(i).first;
           tHoughPar1[i]=HoughPar.at(i).second;
           tAngleHough[i]=180-HoughPar.at(i).first*180/TMath::Pi();

           fHoughFit[i]->SetParameter(0,HoughPar.at(i).first);
					 fHoughFit[i]->SetParameter(1,HoughPar.at(i).second);

      }


}

void
ATEventDrawTaskProto::DrawProtoPatternAna()
{

    for(Int_t i=0;i<4;i++) fQELossPatternAna[i]-> Set(0);

    fQVertex[2]->Reset(0);
    fQVertex[3]->Reset(0);
    fQKine[2]->Reset(0);
    fQKine[3]->Reset(0);

      ATProtoEventAna* protoeventAna = (ATProtoEventAna*) fProtoEventAnaArray->At(0);

      std::vector<Double_t>* Par0 = protoeventAna->GetPar0();
      std::vector<Double_t>* Par1 = protoeventAna->GetPar1();

      std::vector<Double_t>* Range = protoeventAna->GetRange();


      //std::vector<std::pair<Double_t,Double_t>>* ELossHitPattern = protoeventAna->GetELossHitPattern();
      std::vector<std::vector<std::pair<Double_t,Double_t>>>* QELossHitPattern = protoeventAna->GetQELossHitPattern();

      for(Int_t i=0;i<QELossHitPattern->size();i++){
                  std::vector<std::pair<Double_t,Double_t>> ELossHitPattern = QELossHitPattern->at(i);
                  fFit[i]->SetParameter(0,Par0->at(i));
                  fFit[i]->SetParameter(1,Par1->at(i));

                for(Int_t j=0;j<ELossHitPattern.size();j++){
                  std::pair<Double_t,Double_t> HPbuffer = ELossHitPattern.at(j);
                  Double_t radius = HPbuffer.second;
                  Double_t charge = HPbuffer.first;
                  fQELossPatternAna[i] ->SetPoint(fQELossPatternAna[i]->GetN(),radius,charge);
                }


      }

      std::vector<Double_t>* vertex = protoeventAna->GetVertex();
      std::vector<Double_t>* KineAA = protoeventAna->GetAngleFit();
      std::vector<Double_t>* Chi2   = protoeventAna->GetChi2();
      std::vector<Int_t>* NDF       = protoeventAna->GetNDF();
      fQVertex[0]->Fill(vertex->at(0),vertex->at(2));
      fQVertex[1]->Fill(vertex->at(1),vertex->at(3));
      fQVertex[2]->Fill(vertex->at(0),vertex->at(2));
      fQVertex[3]->Fill(vertex->at(1),vertex->at(3));

      fQKine[0]->Fill(KineAA->at(0),KineAA->at(2));
      fQKine[1]->Fill(KineAA->at(1),KineAA->at(3));
      fQKine[2]->Fill(KineAA->at(0),KineAA->at(2));
      fQKine[3]->Fill(KineAA->at(1),KineAA->at(3));




      std::cout<<cYELLOW<<" ==================================================================== "<<std::endl;
      std::cout<<"                                  ATEventDrawTask : Fit Results                     "<<std::endl;
      std::cout<<"       -   Quadrant 0   -   Quadrant 1   -   Quadrant 2  -   Quadrant 3      "<<std::endl;
      std::cout<<"  Angle  : "<<KineAA->at(0)<<"              "<<KineAA->at(1)<<"              "<<KineAA->at(2)<<"                "<<KineAA->at(3)<<std::endl;
      std::cout<<"  Vertex : "<<vertex->at(0)<<"              "<<vertex->at(1)<<"              "<<vertex->at(2)<<"                "<<vertex->at(3)<<std::endl;
      std::cout<<"  Chi2   : "<<Chi2->at(0)  <<"              "<<Chi2->at(1  )<<"              "<<Chi2->at(2)  <<"                "<<Chi2->at(3)<<std::endl;
      std::cout<<"  NDF    : "<<NDF->at(0)   <<"              "<<NDF->at(1)   <<"              "<<NDF->at(2)   <<"                "<<NDF->at(3)<<std::endl;
      std::cout<<" ==================================================================== "<<cNORMAL<<std::endl;
      std::cout<<std::endl;


}

// Draw functions ////

void
ATEventDrawTaskProto::DrawPadWave()
{


        fPadWave = new TH1I("fPadWave","fPadWave",512,0,511);
        gROOT->GetListOfSpecials()->Add(fPadWave);
        fCvsPadWave->cd();
        fPadWave -> Draw();
}

void
ATEventDrawTaskProto::DrawPadPlane()
{

  /*if(fPadPlane)
  {
    fPadPlane->Reset(0);
    return;
  }*/

    fPadPlane = fDetmap->GetATTPCPlane("ATTPC_Proto");
    fCvsPadPlane -> cd();
    //fPadPlane -> Draw("zcol");
    //fPadPlane -> Draw("COL L0");
    fPadPlane -> Draw("COL L");
    fPadPlane -> SetMinimum(1.0);
    gStyle->SetOptStat(0);
    gStyle->SetPalette(103);
    gPad ->Update();




}

void
ATEventDrawTaskProto::DrawPadAll()
{
    
    fCvsPadAll->cd();
    
    for(Int_t i=0;i<2015;i++){
        //fPadAll[i]->Reset(0);
        //fPadAll[i] = new TH1I("fPadAll","fPadAll",512,0,511);
        fPadAll[i]->GetYaxis()->SetRangeUser(0,2500);
        fPadAll[i]->SetLineColor(8);

        /*if (i<64) fPadAll[i]->SetLineColor(6);                         // Q1, pink
        else if(i>=64 && i<127) fPadAll[i]->SetLineColor(8);           // Q2, green
        else if(i>=127 && i<190) fPadAll[i]->SetLineColor(7);           // Q3, blue
        else if(i>=190 && i<253) fPadAll[i]->SetLineColor(kOrange-3);   // Q4, orange
        else fPadAll[i]->SetLineColor(0);                              //white for non physical pads*/

        fPadAll[i] -> Draw("SAME");
        
    }
    
    
}

void
ATEventDrawTaskProto::DrawMesh()
{

    fCvsMesh->cd();
    fMesh = new TH1F("Mesh","Mesh",512,0,511);
    fMesh -> Draw();

}

void
ATEventDrawTaskProto::DrawProtoSpace()
{

      for(Int_t i=0;i<4;i++){
        fQHitPattern[i] = new TGraph();
        fQHitPattern[i]->SetMarkerStyle(22);
				fQHitPattern[i]->SetMarkerSize(0.7);
        fQHitPattern[i]->SetPoint(1,0,0);
        if(i==0) {
          fCvsQuadrant1->cd();
          fQHitPattern[0]->Draw("A*");
          if(fHoughSpaceArray){
              fHoughFit[0]->Draw("SAME");
              if(fProtoEventAnaArray) fFit[0]->Draw("SAME");
            }
        }else if(i==1){
          fCvsQuadrant2->cd();
          fQHitPattern[1]->Draw("A*");
          if(fHoughSpaceArray){
             fHoughFit[1]->Draw("SAME");
             if(fProtoEventAnaArray) fFit[1]->Draw("SAME");
             }
        }else if(i==2) {
            fCvsQuadrant3->cd();
            fQHitPattern[2]->Draw("A*");
            if(fHoughSpaceArray){
               fHoughFit[2]->Draw("SAME");
               if(fProtoEventAnaArray) fFit[2]->Draw("SAME");
             }
        }else if(i==3){
            fCvsQuadrant4->cd();
            fQHitPattern[3]->Draw("A*");
            if(fHoughSpaceArray){
               fHoughFit[3]->Draw("SAME");
               if(fProtoEventAnaArray) fFit[3]->Draw("SAME");
             }
        }
      }


}

void
ATEventDrawTaskProto::DrawProtoEL()
{

  for(Int_t i=0;i<4;i++){
    fQELossPattern[i] = new TGraph();
    fQELossPattern[i]->SetMarkerStyle(22);
    fQELossPattern[i]->SetMarkerSize(0.7);
    fQELossPattern[i]->SetPoint(1,0,0);
    fQELossPatternAna[i] = new TGraph();
    fQELossPatternAna[i]->SetMarkerStyle(20);
    fQELossPatternAna[i]->SetMarkerColor(kRed);
    fQELossPatternAna[i]->SetMarkerSize(0.7);
    fQELossPatternAna[i]->SetPoint(1,0,0);
    if(i==0) {
      fCvsELQuadrant1->cd();
      fQELossPattern[0]->Draw("AP");
      if(fProtoEventAnaArray) fQELossPatternAna[0]->Draw("P");
    }else if(i==1){
      fCvsELQuadrant2->cd();
      fQELossPattern[1]->Draw("AP");
      if(fProtoEventAnaArray) fQELossPatternAna[1]->Draw("P");
    }else if(i==2) {
      fCvsELQuadrant3->cd();
      fQELossPattern[2]->Draw("AP");
      if(fProtoEventAnaArray) fQELossPatternAna[2]->Draw("P");
    }else if(i==3){
      fCvsELQuadrant4->cd();
      fQELossPattern[3]->Draw("AP");
      if(fProtoEventAnaArray) fQELossPatternAna[3]->Draw("P");

    }
  }

}

void
ATEventDrawTaskProto::DrawProtoELAna()
{

  for(Int_t i=0;i<4;i++){
    fQELossPatternAna[i] = new TGraph();
    fQELossPatternAna[i]->SetMarkerStyle(22);
    fQELossPatternAna[i]->SetMarkerSize(0.7);
    fQELossPatternAna[i]->SetMarkerStyle(kRed);
    fQELossPatternAna[i]->SetPoint(1,0,0);
    if(i==0) {
      fCvsELQuadrant1->cd();
      fQELossPatternAna[0]->Draw("A*");
    }else if(i==1){
      fCvsELQuadrant2->cd();
      fQELossPatternAna[1]->Draw("A*");
    }else if(i==2) {
        fCvsELQuadrant3->cd();
        fQELossPatternAna[2]->Draw("A*");
    }else if(i==3){
        fCvsELQuadrant4->cd();
        fQELossPatternAna[3]->Draw("A*");

    }
  }

}

void
ATEventDrawTaskProto::DrawProtoVertex()
{

  for(Int_t i=0;i<4;i++){
    fQVertex[i]  = new TH2F(Form("Vertex_%i",i),Form("Vertex%i",i),1000,0,1000,1000,0,1000);
    fQVertex[i]->SetMarkerSize(1.2);
    fQVertex[i]->SetMarkerStyle(22);
    if(i==0) fQVertex[i]->SetMarkerColor(kRed);
    else if(i==1) {fQVertex[i]->SetMarkerColor(kRed);fQVertex[i]->SetMarkerStyle(20);}
    else if(i==2) fQVertex[i]->SetMarkerColor(kBlue);
    else if(i==3) {fQVertex[i]->SetMarkerColor(kBlue);fQVertex[i]->SetMarkerStyle(20);}
  }

    fCvsVertex->cd();
    fQVertex[0]->Draw();
    fQVertex[1]->Draw("SAME");
    fQVertex[2]->Draw("SAME");
    fQVertex[3]->Draw("SAME");





}

void
ATEventDrawTaskProto::DrawProtoKine()
{

  for(Int_t i=0;i<4;i++){
    fQKine[i]  = new TH2F(Form("Angle_Angle_Kinematics_%i",i),Form("Angle_Angle_Kinematics%i",i),1000,0,180,1000,0,180);
    fQKine[i]->SetMarkerSize(1.2);
    fQKine[i]->SetMarkerStyle(22);
    if(i==0) fQKine[i]->SetMarkerColor(kRed);
    else if(i==1) {fQKine[i]->SetMarkerColor(kRed);fQKine[i]->SetMarkerStyle(20);}
    else if(i==2) fQKine[i]->SetMarkerColor(kBlue);
    else if(i==3) {fQKine[i]->SetMarkerColor(kBlue);fQKine[i]->SetMarkerStyle(20);}
  }

    fCvsKineAA->cd();
    fQKine[0]->Draw();
    fQKine[1]->Draw("SAME");
    fQKine[2]->Draw("SAME");
    fQKine[3]->Draw("SAME");


}

void ATEventDrawTaskProto::DrawProtoAux()
{

    fCvsAux->Divide(3,3);
    for(Int_t i=0;i<9;i++){
      fAuxChannels[i] = new TH1F(Form("Auxiliary_Channel_%i",i),Form("AuxChannel%i",i),512,0,511);
      fCvsAux->cd(1+i);
      fAuxChannels[i]->Draw();
    }





}

/// Update functions //////

void
ATEventDrawTaskProto::UpdateCvsPadWave()
{
    fCvsPadWave -> Modified();
    fCvsPadWave -> Update();


}

void
ATEventDrawTaskProto::UpdateCvsPadPlane()
{
  fCvsPadPlane -> Modified();
  fCvsPadPlane -> Update();

}

void
ATEventDrawTaskProto::UpdateCvsPadAll()
{
    fCvsPadAll -> Modified();
    fCvsPadAll -> Update();

}

void
ATEventDrawTaskProto::UpdateCvsMesh()
{

    fCvsMesh -> Modified();
    fCvsMesh -> Update();


}

void
ATEventDrawTaskProto::UpdateCvsProtoQ(){

  fCvsQuadrant1->Modified();
  fCvsQuadrant1->Update();
  fCvsQuadrant2->Modified();
  fCvsQuadrant2->Update();
  fCvsQuadrant3->Modified();
  fCvsQuadrant3->Update();
  fCvsQuadrant4->Modified();
  fCvsQuadrant4->Update();


}

void
ATEventDrawTaskProto::UpdateCvsProtoEL(){

  fCvsELQuadrant1->Modified();
  fCvsELQuadrant1->Update();
  fCvsELQuadrant2->Modified();
  fCvsELQuadrant2->Update();
  fCvsELQuadrant3->Modified();
  fCvsELQuadrant3->Update();
  fCvsELQuadrant4->Modified();
  fCvsELQuadrant4->Update();


}

void
ATEventDrawTaskProto::UpdateCvsProtoVertex(){

    fCvsVertex->Modified();
    fCvsVertex->Update();

}

void
ATEventDrawTaskProto::UpdateCvsProtoKine(){

    fCvsKineAA->Modified();
    fCvsKineAA->Update();

}

void
ATEventDrawTaskProto::UpdateCvsProtoAux(){

   //for(Int_t i = 0;i<4;i++){
    //fCvsAux->cd(1);
    TPad* Pad_1 = (TPad*)fCvsAux->GetPad(1);
    Pad_1->Modified();
    Pad_1->Update();
    TPad* Pad_2 = (TPad*)fCvsAux->GetPad(2);
    Pad_2->Modified();
    Pad_2->Update();
    TPad* Pad_3 = (TPad*)fCvsAux->GetPad(3);
    Pad_3->Modified();
    Pad_3->Update();
    TPad* Pad_4 = (TPad*)fCvsAux->GetPad(4);
    Pad_4->Modified();
    Pad_4->Update();
    TPad* Pad_5 = (TPad*)fCvsAux->GetPad(5);
    Pad_5->Modified();
    Pad_5->Update();
    TPad* Pad_6 = (TPad*)fCvsAux->GetPad(6);
    Pad_6->Modified();
    Pad_6->Update();
    TPad* Pad_7 = (TPad*)fCvsAux->GetPad(7);
    Pad_7->Modified();
    Pad_7->Update();
    TPad* Pad_8 = (TPad*)fCvsAux->GetPad(8);
    Pad_8->Modified();
    Pad_8->Update();
    TPad* Pad_9 = (TPad*)fCvsAux->GetPad(9);
    Pad_9->Modified();
    Pad_9->Update();
    fCvsAux->Modified();
    fCvsAux->Update();
  //}

}



void
ATEventDrawTaskProto::SelectPad(const char *rawevt)
{
    int event = gPad->GetEvent();
    if (event != 11) return; //may be comment this line
    TObject *select = gPad->GetSelected();
    if (!select) return;
    if (select->InheritsFrom(TH2Poly::Class())) {
        TH2Poly *h = (TH2Poly*)select;
        gPad->GetCanvas()->FeedbackMode(kTRUE);
         ATRawEvent* tRawEvent = NULL;
         tRawEvent = (ATRawEvent*)gROOT->GetListOfSpecials()->FindObject(rawevt);
         if(tRawEvent == NULL){
		std::cout<<" = ATEventDrawTaskProto::SelectPad NULL pointer for the ATRawEvent! Please select an event first "<<std::endl;
		return;
	}

        int pyold = gPad->GetUniqueID();
        int px = gPad->GetEventX();
        int py = gPad->GetEventY();
        float uxmin = gPad->GetUxmin();
        float uxmax = gPad->GetUxmax();
        int pxmin = gPad->XtoAbsPixel(uxmin);
        int pxmax = gPad->XtoAbsPixel(uxmax);
        if(pyold) gVirtualX->DrawLine(pxmin,pyold,pxmax,pyold);
        gVirtualX->DrawLine(pxmin,py,pxmax,py);
        gPad->SetUniqueID(py);
        Float_t upx = gPad->AbsPixeltoX(px);
        Float_t upy = gPad->AbsPixeltoY(py);
        Double_t x = gPad->PadtoX(upx);
        Double_t y = gPad->PadtoY(upy);
        Int_t bin = h->FindBin(x,y);
        const char *bin_name = h->GetBinName(bin);
        //std::cout<<" X : "<<x<<"  Y: "<<y<<std::endl;
        //std::cout<<bin_name<<std::endl;
        std::cout<<" =========================="<<std::endl;
        std::cout<<" Bin number selected : "<<bin<<" Bin name :"<<bin_name<<std::endl;
        Bool_t IsValid = kFALSE;

        AtTpcMap *tmap = NULL;
        tmap = (AtTpcMap*)gROOT->GetListOfSpecials()->FindObject("fMap");
        //new AtTpcProtoMap();
        //TString map = "/Users/yassidayyad/fair_install/ATTPCROOT_v2_06042015/scripts/proto.map";
        //tmap->SetProtoMap(map.Data());
        Int_t tPadNum =tmap->BinToPad(bin);
        std::cout<<" Bin : "<<bin<<" to Pad : "<<tPadNum<<std::endl;
        ATPad *tPad = tRawEvent->GetPad(tPadNum,IsValid);
        std::cout<<" Event ID (Select Pad) : "<<tRawEvent->GetEventID()<<std::endl;
        std::cout<<" Raw Event Pad Num "<<tPad->GetPadNum()<<" Is Valid? : "<<IsValid<<std::endl;
        std::cout<<std::endl;
        //TH1D* tPadWaveSub = NULL;
        //tPadWaveSub = new TH1D("tPadWaveSub","tPadWaveSub",512.0,0.0,511.0);
        //tPadWaveSub->SetLineColor(kRed);
        TH1I* tPadWave = NULL;
        tPadWave = (TH1I*)gROOT->GetListOfSpecials()->FindObject("fPadWave");
        Int_t *rawadc = tPad->GetRawADC();
        Double_t *adc = tPad->GetADC();
        if(tPadWave == NULL){
            std::cout<<" = ATEventDrawTask::SelectPad NULL pointer for the TH1I! Please select an event first "<<std::endl;
            return;
	     }
         tPadWave->Reset();
         //tPadWaveSub->Reset();
        for(Int_t i=0;i<512;i++){

			      // tPadWave->SetBinContent(i,rawadc[i]);
                   tPadWave->SetBinContent(i,adc[i]);
         		   //tPadWaveSub->SetBinContent(i,adc[i]);

		    }



        TCanvas *tCvsPadWave = NULL;
        tCvsPadWave = (TCanvas*)gROOT->GetListOfSpecials()->FindObject("fCvsPadWave");
        if(tCvsPadWave == NULL){
            std::cout<<" = ATEventDrawTask::SelectPad NULL pointer for the TCanvas! Please select an event first "<<std::endl;
            return;
        }
        tCvsPadWave->cd();
        tPadWave->Draw();
        //tPadWaveSub->Draw("SAME");
        tCvsPadWave->Update();
    }


}

void
ATEventDrawTaskProto::SetHitAttributes(Color_t color, Size_t size, Style_t style)
{
  fHitColor = color;
  fHitSize = size;
  fHitStyle = style;
}

void
ATEventDrawTaskProto::Set3DHitStyleBar() {f3DHitStyle=0;}

void
ATEventDrawTaskProto::Set3DHitStyleBox() {f3DHitStyle=1;}

EColor ATEventDrawTaskProto::GetTrackColor(int i)
{
   std::vector<EColor> colors = {kAzure,kOrange,kViolet,kTeal,kMagenta,kBlue,kViolet,kYellow,kCyan,kAzure};
   if(i<10){
     return colors.at(i);
   }else return kAzure;

}
