/*********************************************************************
 *   AtTPC Unpacker and Decoder Core Class	AtCore               *
 *   Author: Y. Ayyad            				     *
 *   Log: 04-03-2015 17:16 JST					     *
 *   Adapted from STCore from SPiRITROOT by G. Jhang		     *
 *********************************************************************/

#include <iostream>
#include <fstream>
#include <cmath>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "AtCore.h"
#include "GETDecoder.h"
#include "GETFrame.h"

ClassImp(AtCore);

AtCore::AtCore() : AtPadCoordArr(boost::extents[10240][3][2]), kOpt(0)
{
   Initialize();
   kDebug = kFALSE;
}

AtCore::AtCore(Int_t opt) : AtPadCoordArr(boost::extents[10240][3][2]), kOpt(0)
{
   kOpt = opt;
   Initialize();
   kDebug = kFALSE;
}

AtCore::AtCore(TString filename, Int_t numTbs) : AtPadCoordArr(boost::extents[10240][3][2]), kOpt(0)
{

   Initialize();
   AddData(filename);
   SetNumTbs(numTbs);
   kDebug = kFALSE;
}

AtCore::~AtCore()
{
   delete fGETDecoderPtr;
   delete fAtMapPtr;
   // delete fAtPadCoordPtr;
   // delete fPedestalPtr;
   // delete fMapPtr;
}

void AtCore::Initialize()
{

   fGETDecoderPtr = new GETDecoder();

   fIsData = kFALSE;
   if (kOpt == 0)
      fAtMapPtr = new AtTpcMap();
   else if (kOpt == 1)
      fAtMapPtr = new AtTpcProtoMap();
   else
      std::cout << "== AtCore Initialization Error : Option not found. Current available options: AtTPC Map 0 / "
                   "Prototype Map 1"
                << std::endl;

   fRawEventPtr = NULL;
   // fAtPadCoordPtr = NULL;

   fIsData = kFALSE;

   fIsProtoGeoSet = kFALSE;
   fIsProtoMapSet = kFALSE;
   fIsInternalPedestal = kFALSE;
   fPedestalMode = kNoPedestal;
   fPedestalRMSFactor = 0;
   fIsFPNPedestal = kFALSE;
   fFPNSigmaThreshold = 5;

   fNumTbs = 512;
   fStartTb = 3;
   fAverageTbs = 20;
}

Bool_t AtCore::AddData(TString filename)
{
   return fGETDecoderPtr->AddData(filename);
}

Bool_t AtCore::SetData(Int_t value)
{
   fIsData = fGETDecoderPtr->SetData(value);

   fPrevEventNo = -1;
   fCurrEventNo = -1;
   fCurrFrameNo = 0;

   return fIsData;
}

Bool_t AtCore::SetAtTpcMap(Char_t const *lookup)
{

   if (kOpt == 0) {
      dynamic_cast<AtTpcMap *>(fAtMapPtr)->GenerateAtTpc();
   }

   Bool_t MapIn = fAtMapPtr->ParseXMLMap(lookup);
   if (!MapIn)
      return false;

   // AtPadCoordArr = fAtMapPtr->GetPadCoordArr();//TODO Use a pointer to a simpler container
   //**** For debugging purposes only! ******//
   // fAtMapPtr->SetGUIMode();
   // fAtMapPtr->GetAtTPCPlane();
   return true;
}

Bool_t AtCore::SetProtoGeoFile(TString geofile)
{

   if (kOpt == 1) {

      fIsProtoGeoSet = dynamic_cast<AtTpcProtoMap *>(fAtMapPtr)->SetGeoFile(geofile);
      return fIsProtoGeoSet;

   } else {
      std::cout << "== AtCore::SetProtoGeoMap. This method must be used only with Prototype mapping (kOpt=1)!"
                << std::endl;
      return kFALSE;
   }
}

Bool_t AtCore::SetProtoMapFile(TString mapfile)
{

   if (kOpt == 1) {

      fIsProtoMapSet = dynamic_cast<AtTpcProtoMap *>(fAtMapPtr)->SetProtoMap(mapfile);
      return fIsProtoMapSet;

   } else {
      std::cout << "== AtCore::SetProtoMapFile. This method must be used only with Prototype mapping (kOpt=1)!"
                << std::endl;
      return kFALSE;
   }
}

void AtCore::SetPositivePolarity(Bool_t value)
{
   fGETDecoderPtr->SetPositivePolarity(value);
}

void AtCore::SetNumTbs(Int_t value)
{
   fNumTbs = value;
   fGETDecoderPtr->SetNumTbs(value);
}

void AtCore::SetInternalPedestal(Int_t startTb, Int_t averageTbs)
{
   /* if (fIsPedestalData) {// TODO Pedestal Data mode
      fPedestalMode = kPedestalBothIE;
      std::cout << "== AtCore Using both pedestal data is set!" << std::endl;
    } else {*/
   fPedestalMode = kPedestalInternal;
   std::cout << "== AtCore Internal pedestal calculation will be done!" << std::endl;
   //}

   fIsInternalPedestal = kTRUE;
   fStartTb = startTb;
   fAverageTbs = averageTbs;
}

void AtCore::SetFPNPedestal(Double_t sigmaThreshold)
{
   fIsFPNPedestal = kTRUE;
   fPedestalMode = kPedestalFPN;
   fFPNSigmaThreshold = sigmaThreshold;

   std::cout << "== AtCore Using FPN pedestal is set!" << std::endl;
}

AtRawEvent *AtCore::GetRawEvent(Int_t eventID)
{

   // TH2I *wave = new TH2I("","",612,-100,511,1000,1,4000);
   // TCanvas *c = new TCanvas();

   if (kDebug)
      fAtMapPtr->SetDebugMode();

   if (!fIsData) {
      std::cout << "== AtCore -  Data file is not set" << std::endl;
      return NULL;
   }

   fPrevEventNo = eventID;

   if (fRawEventPtr != NULL)
      delete fRawEventPtr;

   fRawEventPtr = new AtRawEvent();
   GETFrame *frame = NULL;

   while ((frame = fGETDecoderPtr->GetFrame(fCurrFrameNo))) {
      if (fPrevEventNo == -1)
         fPrevEventNo = frame->GetEventID();

      fCurrEventNo = frame->GetEventID();

      if (fCurrEventNo == fPrevEventNo + 1) {
         fPrevEventNo = fCurrEventNo;
         return fRawEventPtr;
      } else if (fCurrEventNo > fPrevEventNo + 1) {
         fCurrFrameNo = 0;
         continue;
      } else if (fCurrEventNo < fPrevEventNo) {
         fCurrFrameNo++;
         continue;
      }

      Int_t frameType = fGETDecoderPtr->GetFrameType();

      if ((frameType == GETDecoder::kMergedID || frameType == GETDecoder::kMergedTime) && !(fRawEventPtr->IsGood())) {
         Int_t currentInnerFrameID = fGETDecoderPtr->GetCurrentInnerFrameID();
         Int_t numInnerFrames = fGETDecoderPtr->GetNumMergedFrames();

         while (!(currentInnerFrameID + 1 == numInnerFrames)) {
            fGETDecoderPtr->GetFrame(fCurrFrameNo);

            currentInnerFrameID = fGETDecoderPtr->GetCurrentInnerFrameID();
            numInnerFrames = fGETDecoderPtr->GetNumMergedFrames();
         }

         fCurrFrameNo++;
         fPrevEventNo = fCurrEventNo;
         return fRawEventPtr;
      }

      fRawEventPtr->SetEventID(fCurrEventNo);

      eventID = frame->GetEventID();
      Int_t iCobo = frame->GetCoboID();
      Int_t iAsad = frame->GetAsadID();

      // std::cout<<" Event ID : "<<eventID<<" coboID : "<<iCobo<<" asadID : "<<iAsad<<'\xd';//std::endl;
      // std::cout<<" Event ID : "<<eventID<<" coboID : "<<iCobo<<" asadID : "<<iAsad<<std::endl;

      // #pragma omp parallel for
      for (Int_t iAget = 0; iAget < 4; iAget++) {
         for (Int_t iCh = 0; iCh < 68; iCh++) {
            // std::cout<<std::endl;
            // std::cout<<" Event ID : "<<eventID<<" coboID : "<<iCobo<<" asadID : "<<iAsad<<std::endl;
            // std::cout<<" AgetID : "<<iAget<<" ChannelID : "<<iCh<<std::endl;

            // fAtMapPtr->Dump();

            PadReference PadRef = {.cobo = iCobo, .asad = iAsad, .aget = iAget, .ch = iCh};
            Int_t PadRefNum = fAtMapPtr->GetPadNum(PadRef);
            std::vector<Float_t> PadCenterCoord;
            PadCenterCoord.reserve(2);
            PadCenterCoord = fAtMapPtr->CalcPadCenter(PadRefNum);

            auto &pad = fRawEventPtr->AddPad(PadRefNum);

            pad.SetPadXCoord(PadCenterCoord[0]);
            pad.SetPadYCoord(PadCenterCoord[1]);
            if (PadRefNum == -1)
               pad.SetValidPad(kFALSE);
            else
               pad.SetValidPad(kTRUE);
            // else continue;

            Int_t *rawadc = frame->GetRawADC(iAget, iCh);

            for (Int_t iTb = 0; iTb < fGETDecoderPtr->GetNumTbs(); iTb++) {
               // if(*rawadc>0) std::cout<<" AGet "<<iAget<<" Channel : "<<iCh<<" ADC : "<<*rawadc<<" Time Bucket :
               // "<<iTb<<std::endl;
               pad.SetRawADC(iTb, rawadc[iTb]);
               // wave-> Fill(iTb, rawadc[iTb]);
            }

            // wave->Draw("col");

            Int_t signalDelay = 0;
            /* if (fIsSignalDelayData) { //TODO Implement in the future
                  Int_t uaIdx = fMapPtr -> GetUAIdx(coboID, asadID);
               signalDelay = ceil(fSignalDelayPtr -> GetSignalDelay(uaIdx));
             }*/

            if (fPedestalMode == kPedestalInternal) { // TODO Obsolete, FPN does both
               frame->CalcPedestal(iAget, iCh, fStartTb, fAverageTbs);
               frame->SubtractPedestal(iAget, iCh);

               Double_t *adc = frame->GetADC(iAget, iCh);

               for (Int_t iTb = 0; iTb < fGETDecoderPtr->GetNumTbs(); iTb++) {
                  Int_t delayedTb = iTb - signalDelay;
                  Double_t delayedADC = 0;
                  /*if (delayedTb < 0 || delayedTb >= fGETDecoderPtr -> GetNumTbs())//TODO
                     delayedADC = 0;
                  else
                     delayedADC = adc[delayedTb];*/

                  // pad -> SetADC(iTb, delayedADC);

                  pad.SetADC(iTb, adc[iTb]); // TODO For the moment we fill it without delay
               }

               Int_t maxADCIdx = frame->GetMaxADCIdx(iAget, iCh) + signalDelay;
               if (maxADCIdx < 0 || maxADCIdx >= fGETDecoderPtr->GetNumTbs())
                  maxADCIdx = 0;

               pad.SetPedestalSubtracted(kTRUE);

            } else if (fPedestalMode != kNoPedestal) {
               /* if (fPedestalMode == kPedestalBothIE)//TODO
                   frame -> CalcPedestal(iAget, iCh, fStartTb, fAverageTbs);*/

               Double_t pedestal[512];
               Double_t pedestalSigma[512];

               if (fPedestalMode == kPedestalExternal || fPedestalMode == kPedestalBothIE) {
                  // fPedestalPtr -> GetPedestal(row, layer, pedestal, pedestalSigma); //TODO
                  // frame -> SetPedestal(iAget, iCh, pedestal, pedestalSigma);
               } else if (fPedestalMode == kPedestalFPN)
                  frame->SetFPNPedestal(fFPNSigmaThreshold);

               Bool_t good = frame->SubtractPedestal(iAget, iCh, fPedestalRMSFactor); // TODO
               fRawEventPtr->SetIsGood(good);

               if (!good) {
                  iAget = 4;
                  iCh = 68;

                  continue;
               }

               Double_t *adc = frame->GetADC(iAget, iCh); // DEBUG Chrasih because SetPedestalSubstracted is not done!!!

               // if (fIsGainCalibrationData)//TODO
               //  fGainCalibrationPtr -> CalibrateADC(row, layer, fNumTbs, adc);

               for (Int_t iTb = 0; iTb < fGETDecoderPtr->GetNumTbs(); iTb++) {
                  Int_t delayedTb = iTb - signalDelay;
                  Double_t delayedADC = 0;
                  //  if (delayedTb < 0 || delayedTb >= fGETDecoderPtr -> GetNumTbs())
                  // 	 delayedADC = 0;
                  //   else
                  //	 delayedADC = adc[delayedTb];

                  //  std::cout<<delayedADC<<"   "<<adc[delayedTb]<<std::endl;
                  // pad -> SetADC(iTb, delayedADC);
                  pad.SetADC(iTb, adc[iTb]); // TODO For the moment we fill it without delay
                  // wave-> Fill(iTb, adc[iTb]);
               }

               // wave->Draw("col");

               Int_t maxADCIdx = frame->GetMaxADCIdx(iAget, iCh) + signalDelay;
               if (maxADCIdx < 0 || maxADCIdx >= fGETDecoderPtr->GetNumTbs())
                  maxADCIdx = 0;

               pad.SetPedestalSubtracted(kTRUE);
            }
         }
      }

      if (frameType == GETDecoder::kMergedID || frameType == GETDecoder::kMergedTime) {
         Int_t currentInnerFrameID = fGETDecoderPtr->GetCurrentInnerFrameID();
         Int_t numInnerFrames = fGETDecoderPtr->GetNumMergedFrames();

         if (currentInnerFrameID + 1 == numInnerFrames) {
            fCurrFrameNo++;
            fPrevEventNo = fCurrEventNo;
            return fRawEventPtr;
         }
      } else
         fCurrFrameNo++;
   }

   return NULL;
}

void AtCore::SetDebugMode(Bool_t Debug)
{

   kDebug = Debug;
   fAtMapPtr->SetDebugMode();
}
