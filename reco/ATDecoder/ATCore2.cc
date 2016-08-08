// =================================================
//  ATCore Class
//  Original author : Genie Jhang ( geniejhang@majimak.com )
//  Adapted for ATTPCROOT by Y. Ayyad (ayyadlim@nscl.msu.edu)
// =================================================

#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>

#include "ATCore2.hh"

#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "ATPedestal.hh"
#include "ATRawEvent.hh"

#include "GETCoboFrame.hh"
#include "GETLayeredFrame.hh"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATCore2);

ATCore2::ATCore2():AtPadCoordArr(boost::extents[10240][3][2]),kOpt(0)
{
  Initialize();
}

ATCore2::ATCore2(Int_t opt):AtPadCoordArr(boost::extents[10240][3][2]),kOpt(0)
{
  kOpt=opt;
  Initialize();
  SetNumTbs(512);

}

ATCore2::ATCore2(TString filename,Int_t opt):AtPadCoordArr(boost::extents[10240][3][2]),kOpt(0)
{

  kOpt=opt;
  Initialize();
  AddData(filename);
  SetNumTbs(512);

}

ATCore2::ATCore2(TString filename, Int_t numTbs, Int_t windowNumTbs, Int_t windowStartTb):AtPadCoordArr(boost::extents[10240][3][2]),kOpt(0)
{
  Initialize();
  AddData(filename);
  SetNumTbs(numTbs);
}

ATCore2::~ATCore2()
{
  /*for(Int_t i=0;i<10;i++)
  {
    delete fDecoderPtr[i];
    delete fPedestalPtr[i];
  }
  delete fAtMapPtr;*/

}

void ATCore2::Initialize()
{
  fRawEventPtr = new ATRawEvent();

  if(kOpt==0) fAtMapPtr = new AtTpcMap();
  else if(kOpt==1) fAtMapPtr = new AtTpcProtoMap();
  else std::cout << "== ATCore Initialization Error : Option not found. Current available options: ATTPC Map 0 / Prototype Map 1" << std::endl;

  fIsNegativePolarity = kTRUE;
  fPedestalPtr[0] = new ATPedestal();
  for (Int_t iCobo = 1; iCobo < 10; iCobo++)
    fPedestalPtr[iCobo] = NULL;

  //fPlotPtr = NULL;

  fDecoderPtr[0] = new GETDecoder2();
//  fDecoderPtr[0] -> SetDebugMode(1);
  fPadArray = new TClonesArray("ATPad", 10240);

  fIsData = kFALSE;
  fFPNSigmaThreshold = 5;
  fIsProtoGeoSet = kFALSE;
  fIsProtoMapSet = kFALSE;

  //fGainCalibrationPtr = new STGainCalibration();
  //fIsGainCalibrationData = kFALSE;

  fNumTbs = 512;

  fTargetFrameID = -1;
  memset(fCurrentEventID, 0, sizeof(Int_t)*10);

  fIsSeparatedData = kFALSE;
  kEnableAuxChannel = kFALSE;
  fAuxChannels.clear();

  fNumCobo = 10;
}

Bool_t ATCore2::AddData(TString filename, Int_t coboIdx)
{
  return fDecoderPtr[coboIdx] -> AddData(filename);
}

void ATCore2::SetPositivePolarity(Bool_t value)
{
  fIsNegativePolarity = !value;
}

Bool_t ATCore2::SetData(Int_t value)
{
  fIsData = fDecoderPtr[0] -> SetData(value);
  GETDecoder2::EFrameType frameType = fDecoderPtr[0] -> GetFrameType();

  if (fIsSeparatedData) {
    for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++) {
      if (fPedestalPtr[iCobo] == NULL)
        fPedestalPtr[iCobo] = new ATPedestal();

      fIsData &= fDecoderPtr[iCobo] -> SetData(value);
      frameType = fDecoderPtr[iCobo] -> GetFrameType();

      if (frameType != GETDecoder2::kCobo) {
        std::cout << cRED << "== [ATCore] When using separated data, only accepted are not merged frame data files!" << cNORMAL << std::endl;

        fIsData = kFALSE;
        return fIsData;
      }
    }
  }

  fTargetFrameID = -1;
  memset(fCurrentEventID, 0, sizeof(Int_t)*12);

  return fIsData;
}

void ATCore2::SetDiscontinuousData(Bool_t value)
{
  fDecoderPtr[0] -> SetDiscontinuousData(value);
  if (fIsSeparatedData)
    for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++)
      fDecoderPtr[iCobo] -> SetDiscontinuousData(value);
}

Int_t ATCore2::GetNumData(Int_t coboIdx)
{
  return fDecoderPtr[coboIdx] -> GetNumData();
}

TString ATCore2::GetDataName(Int_t index, Int_t coboIdx)
{
  return fDecoderPtr[coboIdx] -> GetDataName(index);
}

void ATCore2::SetNumTbs(Int_t value)
{
  fNumTbs = value;
  fDecoderPtr[0] -> SetNumTbs(value);

  if (fIsSeparatedData)
    for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++)
      fDecoderPtr[iCobo] -> SetNumTbs(value);
}

void ATCore2::SetFPNPedestal(Double_t sigmaThreshold)
{
  fFPNSigmaThreshold = sigmaThreshold;

  std::cout << "== [ATCore] Using FPN pedestal is set!" << std::endl;
}

Bool_t ATCore2::SetATTPCMap(Char_t const *lookup){

  if(kOpt==0) fAtMapPtr->GenerateATTPC(); // NOTE: In the case of the ATTPC Map we need to generate the coordinates to calculate the Pad Center
  Bool_t MapIn = fAtMapPtr->ParseXMLMap(lookup);
  if(!MapIn) return false;

  Bool_t kIsIniParsed = fAtMapPtr->ParseInhibitMap(fIniMap,fLowgMap,fXtalkMap);

  //AtPadCoordArr = fAtMapPtr->GetPadCoordArr();//TODO Use a pointer to a simpler container
  //**** For debugging purposes only! ******//
  //fAtMapPtr->SetGUIMode();
  //fAtMapPtr->GetATTPCPlane();
  return true;

}

Bool_t ATCore2::SetProtoGeoFile(TString geofile){

   if(kOpt==1){

	fIsProtoGeoSet = fAtMapPtr->SetGeoFile(geofile);
        return fIsProtoGeoSet;

   }else{
	 std::cout << "== ATCore::SetProtoGeoMap. This method must be used only with Prototype mapping (kOpt=1)!" << std::endl;
         return kFALSE;
   }


}


Bool_t ATCore2::SetProtoMapFile(TString mapfile){

  if(kOpt==1){

	fIsProtoMapSet = fAtMapPtr->SetProtoMap(mapfile);
        return fIsProtoMapSet;

   }else{
	 std::cout << "== ATCore::SetProtoMapFile. This method must be used only with Prototype mapping (kOpt=1)!" << std::endl;
         return kFALSE;
   }


}


Bool_t ATCore2::SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap)
{
   fIniMap = inimap;
   fLowgMap = lowgmap;
   fXtalkMap = xtalkmap;
   return kTRUE;
}

/*Bool_t STCore::SetGainCalibrationData(TString filename, TString dataType)
{
  fIsGainCalibrationData = fGainCalibrationPtr -> SetGainCalibrationData(filename, dataType);

  std::cout << "== [STCore] Gain calibration data is set!" << std::endl;
  return fIsGainCalibrationData;
}*/

/*void STCore::SetGainReference(Int_t row, Int_t layer)
{
  if (!fIsGainCalibrationData) {
    std::cout << "== [STCore] Set gain calibration data first!" << std::endl;

    return;
  }

  fGainCalibrationPtr -> SetGainReference(row, layer);
}*/

/*void STCore::SetGainReference(Double_t constant, Double_t linear, Double_t quadratic)
{
  if (!fIsGainCalibrationData) {
    std::cout << "== [STCore] Set gain calibration data first!" << std::endl;

    return;
  }

  fGainCalibrationPtr -> SetGainReference(constant, linear, quadratic);
}*/

/*Bool_t STCore::SetUAMap(TString filename)
{
  return fMapPtr -> SetUAMap(filename);
}*/

/*Bool_t STCore::SetAGETMap(TString filename)
{
  return fMapPtr -> SetAGETMap(filename);
}*/

void ATCore2::ProcessCobo(Int_t coboIdx)
{

  GETCoboFrame *coboFrame = fDecoderPtr[coboIdx] -> GetCoboFrame(fTargetFrameID);

  if (coboFrame == NULL) {
    fRawEventPtr -> SetIsGood(kFALSE);

    return;
  }

  fCurrentEventID[coboIdx] = coboFrame -> GetEventID();
  Int_t numFrames = coboFrame -> GetNumFrames();
  for (Int_t iFrame = 0; iFrame < numFrames; iFrame++) {
    GETBasicFrame *frame = coboFrame -> GetFrame(iFrame);


    Int_t iCobo = frame -> GetCoboID();
    Int_t iAsad = frame -> GetAsadID();

    for (Int_t iAget = 0; iAget < 4; iAget++) {
      for (Int_t iCh = 0; iCh < 68; iCh++) {

        std::vector<int> PadRef={iCobo,iAsad,iAget,iCh};
        Int_t PadRefNum = fAtMapPtr->GetPadNum(PadRef);
        std::vector<Float_t> PadCenterCoord;
        PadCenterCoord.reserve(2);
        PadCenterCoord = fAtMapPtr->CalcPadCenter(PadRefNum);
        Bool_t IsInhibited = fAtMapPtr->GetIsInhibited(PadRefNum);

            if(PadRefNum!=-1  && !fAtMapPtr->GetIsInhibited(PadRefNum)){
                ATPad *pad = new ((*fPadArray)[PadRefNum]) ATPad(PadRefNum);
                //if(PadRefNum)
                pad->SetPadXCoord(PadCenterCoord[0]);
                pad->SetPadYCoord(PadCenterCoord[1]);
                if(PadRefNum==-1) pad->SetValidPad(kFALSE);
                else pad->SetValidPad(kTRUE);

                Int_t *rawadc = frame -> GetSample(iAget, iCh);
                for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad -> SetRawADC(iTb, rawadc[iTb]);

                Int_t fpnCh = GetFPNChannel(iCh);
                Double_t adc[512] = {0};
                fPedestalPtr[coboIdx] -> SubtractPedestal(fNumTbs, frame -> GetSample(iAget, fpnCh), rawadc, adc, fFPNSigmaThreshold);

                //if (fIsGainCalibrationData)
                //  fGainCalibrationPtr -> CalibrateADC(row, layer, fNumTbs, adc);

                for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad -> SetADC(iTb, adc[iTb]);

                  pad -> SetPedestalSubtracted(kTRUE);
            }
      }
    }
  }

}

Bool_t ATCore2::SetWriteFile(TString filename, Int_t coboIdx, Bool_t overwrite)
{
  return fDecoderPtr[coboIdx] -> SetWriteFile(filename, overwrite);
}

void ATCore2::WriteData()
{
  if (fRawEventPtr == NULL) {
    std::cout << "== [ATCore] Call this method after GetRawEvent()!" << std::endl;

    return;
  }

  if (fIsSeparatedData)  {
    for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++) {
      fDecoderPtr[iCobo] -> GetCoboFrame(fTargetFrameID);
      fDecoderPtr[iCobo] -> WriteFrame();
    }
  } else
    fDecoderPtr[0] -> WriteFrame();
}

ATRawEvent *ATCore2::GetRawEvent(Long64_t frameID)
{
  if (!fIsData) {
    std::cout << "== [ATCore] Data file is not set!" << std::endl;

    return NULL;
  }

  if (fIsSeparatedData) {
    fRawEventPtr -> Clear();
    fPadArray -> Clear("C");

    if (frameID == -1)
      fTargetFrameID++;
    else
      fTargetFrameID = frameID;

/*  if(fNumCobo==10){
    std::thread cobo0([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 0);
    std::thread cobo1([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 1);
    std::thread cobo2([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 2);
    std::thread cobo3([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 3);
    std::thread cobo4([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 4);
    std::thread cobo5([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 5);
    std::thread cobo6([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 6);
    std::thread cobo7([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 7);
    std::thread cobo8([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 8);
    std::thread cobo9([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 9);
    //std::thread cobo10([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 10);
    //std::thread cobo11([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 11);

    cobo0.join();
    cobo1.join();
    cobo2.join();
    cobo3.join();
    cobo4.join();
    cobo5.join();
    cobo6.join();
    cobo7.join();
    cobo8.join();
    cobo9.join();
    //cobo10.join();
    //cobo11.join();
  }else if(fNumCobo==9){

    std::thread cobo0([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 0);
    std::thread cobo1([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 1);
    std::thread cobo2([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 2);
    std::thread cobo3([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 3);
    std::thread cobo4([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 4);
    std::thread cobo5([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 5);
    std::thread cobo6([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 6);
    std::thread cobo7([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 7);
    std::thread cobo8([this](Int_t coboIdx) { this -> ProcessCobo(coboIdx); }, 8);
    cobo0.join();
    cobo1.join();
    cobo2.join();
    cobo3.join();
    cobo4.join();
    cobo5.join();
    cobo6.join();
    cobo7.join();
    cobo8.join();


  }*/

  std::thread* cobo = new std::thread[10];

  for (Int_t iCobo = 0; iCobo < fNumCobo ; iCobo++)
            cobo[iCobo]  = std::thread([this](Int_t coboIdx) {  this -> ProcessCobo(coboIdx); }, iCobo);

  for (Int_t iCobo = 0; iCobo < fNumCobo ; iCobo++)
            cobo[iCobo].join();




    for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++)
      if (fCurrentEventID[0] != fCurrentEventID[iCobo]) {
        std::cout << "== [ATCore] Event IDs don't match between CoBos! fCurrentEventID[0]: " << fCurrentEventID[0] << " fCurrentEventID[" << iCobo << "]: " << fCurrentEventID[iCobo] << std::endl;

        fRawEventPtr -> SetIsGood(kFALSE);
      }

    fRawEventPtr -> SetEventID(fCurrentEventID[0]);

    Int_t iNumPads=10240;
    for (Int_t i = 0; i < iNumPads; i++) {

        ATPad *pad = (ATPad *) fPadArray -> At(i);
        if (pad != NULL)
          fRawEventPtr -> SetPad(pad);
    }

    if (fRawEventPtr -> GetNumPads() == 0 && fRawEventPtr -> IsGood() == kFALSE)
      return NULL;
    else
      return fRawEventPtr;
  } else { //! For basic and merged frames, i.e. one graw file


    fRawEventPtr -> Clear();
    fPadArray -> Clear("C");

    if (frameID == -1)
      fTargetFrameID++;
    else
      fTargetFrameID = frameID;

     if(kOpt==0){
          GETLayeredFrame *layeredFrame = fDecoderPtr[0] -> GetLayeredFrame(fTargetFrameID);
            if (layeredFrame == NULL) return NULL;
            ProcessLayeredFrame(layeredFrame);
      }else if (kOpt==1){
          GETBasicFrame *basicFrame = fDecoderPtr[0] -> GetBasicFrame(fTargetFrameID);
          if (basicFrame == NULL) return NULL;
          ProcessBasicFrame(basicFrame);
      }

  /*  fRawEventPtr -> SetEventID(layeredFrame -> GetEventID());

    Int_t numFrames = layeredFrame -> GetNItems();
    for (Int_t iFrame = 0; iFrame < numFrames; iFrame++) {
      GETBasicFrame *frame = layeredFrame -> GetFrame(iFrame);

      Int_t iCobo = frame -> GetCoboID();
      Int_t iAsad = frame -> GetAsadID();

      for (Int_t iAget = 0; iAget < 4; iAget++) {
        for (Int_t iCh = 0; iCh < 68; iCh++) {

          std::vector<int> PadRef={iCobo,iAsad,iAget,iCh};
          Int_t PadRefNum = fAtMapPtr->GetPadNum(PadRef);
          std::vector<Float_t> PadCenterCoord;
          PadCenterCoord.reserve(2);
          PadCenterCoord = fAtMapPtr->CalcPadCenter(PadRefNum);

                  if(PadRefNum!=-1){
                  ATPad *pad = new ((*fPadArray)[PadRefNum]) ATPad(PadRefNum);
                  //ATPad *pad = new ATPad(PadRefNum);
                  pad->SetPadXCoord(PadCenterCoord[0]);
                  pad->SetPadYCoord(PadCenterCoord[1]);
                  if(PadRefNum==-1) pad->SetValidPad(kFALSE);
                  else pad->SetValidPad(kTRUE);

                  Int_t *rawadc = frame -> GetSample(iAget, iCh);
                  for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                    pad -> SetRawADC(iTb, rawadc[iTb]);

                  Int_t fpnCh = GetFPNChannel(iCh);
                  Double_t adc[512] = {0};
                  Bool_t good = fPedestalPtr[0] -> SubtractPedestal(fNumTbs, frame -> GetSample(iAget, fpnCh), rawadc, adc, fFPNSigmaThreshold);

                  //if (fIsGainCalibrationData)
                    //=fGainCalibrationPtr -> CalibrateADC(row, layer, fNumTbs, adc);

                  for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                    pad -> SetADC(iTb, adc[iTb]);

                  pad -> SetPedestalSubtracted(kTRUE);
                  fRawEventPtr -> SetIsGood(good);

                  fRawEventPtr -> SetPad(pad);
                }
        }
      }
    }*/

    return fRawEventPtr;
  }//! else


  return NULL;
}

Int_t ATCore2::GetEventID()
{
  return fRawEventPtr -> GetEventID();
}

Int_t ATCore2::GetNumTbs(Int_t coboIdx)
{
  return fDecoderPtr[coboIdx] -> GetNumTbs();
}

void ATCore2::SetUseSeparatedData(Bool_t value) {
  fIsSeparatedData = value;

  if (fIsSeparatedData) {
    std::cout << cYELLOW << "== [ATCore] You set the decoder to analyze seperated data files." << std::endl;
    std::cout << "            Make sure to call this method right after the instance created!" << cNORMAL << std::endl;

//    fDecoderPtr[0] -> SetDebugMode(1);
    for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++) {
      fDecoderPtr[iCobo] = new GETDecoder2();
//      fDecoderPtr[iCobo] -> SetDebugMode(1);
    }
  }
}

/*STMap *STCore2::GetSTMap()
{
  return fMapPtr;
}*/

/*STPlot *STCore::GetSTPlot()
{
  if (fPlotPtr == NULL)
    fPlotPtr = new STPlot(this);

  return fPlotPtr;
}*/

Int_t ATCore2::GetFPNChannel(Int_t chIdx)
{
  Int_t fpn = -1;

       if (chIdx < 17) fpn = 11;
  else if (chIdx < 34) fpn = 22;
  else if (chIdx < 51) fpn = 45;
  else                 fpn = 56;

  return fpn;
}

void ATCore2::SetPseudoTopologyFrame(Int_t asadMask, Bool_t check) {
          for(Int_t i=0;i<fNumCobo;i++) fDecoderPtr[i]->SetPseudoTopologyFrame(asadMask,check);
}

void ATCore2::ProcessLayeredFrame(GETLayeredFrame *layeredFrame)
{

  fRawEventPtr -> SetEventID(layeredFrame -> GetEventID());

  Int_t numFrames = layeredFrame -> GetNItems();
  for (Int_t iFrame = 0; iFrame < numFrames; iFrame++) {
    GETBasicFrame *frame = layeredFrame -> GetFrame(iFrame);

    Int_t iCobo = frame -> GetCoboID();
    Int_t iAsad = frame -> GetAsadID();

    for (Int_t iAget = 0; iAget < 4; iAget++) {
      for (Int_t iCh = 0; iCh < 68; iCh++) {

        std::vector<int> PadRef={iCobo,iAsad,iAget,iCh};
        Int_t PadRefNum = fAtMapPtr->GetPadNum(PadRef);
        std::vector<Float_t> PadCenterCoord;
        PadCenterCoord.reserve(2);
        PadCenterCoord = fAtMapPtr->CalcPadCenter(PadRefNum);

                if(PadRefNum!=-1){
                ATPad *pad = new ((*fPadArray)[PadRefNum]) ATPad(PadRefNum);
                //ATPad *pad = new ATPad(PadRefNum);
                pad->SetPadXCoord(PadCenterCoord[0]);
                pad->SetPadYCoord(PadCenterCoord[1]);
                if(PadRefNum==-1) pad->SetValidPad(kFALSE);
                else pad->SetValidPad(kTRUE);

                Int_t *rawadc = frame -> GetSample(iAget, iCh);
                for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad -> SetRawADC(iTb, rawadc[iTb]);

                Int_t fpnCh = GetFPNChannel(iCh);
                Double_t adc[512] = {0};
                Bool_t good = fPedestalPtr[0] -> SubtractPedestal(fNumTbs, frame -> GetSample(iAget, fpnCh), rawadc, adc, fFPNSigmaThreshold);

                //if (fIsGainCalibrationData)
                  //=fGainCalibrationPtr -> CalibrateADC(row, layer, fNumTbs, adc);

                for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad -> SetADC(iTb, adc[iTb]);

                pad -> SetPedestalSubtracted(kTRUE);
                fRawEventPtr -> SetIsGood(good);

                fRawEventPtr -> SetPad(pad);
              }
      }
    }
  }



}

void ATCore2::ProcessBasicFrame(GETBasicFrame *basicFrame)
{

  Int_t iCobo = basicFrame -> GetCoboID();
  Int_t iAsad = basicFrame -> GetAsadID();

  for (Int_t iAget = 0; iAget < 4; iAget++) {
    for (Int_t iCh = 0; iCh < 68; iCh++) {

      std::vector<int> PadRef={iCobo,iAsad,iAget,iCh};
      Int_t PadRefNum = fAtMapPtr->GetPadNum(PadRef);
      std::vector<Float_t> PadCenterCoord;
      PadCenterCoord.reserve(2);
      PadCenterCoord = fAtMapPtr->CalcPadCenter(PadRefNum);



              if(PadRefNum!=-1){
              ATPad *pad = new ((*fPadArray)[PadRefNum]) ATPad(PadRefNum);
              if(GetIsAuxChannel(PadRefNum) && kEnableAuxChannel) pad->SetIsAux(kTRUE);
              //ATPad *pad = new ATPad(PadRefNum);
              pad->SetPadXCoord(PadCenterCoord[0]);
              pad->SetPadYCoord(PadCenterCoord[1]);
              if(PadRefNum==-1) pad->SetValidPad(kFALSE);
              else pad->SetValidPad(kTRUE);

              Int_t *rawadc = basicFrame -> GetSample(iAget, iCh);
              for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                pad -> SetRawADC(iTb, rawadc[iTb]);

              Int_t fpnCh = GetFPNChannel(iCh);
              Double_t adc[512] = {0};
              Bool_t good = fPedestalPtr[0] -> SubtractPedestal(fNumTbs, basicFrame -> GetSample(iAget, fpnCh), rawadc, adc, fFPNSigmaThreshold);

              //if (fIsGainCalibrationData)
                //=fGainCalibrationPtr -> CalibrateADC(row, layer, fNumTbs, adc);

              for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
              pad -> SetADC(iTb, adc[iTb]);

              pad -> SetPedestalSubtracted(kTRUE);
              fRawEventPtr -> SetIsGood(good);

              fRawEventPtr -> SetPad(pad);
            }
    }
  }




}

void ATCore2::SetAuxChannel(std::vector<Int_t> AuxCh)
{
  kEnableAuxChannel = kTRUE;
  fAuxChannels = AuxCh;

  if(AuxCh.size()==0) std::cout<<cRED<<" ATPSATask : ATPSAProto Mode -  No auxiliary channels found --"<<cNORMAL<<std::endl;
  else{
        std::cout<<cGREEN<<" ATPSATask : Auxiliary pads found : "<<std::endl;
        for(Int_t i=0;i<AuxCh.size();i++) std::cout<<"  "<<AuxCh.at(i)<<std::endl;
  }
  std::cout<<cNORMAL<<std::endl;

}

Bool_t  ATCore2::GetIsAuxChannel(Int_t val){

  return std::find(fAuxChannels.begin(), fAuxChannels.end(), val) != fAuxChannels.end();

}

void ATCore2::SetNumCobo(Int_t numCobo) {fNumCobo=numCobo;}
