// =================================================
//  AtCore Class
//  Original author : Genie Jhang ( geniejhang@majimak.com )
//  Adapted for AtTPCROOT by Y. Ayyad (ayyadlim@nscl.msu.edu)
// =================================================

#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>

#include "AtCore2.h"
#include "FairLogger.h"

#include "GETCoboFrame.h"
#include "GETLayeredFrame.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtCore2);

AtCore2::AtCore2() : AtPadCoordArr(boost::extents[10240][3][2])
{
   Initialize();
}

AtCore2::AtCore2(std::shared_ptr<AtMap> map) : AtPadCoordArr(boost::extents[10240][3][2]), fMap(map)
{
   Initialize();
   SetNumTbs(512);
}

AtCore2::AtCore2(std::shared_ptr<AtMap> map, Int_t numCobos) : fNumCobo(numCobos), fMap(map)
{
   Initialize();
   SetNumTbs(512);
}
AtCore2::AtCore2(TString filename, std::shared_ptr<AtMap> map) : AtPadCoordArr(boost::extents[10240][3][2])
{

   Initialize();
   AddData(filename);
   SetNumTbs(512);
}

AtCore2::AtCore2(TString filename, Int_t numTbs, Int_t windowNumTbs, Int_t windowStartTb)
   : AtPadCoordArr(boost::extents[10240][3][2])
{
   Initialize();
   AddData(filename);
   SetNumTbs(numTbs);
}

AtCore2::~AtCore2()
{
   /*for(Int_t i=0;i<10;i++)
   {
     delete fDecoderPtr[i];
     delete fPedestalPtr[i];
   }
   delete fAtMapPtr;*/
}

void AtCore2::Initialize()
{

   std::cout << " ======= AtCore2::Initialize "
             << "\n";

   fRawEventPtr = new AtRawEvent();

   fIsNegativePolarity = kTRUE;
   fPedestalPtr[0] = new AtPedestal();
   for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++)
      fPedestalPtr[iCobo] = nullptr;

   fDecoderPtr[0] = new GETDecoder2();
   // fPadArray = new TClonesArray("AtPad", 10240);

   fIsData = kFALSE;
   fFPNSigmaThreshold = 5;

   fNumTbs = 512;

   fTargetFrameID = -1;
   memset(fCurrentEventID, 0, sizeof(Int_t) * fNumCobo);

   fIsSeparatedData = kFALSE;

   std::cout << " =========== End of AtCore2 Initialization. Number of Cobo/AsAd : " << fNumCobo << "\n";
}

Bool_t AtCore2::AddData(TString filename, Int_t coboIdx)
{
   return fDecoderPtr[coboIdx]->AddData(filename);
}

void AtCore2::SetPositivePolarity(Bool_t value)
{
   fIsNegativePolarity = !value;
}

Bool_t AtCore2::SetData(Int_t value)
{
   fIsData = fDecoderPtr[0]->SetData(value);
   GETDecoder2::EFrameType frameType = fDecoderPtr[0]->GetFrameType();

   if (fIsSeparatedData) {
      for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++) {
         if (fPedestalPtr[iCobo] == nullptr)
            fPedestalPtr[iCobo] = new AtPedestal();

         fIsData &= fDecoderPtr[iCobo]->SetData(value);
         frameType = fDecoderPtr[iCobo]->GetFrameType();
         /*
               if (frameType != GETDecoder2::kCobo || frameType != GETDecoder2::kBasic) {
                 std::cout << cRED << "== [AtCore] When using separated data, only accepted are not merged frame data
            files!" << cNORMAL << std::endl;

                 fIsData = kFALSE;
                 return fIsData;
               }
               */
         fIsData = kTRUE;
      }
   }

   fTargetFrameID = -1;
   memset(fCurrentEventID, 0, sizeof(Int_t) * fNumCobo);

   return fIsData;
}

void AtCore2::SetDiscontinuousData(Bool_t value)
{
   fDecoderPtr[0]->SetDiscontinuousData(value);
   if (fIsSeparatedData)
      for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++)
         fDecoderPtr[iCobo]->SetDiscontinuousData(value);
}

Int_t AtCore2::GetNumData(Int_t coboIdx)
{
   return fDecoderPtr[coboIdx]->GetNumData();
}

TString AtCore2::GetDataName(Int_t ind, Int_t coboIdx)
{
   return fDecoderPtr[coboIdx]->GetDataName(ind);
}

void AtCore2::SetNumTbs(Int_t value)
{
   fNumTbs = value;
   fDecoderPtr[0]->SetNumTbs(value);

   if (fIsSeparatedData)
      for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++)
         fDecoderPtr[iCobo]->SetNumTbs(value);
}

void AtCore2::SetFPNPedestal(Double_t sigmaThreshold)
{
   fFPNSigmaThreshold = sigmaThreshold;

   std::cout << "== [AtCore] Using FPN pedestal is set!" << std::endl;
}

void AtCore2::ProcessCobo(Int_t coboIdx)
{

   GETCoboFrame *coboFrame = fDecoderPtr[coboIdx]->GetCoboFrame(fTargetFrameID);

   if (coboFrame == nullptr) {
      fRawEventPtr->SetIsGood(kFALSE);
      std::cout << " Null frame! CoboIdx " << coboIdx << "\n";
      return;
   }

   fCurrentEventID[coboIdx] = coboFrame->GetEventID();
   Int_t numFrames = coboFrame->GetNumFrames();

   for (Int_t iFrame = 0; iFrame < numFrames; iFrame++) {
      GETBasicFrame *frame = coboFrame->GetFrame(iFrame);

      Int_t iCobo = frame->GetCoboID();
      Int_t iAsad = frame->GetAsadID();

      for (Int_t iAget = 0; iAget < 4; iAget++) {
         for (Int_t iCh = 0; iCh < 68; iCh++) {

            PadReference PadRef = {iCobo, iAsad, iAget, iCh};
            Int_t PadRefNum = fMap->GetPadNum(PadRef);
            auto PadCenterCoord = fMap->CalcPadCenter(PadRefNum);
            Bool_t IsInhibited = fMap->IsInhibited(PadRefNum);

            if (PadRefNum != -1 && !fMap->IsInhibited(PadRefNum)) {
               AtPad *pad;
               {
                  std::lock_guard<std::mutex> lk(fRawEventMutex);
                  // pad = new ((*fPadArray)[PadRefNum]) AtPad(PadRefNum);
                  pad = fRawEventPtr->AddPad(PadRefNum);
               }

               pad->SetPadCoord(PadCenterCoord);
               if (PadRefNum == -1)
                  pad->SetValidPad(kFALSE);
               else
                  pad->SetValidPad(kTRUE);

               Int_t *rawadc = frame->GetSample(iAget, iCh);
               for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad->SetRawADC(iTb, rawadc[iTb]);

               Int_t fpnCh = GetFPNChannel(iCh);
               Double_t adc[512] = {0};
               fPedestalPtr[coboIdx]->SubtractPedestal(fNumTbs, frame->GetSample(iAget, fpnCh), rawadc, adc,
                                                       fFPNSigmaThreshold);

               for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad->SetADC(iTb, adc[iTb]);

               pad->SetPedestalSubtracted(kTRUE);
            }
         }
      }
   }
}

void AtCore2::ProcessBasicCobo(Int_t coboIdx)
{

   GETBasicFrame *basicFrame = fDecoderPtr[coboIdx]->GetBasicFrame(fTargetFrameID);

   if (basicFrame == nullptr) {
      fRawEventPtr->SetIsGood(kFALSE);

      return;
   }

   Int_t iCobo = basicFrame->GetCoboID();
   Int_t iAsad = basicFrame->GetAsadID();

   for (Int_t iAget = 0; iAget < 4; iAget++) {
      for (Int_t iCh = 0; iCh < 68; iCh++) {

         PadReference PadRef = {iCobo, iAsad, iAget, iCh};
         auto PadRefNum = fMap->GetPadNum(PadRef);
         auto PadCenterCoord = fMap->CalcPadCenter(PadRefNum);
         Bool_t IsInhibited = fMap->IsInhibited(PadRefNum);

         if (PadRefNum != -1 && !fMap->IsInhibited(PadRefNum)) {
            AtPad *pad = nullptr;
            {
               // Ensure the threads aren't both trying to create pads at the same time
               std::lock_guard<std::mutex> lk(fRawEventMutex);
               pad = fRawEventPtr->AddPad(PadRefNum);
            }

            pad->SetPadCoord(PadCenterCoord);
            pad->SetValidPad(kTRUE);

            Int_t *rawadc = basicFrame->GetSample(iAget, iCh);

            for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
               pad->SetRawADC(iTb, rawadc[iTb]);
               // std::cout<<iTb<<" "<<rawadc[iTb]<<"\n";
            }

            Int_t fpnCh = GetFPNChannel(iCh);
            Double_t adc[512] = {0};
            fPedestalPtr[coboIdx]->SubtractPedestal(fNumTbs, basicFrame->GetSample(iAget, fpnCh), rawadc, adc,
                                                    fFPNSigmaThreshold);

            for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
               pad->SetADC(iTb, adc[iTb]);

            pad->SetPedestalSubtracted(kTRUE);
         }
      }
   }
}

Bool_t AtCore2::SetWriteFile(TString filename, Int_t coboIdx, Bool_t overwrite)
{
   return fDecoderPtr[coboIdx]->SetWriteFile(filename, overwrite);
}

void AtCore2::WriteData()
{
   if (fRawEventPtr == nullptr) {
      std::cout << "== [AtCore] Call this method after GetRawEvent()!" << std::endl;

      return;
   }

   if (fIsSeparatedData) {
      for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++) {
         fDecoderPtr[iCobo]->GetCoboFrame(fTargetFrameID);
         fDecoderPtr[iCobo]->WriteFrame();
      }
   } else
      fDecoderPtr[0]->WriteFrame();
}

AtRawEvent *AtCore2::GetRawEvent(Long64_t frameID)
{

   if (!fIsData) {
      std::cout << cRED << "== [AtCore] Data file is not set!" << cNORMAL << std::endl;

      return nullptr;
   }

   if (fIsSeparatedData) {
      fRawEventPtr->Clear();
      // fPadArray->Clear("C");

      if (frameID == -1)
         fTargetFrameID++;
      else
         fTargetFrameID = frameID;

      std::thread *cobo = new std::thread[fNumCobo];

      for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++) {
         cobo[iCobo] = std::thread([this](Int_t coboIdx) { this->ProcessBasicCobo(coboIdx); }, iCobo);
      }

      for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++)
         cobo[iCobo].join();

      // NB: Do not delete. To be refactored using functors
      /* for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++){
   cobo[iCobo] = std::thread([this](Int_t coboIdx) { this->ProcessCobo(coboIdx); }, iCobo);
      }*/

      for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++)
         if (fCurrentEventID[0] != fCurrentEventID[iCobo]) {
            std::cout << "== [AtCore] Event IDs don't match between CoBos! fCurrentEventID[0]: " << fCurrentEventID[0]
                      << " fCurrentEventID[" << iCobo << "]: " << fCurrentEventID[iCobo] << std::endl;

            fRawEventPtr->SetIsGood(kFALSE);
         }

      fRawEventPtr->SetEventID(fCurrentEventID[0]);

      delete[] cobo;

      if (fRawEventPtr->GetNumPads() == 0 && fRawEventPtr->IsGood() == kFALSE)
         return nullptr;
      else
         return fRawEventPtr;
   } else { //! For basic and merged frames, i.e. one graw file

      fRawEventPtr->Clear();
      // fPadArray->Clear("C");

      if (frameID == -1)
         fTargetFrameID++;
      else
         fTargetFrameID = frameID;

      if (dynamic_cast<AtTpcMap *>(fMap.get()) != nullptr) {
         GETLayeredFrame *layeredFrame = fDecoderPtr[0]->GetLayeredFrame(fTargetFrameID);
         if (layeredFrame == nullptr)
            return nullptr;
         ProcessLayeredFrame(layeredFrame);
      } else if (dynamic_cast<AtTpcProtoMap *>(fMap.get()) != nullptr) {
         GETBasicFrame *basicFrame = fDecoderPtr[0]->GetBasicFrame(fTargetFrameID);
         if (basicFrame == nullptr)
            return nullptr;
         ProcessBasicFrame(basicFrame);
      }

      return fRawEventPtr;
   } //! else

   return nullptr;
}

Int_t AtCore2::GetEventID()
{
   return fRawEventPtr->GetEventID();
}

Int_t AtCore2::GetNumTbs(Int_t coboIdx)
{
   return fDecoderPtr[coboIdx]->GetNumTbs();
}

void AtCore2::SetUseSeparatedData(Bool_t value)
{
   fIsSeparatedData = value;

   if (fIsSeparatedData) {
      std::cout << cYELLOW << "== [AtCore] You set the decoder to analyze seperated data files." << std::endl;
      std::cout << "            Make sure to call this method right after the instance created!" << cNORMAL
                << std::endl;

      for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++) {
         fDecoderPtr[iCobo] = new GETDecoder2();
      }
   }
}

Int_t AtCore2::GetFPNChannel(Int_t chIdx)
{
   Int_t fpn = -1;

   if (chIdx < 17)
      fpn = 11;
   else if (chIdx < 34)
      fpn = 22;
   else if (chIdx < 51)
      fpn = 45;
   else
      fpn = 56;

   return fpn;
}

void AtCore2::SetPseudoTopologyFrame(Int_t asadMask, Bool_t check)
{
   for (Int_t i = 0; i < fNumCobo; i++)
      fDecoderPtr[i]->SetPseudoTopologyFrame(asadMask, check);
}

void AtCore2::ProcessLayeredFrame(GETLayeredFrame *layeredFrame)
{

   fRawEventPtr->SetEventID(layeredFrame->GetEventID());

   Int_t numFrames = layeredFrame->GetNItems();
   for (Int_t iFrame = 0; iFrame < numFrames; iFrame++) {
      GETBasicFrame *frame = layeredFrame->GetFrame(iFrame);

      Int_t iCobo = frame->GetCoboID();
      Int_t iAsad = frame->GetAsadID();

      for (Int_t iAget = 0; iAget < 4; iAget++) {
         for (Int_t iCh = 0; iCh < 68; iCh++) {

            PadReference PadRef = {iCobo, iAsad, iAget, iCh};
            Int_t PadRefNum = fMap->GetPadNum(PadRef);
            auto PadCenterCoord = fMap->CalcPadCenter(PadRefNum);

            if (PadRefNum != -1) {
               AtPad *pad;
               {
                  std::lock_guard<std::mutex> lk(fRawEventMutex);
                  // AtPad *pad = new ((*fPadArray)[PadRefNum]) AtPad(PadRefNum);
                  pad = fRawEventPtr->AddPad(PadRefNum);
               }

               pad->SetPadCoord(PadCenterCoord);

               if (PadRefNum == -1)
                  pad->SetValidPad(kFALSE);
               else
                  pad->SetValidPad(kTRUE);

               Int_t *rawadc = frame->GetSample(iAget, iCh);
               for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad->SetRawADC(iTb, rawadc[iTb]);

               Int_t fpnCh = GetFPNChannel(iCh);
               Double_t adc[512] = {0};
               Bool_t good = fPedestalPtr[0]->SubtractPedestal(fNumTbs, frame->GetSample(iAget, fpnCh), rawadc, adc,
                                                               fFPNSigmaThreshold);

               for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
                  pad->SetADC(iTb, adc[iTb]);

               pad->SetPedestalSubtracted(kTRUE);
               fRawEventPtr->SetIsGood(good);
            }
         }
      }
   }
}

void AtCore2::ProcessBasicFrame(GETBasicFrame *basicFrame)
{

   Int_t iCobo = basicFrame->GetCoboID();
   Int_t iAsad = basicFrame->GetAsadID();

   for (Int_t iAget = 0; iAget < 4; iAget++) {
      for (Int_t iCh = 0; iCh < 68; iCh++) {

         PadReference PadRef = {iCobo, iAsad, iAget, iCh};
         Int_t PadRefNum = fMap->GetPadNum(PadRef);
         auto PadCenterCoord = fMap->CalcPadCenter(PadRefNum);

         if (PadRefNum != -1) {
            // AtPad *pad = new ((*fPadArray)[PadRefNum]) AtPad(PadRefNum);
            AtPad *pad;
            if (fMap->IsAuxPad(PadRef)) {
               std::lock_guard<std::mutex> lk(fRawEventMutex);
               pad = fRawEventPtr->AddAuxPad(fMap->GetAuxName(PadRef)).first;
            } else {
               std::lock_guard<std::mutex> lk(fRawEventMutex);
               pad = fRawEventPtr->AddPad(PadRefNum);
            }

            pad->SetPadCoord(PadCenterCoord);

            if (PadRefNum == -1)
               pad->SetValidPad(kFALSE);
            else
               pad->SetValidPad(kTRUE);

            Int_t *rawadc = basicFrame->GetSample(iAget, iCh);
            for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
               // std::cout<<" iTb "<<iTb<<" "<<rawadc[iTb]<<"\n";
               pad->SetRawADC(iTb, rawadc[iTb]);
            }

            Int_t fpnCh = GetFPNChannel(iCh);
            Double_t adc[512] = {0};
            Bool_t good = fPedestalPtr[0]->SubtractPedestal(fNumTbs, basicFrame->GetSample(iAget, fpnCh), rawadc, adc,
                                                            fFPNSigmaThreshold);

            for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
               pad->SetADC(iTb, adc[iTb]);

            pad->SetPedestalSubtracted(kTRUE);
            fRawEventPtr->SetIsGood(good);
         }
      }
   }
}

void AtCore2::SetNumCobo(Int_t numCobo)
{
   fNumCobo = numCobo;
}
