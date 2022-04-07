#include "AtGRAWUnpacker.h"

#include <fairlogger/Logger.h>
#include <thread>
#include <iostream>
#include <algorithm>
#include <utility>

#include "AtMap.h"
#include "AtPad.h"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "AtRawEvent.h"
#include "GETCoboFrame.h"
#include "GETBasicFrame.h"
#include "AtPedestal.h"
#include "GETDecoder2.h"
#include "PadReference.h"
#include <Rtypes.h>

AtGRAWUnpacker::AtGRAWUnpacker(mapPtr map, Int_t numGrawFiles)
   : AtUnpacker(map), fNumFiles(numGrawFiles), fCurrentEventID(fNumFiles, 0)
{
   fIsSeparatedData = fNumFiles > 1;
   for (int i = 0; i < fNumFiles; ++i) {
      fDecoder.push_back(std::make_unique<GETDecoder2>());
      fPedestal.push_back(std::make_unique<AtPedestal>());
   }
}
void AtGRAWUnpacker::Init()
{
   // Verify input file is there and matches constructor
   processInputFile();

   // Because we added the files after creation, need to set the index of the first data file
   // to unpack for each cobo/asad
   fIsData = true;
   for (auto &decoder : fDecoder)
      fIsData &= decoder->SetData(0);

   if (!fIsData)
      LOG(error) << "Problem setting the data pointer to the first file in the list!";

   fTargetFrameID = -1;
   fDataEventID = 0;
   fEventID = 0;
}

void AtGRAWUnpacker::FillRawEvent(AtRawEvent &event)
{
   fRawEvent = &event;

   if (!fIsData) {
      LOG(error) << "Data was not set, settinge event bad and skipping.";
      event.SetIsGood(false);
      return;
   }

   if (fIsSeparatedData) {
      auto *file = new std::thread[fNumFiles];

      for (Int_t iFile = 0; iFile < fNumFiles; iFile++) {
         file[iFile] = std::thread([this](Int_t fileIdx) { this->ProcessBasicFile(fileIdx); }, iFile);
      }

      for (Int_t iFile = 0; iFile < fNumFiles; iFile++)
         file[iFile].join();

      // NB: Do not delete. To be refactored using functors
      /* for (Int_t iFile = 0; iFile < fNumFiles; iFile++){
   file[iFile] = std::thread([this](Int_t fileIdx) { this->ProcessFile(fileIdx); }, iFile);
      }*/

      for (Int_t iFile = 0; iFile < fNumFiles; iFile++)
         if (fCurrentEventID[0] != fCurrentEventID[iFile]) {
            LOG(error) << "Event IDs don't match between files! fCurrentEventID[0]: " << fCurrentEventID[0]
                       << " fCurrentEventID[" << iFile << "]: " << fCurrentEventID[iFile] << std::endl;

            event.SetIsGood(kFALSE);
         }

      event.SetEventID(fEventID);

      delete[] file;
   } else // not seperated data
   {
      if (dynamic_cast<AtTpcMap *>(fMap.get()) != nullptr) {
         GETLayeredFrame *layeredFrame = fDecoder[0]->GetLayeredFrame(fDataEventID);
         if (layeredFrame == nullptr)
            event.SetIsGood(false);
         else
            ProcessLayeredFrame(layeredFrame);

      } else if (dynamic_cast<AtTpcProtoMap *>(fMap.get()) != nullptr) {
         GETBasicFrame *frame = fDecoder[0]->GetBasicFrame(fDataEventID);
         if (frame == nullptr)
            event.SetIsGood(false);
         else
            ProcessBasicFrame(frame);

      } else {
         LOG(ERROR) << "We are not setup to unpack unseperated data for the current detector type!";
         event.SetIsGood(false);
      }
   }

   fEventID++;
   fDataEventID++;
}

bool AtGRAWUnpacker::IsLastEvent()
{
   if (fIsSeparatedData) {
      bool isLastEvent = false;
      for (int i = 0; i < fNumFiles; ++i)
         isLastEvent |= fDecoder[i]->GetBasicFrame(fDataEventID) == nullptr;
      return isLastEvent;
   } else {
      if (dynamic_cast<AtTpcMap *>(fMap.get()) != nullptr) {
         GETLayeredFrame *frame = fDecoder[0]->GetLayeredFrame(fDataEventID);
         return frame == nullptr;
      } else if (dynamic_cast<AtTpcProtoMap *>(fMap.get()) != nullptr) {
         GETBasicFrame *frame = fDecoder[0]->GetBasicFrame(fDataEventID);
         return frame == nullptr;
      }
   }
   return true;
}

void AtGRAWUnpacker::processInputFile()
{
   if (fNumFiles == 1) {
      fDecoder[0]->AddData(fInputFileName);
      LOG(info) << "Added file to unpacker: " << fInputFileName;
      return;
   }

   std::ifstream listFile(fInputFileName);
   TString dataFileWithPath;
   Int_t fileNum = 0;
   for (int i = 0; i < fNumFiles; ++i) {
      listFile.clear();
      listFile.seekg(0);
      while (dataFileWithPath.ReadLine(listFile)) {
         if (dataFileWithPath.Contains(Form(fFileIDString.data(), i))) {
            fDecoder[i]->AddData(dataFileWithPath);
            LOG(info) << " Added file : " << dataFileWithPath << " - " << fFileIDString << " : " << i;
         } else
            LOG(error) << "Skipping file: " << dataFileWithPath << " did not match the string " << fFileIDString;
      }
   }
}

void AtGRAWUnpacker::SetInputFileName(std::string fileName)
{
   LOG(warn) << "Did not specify a string containing \"%i\" to use to map the incoming files to the proper GETDecoder. "
                "Using \"AsAd%i\" by default. To change call the function SetInputFileName(std::string fileName, "
                "std::string fileIDString) instead.";
   fInputFileName = std::move(fileName);
   fFileIDString = "AsAd%i";
}
void AtGRAWUnpacker::SetInputFileName(std::string fileName, std::string fileIDString)
{
   fInputFileName = std::move(fileName);
   fFileIDString = std::move(fileIDString);
}

Bool_t AtGRAWUnpacker::AddData(TString filename, Int_t fileIdx)
{
   if (fileIdx > fNumFiles) {
      LOG(error) << "Trying to add a file with index " << fileIdx
                 << " which is larger than the number of files we are unpacking (" << fNumFiles << ")!";
      return false;
   }
   return fDecoder[fileIdx]->AddData(filename);
}

void AtGRAWUnpacker::ProcessFile(Int_t coboIdx)
{

   GETCoboFrame *coboFrame = fDecoder[coboIdx]->GetCoboFrame(fTargetFrameID);

   if (coboFrame == nullptr) {
      fRawEvent->SetIsGood(kFALSE);
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

            if (PadRefNum != -1 && !fMap->IsInhibited(PadRefNum)) {
               AtPad *pad;
               {
                  std::lock_guard<std::mutex> lk(fRawEventMutex);
                  // pad = new ((*fPadArray)[PadRefNum]) AtPad(PadRefNum);
                  pad = fRawEvent->AddPad(PadRefNum);
               }

               pad->SetPadCoord(PadCenterCoord);
               if (PadRefNum == -1)
                  pad->SetValidPad(kFALSE);
               else
                  pad->SetValidPad(kTRUE);

               Int_t *rawadc = frame->GetSample(iAget, iCh);
               for (Int_t iTb = 0; iTb < 512; iTb++)
                  pad->SetRawADC(iTb, rawadc[iTb]);

               Int_t fpnCh = GetFPNChannel(iCh);
               Double_t adc[512] = {0};
               fPedestal[coboIdx]->SubtractPedestal(512, frame->GetSample(iAget, fpnCh), rawadc, adc,
                                                    fFPNSigmaThreshold);

               for (Int_t iTb = 0; iTb < 512; iTb++)
                  pad->SetADC(iTb, adc[iTb]);

               pad->SetPedestalSubtracted(kTRUE);
            }
         }
      }
   }
}

void AtGRAWUnpacker::ProcessBasicFile(Int_t coboIdx)
{

   GETBasicFrame *basicFrame = fDecoder[coboIdx]->GetBasicFrame(fTargetFrameID);

   if (basicFrame == nullptr) {
      fRawEvent->SetIsGood(kFALSE);

      return;
   }

   Int_t iCobo = basicFrame->GetCoboID();
   Int_t iAsad = basicFrame->GetAsadID();

   for (Int_t iAget = 0; iAget < 4; iAget++) {
      for (Int_t iCh = 0; iCh < 68; iCh++) {

         PadReference PadRef = {iCobo, iAsad, iAget, iCh};
         auto PadRefNum = fMap->GetPadNum(PadRef);
         auto PadCenterCoord = fMap->CalcPadCenter(PadRefNum);

         if (PadRefNum != -1 && !fMap->IsInhibited(PadRefNum)) {
            AtPad *pad = nullptr;
            {
               // Ensure the threads aren't both trying to create pads at the same time
               std::lock_guard<std::mutex> lk(fRawEventMutex);
               pad = fRawEvent->AddPad(PadRefNum);
            }

            pad->SetPadCoord(PadCenterCoord);
            pad->SetValidPad(kTRUE);

            Int_t *rawadc = basicFrame->GetSample(iAget, iCh);

            for (Int_t iTb = 0; iTb < 512; iTb++) {
               pad->SetRawADC(iTb, rawadc[iTb]);
               // std::cout<<iTb<<" "<<rawadc[iTb]<<"\n";
            }

            Int_t fpnCh = GetFPNChannel(iCh);
            Double_t adc[512] = {0};
            fPedestal[coboIdx]->SubtractPedestal(512, basicFrame->GetSample(iAget, fpnCh), rawadc, adc,
                                                 fFPNSigmaThreshold);

            for (Int_t iTb = 0; iTb < 512; iTb++)
               pad->SetADC(iTb, adc[iTb]);

            pad->SetPedestalSubtracted(kTRUE);
         }
      }
   }
}

Int_t AtGRAWUnpacker::GetFPNChannel(Int_t chIdx)
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
void AtGRAWUnpacker::SetPseudoTopologyFrame(Int_t asadMask, Bool_t check)
{
   for (auto &decoder : fDecoder)
      decoder->SetPseudoTopologyFrame(asadMask, check);
}

ClassImp(AtGRAWUnpacker);
