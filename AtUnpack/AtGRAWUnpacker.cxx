#include "AtGRAWUnpacker.h"

#include "AtMap.h"
#include "AtPad.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtPadReference.h"
#include "AtPadValue.h"
#include "AtPedestal.h"
#include "AtRawEvent.h"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"

#include <FairLogger.h>

#include <Rtypes.h>

#include "GETBasicFrame.h"
#include "GETCoboFrame.h"
#include "GETDecoder2.h"

#include <algorithm>
#include <array> // for array
#include <future>
#include <iostream>
#include <iterator> // for begin, end
#include <numeric>  // for accumulate
#include <thread>
#include <utility>

AtGRAWUnpacker::AtGRAWUnpacker(mapPtr map, Int_t numGrawFiles)
   : AtUnpacker(map), fNumFiles(numGrawFiles), fCurrentEventID(fNumFiles, 0), fIsSeparatedData(fNumFiles > 1)
{
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

   std::vector<int> iniFrameIDs;
   LOG(info) << "Initial frame IDs";
   for (int i = 0; i < fNumFiles; ++i) {
      GETBasicFrame *basicFrame = fDecoder[i]->GetBasicFrame(-1);
      iniFrameIDs.push_back(basicFrame->GetEventID());
      LOG(info) << i << " " << iniFrameIDs.back();
   }
   fTargetFrameID = -1;
   fDataEventID = *std::max_element(begin(iniFrameIDs), end(iniFrameIDs)) + fEventID;
   fTargetFrameID = *std::max_element(begin(iniFrameIDs), end(iniFrameIDs)) + fEventID;
}
void AtGRAWUnpacker::FindAndSetNumEvents()
{
   std::vector<int> eventCoboIDs;
   std::vector<int> lastEvents;
   std::vector<std::thread> file;
   std::vector<std::future<CoboAndEvent>> futureValues;
   if (fIsMutantOneRun) {
      fNumEvents = GetLastEvent(0).second;
   } else {
      for (int i = 0; i < fNumFiles; ++i) {
         std::promise<CoboAndEvent> p;
         futureValues.push_back(p.get_future());
         file.emplace_back(
            [this](Int_t fileIdx, std::promise<CoboAndEvent> &&promise) {
               promise.set_value(this->GetLastEvent(fileIdx));
            },
            i, std::move(p));
      }

      for (auto &fileThread : file)
         fileThread.join();

      for (auto &future : futureValues) {
         CoboAndEvent value = future.get();
         int coboNum = value.first;
         int lastEvent = value.second;
         if (std::find(begin(eventCoboIDs), end(eventCoboIDs), coboNum) != eventCoboIDs.end()) {
            for (int r = 0; r < eventCoboIDs.size(); r++) {
               if (eventCoboIDs[r] == coboNum) {
                  if (lastEvents[r] < lastEvent) {
                     lastEvents[r] = lastEvent;
                  }
               }
            }
         } else {
            eventCoboIDs.push_back(coboNum);
            lastEvents.push_back(lastEvent);
         }
      }

      fNumEvents = *std::min_element(begin(lastEvents), end(lastEvents));
   }
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
      std::vector<std::thread> file;

      for (Int_t iFile = 0; iFile < fNumFiles; iFile++) {
         file.emplace_back([this](Int_t fileIdx) { this->ProcessBasicFile(fileIdx); }, iFile);
         // file[iFile] = std::thread([this](Int_t fileIdx) { this->ProcessBasicFile(fileIdx); }, iFile);
      }

      for (auto &fileThread : file)
         fileThread.join();

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
         LOG(error) << "We are not setup to unpack unseperated data for the current detector type!";
         event.SetIsGood(false);
      }
   }

   fEventID++;
   fDataEventID++;
   if (fTargetFrameID != -1)
      fTargetFrameID++;
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
            LOG(debug) << "Skipping file: " << dataFileWithPath << " did not match the string " << fFileIDString;
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

void AtGRAWUnpacker::ProcessFile(Int_t fileIdx)
{

   GETCoboFrame *coboFrame = fDecoder[fileIdx]->GetCoboFrame(fTargetFrameID);

   if (coboFrame == nullptr) {
      fRawEvent->SetIsGood(kFALSE);
      std::cout << " Null frame! CoboIdx " << fileIdx << "\n";
      return;
   }

   fCurrentEventID[fileIdx] = coboFrame->GetEventID();
   Int_t numFrames = coboFrame->GetNumFrames();

   for (Int_t iFrame = 0; iFrame < numFrames; iFrame++) {
      GETBasicFrame *frame = coboFrame->GetFrame(iFrame);

      Int_t iCobo = frame->GetCoboID();
      Int_t iAsad = frame->GetAsadID();

      for (Int_t iAget = 0; iAget < 4; iAget++) {
         for (Int_t iCh = 0; iCh < 68; iCh++) {

            AtPadReference PadRef = {iCobo, iAsad, iAget, iCh};
            auto PadRefNum = fMap->GetPadNum(PadRef);

            // If this is an FPN channel and we should save it
            if (fMap->IsFPNchannel(PadRef)) {
               if (fSaveFPN)
                  saveFPN(*frame, PadRef, fRawEvent);

               // If this is not an FPN channel add it to the event
            } else if (PadRefNum != -1 && fMap->IsInhibited(PadRefNum) == AtMap::InhibitType::kNone) {
               savePad(*frame, PadRef, fRawEvent, fileIdx);
            } // End check this is a pad to unpack (not FPN)
         }    // End loop over channel
      }       // End loop over aget
   }          // End loop over frame
}
void AtGRAWUnpacker::ProcessBasicFile(Int_t fileIdx)
{

   GETBasicFrame *basicFrame = fDecoder[fileIdx]->GetBasicFrame(fTargetFrameID);

   if (basicFrame == nullptr) {
      fRawEvent->SetIsGood(kFALSE);
      LOG(error) << "Basic frame was null! Skipping event " << fEventID;
      return;
   }
   LOG(debug) << "Looking for " << fTargetFrameID << " found " << basicFrame->GetEventID();

   fCurrentEventID[fileIdx] = basicFrame->GetEventID();
   Int_t iCobo = basicFrame->GetCoboID();
   Int_t iAsad = basicFrame->GetAsadID();

   for (Int_t iAget = 0; iAget < 4; iAget++) {
      for (Int_t iCh = 0; iCh < 68; iCh++) {
         AtPadReference PadRef = {iCobo, iAsad, iAget, iCh};

         // If this is an FPN channel and we should save it
         if (fMap->IsFPNchannel(PadRef)) {
            if (fSaveFPN)
               saveFPN(*basicFrame, PadRef, fRawEvent);
            continue;
         }

         auto PadNum = fMap->GetPadNum(PadRef);
         if (PadNum != -1 && fMap->IsInhibited(PadNum) == AtMap::InhibitType::kNone)
            savePad(*basicFrame, PadRef, fRawEvent, fileIdx);

      } // End loop over channel
   }    // End loop over aget
}

void AtGRAWUnpacker::savePad(GETBasicFrame &frame, AtPadReference PadRef, AtRawEvent *event, Int_t fileIdx)
{
   auto PadRefNum = fMap->GetPadNum(PadRef);
   auto PadCenterCoord = fMap->CalcPadCenter(PadRefNum);

   AtPad *pad = nullptr;
   {
      // Ensure the threads aren't both trying to create pads at the same time
      std::lock_guard<std::mutex> lk(fRawEventMutex);
      pad = fRawEvent->AddPad(PadRefNum);
   }

   pad->SetPadCoord(PadCenterCoord);
   pad->SetValidPad(true);
   fillPadAdc(frame, PadRef, pad);

   if (fIsSubtractFPN)
      doFPNSubtraction(frame, *fPedestal[fileIdx], *pad, fMap->GetNearestFPN(PadRef));
   else if (fIsBaseLineSubtraction)
      doBaselineSubtraction(*pad);

   if (fIsSaveLastCell) {
      saveLastCell(*pad, frame.GetLastCell(PadRef.aget));
   }
}

void AtGRAWUnpacker::fillPadAdc(GETBasicFrame &frame, AtPadReference PadRef, AtPad *pad)
{
   Int_t *rawadc = frame.GetSample(PadRef.aget, PadRef.ch);
   for (Int_t iTb = 0; iTb < 512; iTb++)
      pad->SetRawADC(iTb, rawadc[iTb]);
}

void AtGRAWUnpacker::saveFPN(GETBasicFrame &frame, AtPadReference PadRef, AtRawEvent *event)
{
   AtPad *pad = nullptr;
   {
      std::lock_guard<std::mutex> lk(fRawEventMutex);
      pad = fRawEvent->AddFPN(PadRef);
   }

   pad->SetValidPad(kTRUE);
   fillPadAdc(frame, PadRef, pad);

   if (fIsSaveLastCell)
      saveLastCell(*pad, frame.GetLastCell(PadRef.aget));
}

void AtGRAWUnpacker::saveLastCell(AtPad &pad, Double_t lastCell)
{
   auto lastCellPad = dynamic_cast<AtPadValue *>(pad.AddAugment("lastCell", std::make_unique<AtPadValue>()));
   lastCellPad->SetValue(lastCell);
}
void AtGRAWUnpacker::doFPNSubtraction(GETBasicFrame &basicFrame, AtPedestal &pedestal, AtPad &pad,
                                      AtPadReference fpnRef)
{
   pedestal.SubtractPedestal(512, basicFrame.GetSample(fpnRef.aget, fpnRef.ch), pad.fRawAdc.data(), pad.fAdc.data(),
                             fFPNSigmaThreshold);
   pad.SetPedestalSubtracted(true);
}

void AtGRAWUnpacker::doBaselineSubtraction(AtPad &pad)
{
   auto &rawadc = pad.GetRawADC();
   Double_t baseline = std::accumulate(rawadc.begin() + 5, rawadc.begin() + 25, 0.0) / 20.;
   for (Int_t iTb = 0; iTb < 512; iTb++) {
      pad.SetADC(iTb, rawadc[iTb] - baseline);
   }
   pad.SetPedestalSubtracted(true);
}

void AtGRAWUnpacker::SetPseudoTopologyFrame(Int_t asadMask, Bool_t check)
{
   for (auto &decoder : fDecoder)
      decoder->SetPseudoTopologyFrame(asadMask, check);
}

Long64_t AtGRAWUnpacker::GetNumEvents()
{
   if (fNumEvents == -1 && fCheckNumEvents)
      FindAndSetNumEvents();
   return fNumEvents;
}

AtGRAWUnpacker::CoboAndEvent AtGRAWUnpacker::GetLastEvent(Int_t fileIdx)
{
   GETBasicFrame *basicFrame = fDecoder[fileIdx]->GetBasicFrame(-1);
   bool atEnd = false;
   CoboAndEvent coboInfo;
   coboInfo.first = basicFrame->GetCoboID();
   coboInfo.second = basicFrame->GetEventID();
   while (!atEnd) {
      basicFrame = fDecoder[fileIdx]->GetBasicFrame(-1);
      if (basicFrame != nullptr) {
         coboInfo.second = basicFrame->GetEventID();
      } else {
         // std::cout << "End of file reached!" << std::endl;
         // std::cout << "cobo " << coboInfo.first << " asad " << asad << " final event " << coboInfo.second <<
         // std::endl;
         atEnd = true;
      }
   }
   return coboInfo;
}

ClassImp(AtGRAWUnpacker);
