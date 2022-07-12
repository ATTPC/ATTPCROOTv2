#include "AtLinkDAQTask.h"

#include "AtRawEvent.h"
#include "AtRunAna.h"

#include <FairLogger.h>
#include <FairRootFileSink.h>
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairSink.h> // for FairSink
#include <FairTask.h>

#include <TChain.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TGraph.h>
#include <TMathBase.h>
#include <TObject.h>

#include "HTTimestamp.h"

#include <algorithm>
#include <iostream>
#include <memory>

bool AtLinkDAQTask::SetInputTree(TString fileName, TString treeName)
{
   if (evtTree != nullptr) {
      LOG(error) << "HiRAEVT Tree was already set! Add more trees using the "
                 << "function AddInputTree.";
      return false;
   }
   // Add the tree to the input
   evtTree = std::make_unique<TChain>(treeName);

   // If there is a tree with the correct name in the file
   return AddInputTree(fileName);
}

bool AtLinkDAQTask::AddInputTree(TString fileName)
{
   if (evtTree == nullptr) {
      LOG(fatal) << "HiRAEVT TChain was not initialized  using the "
                 << "function SetInputTree.";
      return false;
   }

   return (evtTree->Add(fileName, -1) == 1);
}

InitStatus AtLinkDAQTask::Init()
{
   auto *run = dynamic_cast<AtRunAna *>(FairRunAna::Instance());
   if (run == nullptr)
      LOG(fatal) << "Run must be of type AtRunAna or a derived type!";

   // Register the input and output branches with the IO manager
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(ERROR) << "Cannot find RootManager!";
      return kERROR;
   }

   fInputEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
   if (fInputEventArray == nullptr) {
      LOG(ERROR) << "Cannot find AtRawEvent array in branch " << fInputBranchName << "!";
      return kERROR;
   }

   // Set the branch addresses for the HiRAEVT detectors
   if (evtTree == nullptr) {
      LOG(error) << "HiRAEVT tree was never initialized!";
      return kERROR;
   }
   std::cout << "EVT address: " << evtTree.get() << std::endl;
   evtTree->SetBranchAddress(fEvtTimestampName, &fEvtTS);

   // Create the clone of the HiRA tree in the new file
   fEvtOutputFile = new TFile(fEvtOutputFileName, "RECREATE"); // NOLINT (owned by ROOT)
   if (fEvtOutputFile->IsZombie()) {
      LOG(fatal) << "Failed to open output file: " << fEvtOutputFileName;
      return kERROR;
   }
   fEvtOutputFile->cd();
   fEvtOutputTree = evtTree->CloneTree(0);

   return kSUCCESS;
}

void AtLinkDAQTask::DoFirstEvent()
{
   evtTree->GetEntry(0);
   fEvtTimestamp = fEvtTS->GetTimestamp();
   fTpcTimestamp = fRawEvent->GetTimestamps();

   LOG(info) << "Initial timestamps: " << fTpcTimestamp.at(fTpcTimestampIndex) << " " << fEvtTimestamp;
   kFirstEvent = false;
   AtRunAna::Instance()->MarkFill(false);
   fEvtTreeIndex = 1;
   fTpcTreeIndex = 1;

   // Get the number of timestamps in the the AtRawEvent
   // And create the graphs to plot
   auto numTimestamps = fRawEvent->GetTimestamps().size();
   for (int i = 0; i < numTimestamps; ++i) {
      fGrDataRatio.emplace_back();
      fGrDataAbs.emplace_back();
   }
}

bool AtLinkDAQTask::UpdateTimestamps()
{
   if (evtTree->GetEntry(fEvtTreeIndex) <= 0)
      return false;

   fEvtTreeIndex++;
   fTpcTreeIndex++;
   fOldEvtTimestamp = fEvtTimestamp;
   fOldTpcTimestamp = fTpcTimestamp;
   fEvtTimestamp = fEvtTS->GetTimestamp();
   fTpcTimestamp = fRawEvent->GetTimestamps();
   if (fTpcTimestamp.size() < fTpcTimestampIndex)
      return false;

   return true;
}

void AtLinkDAQTask::ResetFlags()
{
   auto *run = dynamic_cast<AtRunAna *>(FairRunAna::Instance());
   kFillEvt = run->GetMarkFill();
   kCorruptedTimestamp = false;
}

// return < 0 -> Extra evt event
// return == 0 -> match
// return > 0 -> extra TPC
Int_t AtLinkDAQTask::CheckMatch()
{
   if (kCorruptedTimestamp) {
      // Get the difference in the TPC and EVT timestamps
      // diff = evt - tpc;
      Double_t diff = static_cast<double>(fEvtTimestamp) - static_cast<double>(fTpcTimestamp[fTpcTimestampIndex]);
      diff -= fDifferenceOffset;

      if (TMath::Abs(diff) < fCorruptedSearchRadius)
         return 0;

      LOG(info) << "TPC Timestamp: " << fTpcTimestamp.at(fTpcTimestampIndex) << std::endl
                << "EVT Timestamp: " << fEvtTimestamp << std::endl
                << "diff: " << diff << std::endl;

      return diff;
   } else {
      fIntervalEvt = fEvtTimestamp - fOldEvtTimestamp;
      fIntervalTpc = fTpcTimestamp.at(fTpcTimestampIndex) - fOldTpcTimestamp.at(fTpcTimestampIndex);

      //|intervalTPC-intervalNSCL|/freqRatio
      double scaledInterval = GetScaledInterval(fIntervalEvt, fIntervalTpc);

      if (scaledInterval < fSearchRadius)
         return 0;

      LOG(info) << "TPC Timestamp: " << fTpcTimestamp.at(fTpcTimestampIndex) << std::endl
                << "EVT Timestamp: " << fEvtTimestamp << std::endl
                << "TPC Interval: " << fIntervalTpc << std::endl
                << "EVT Interval: " << fIntervalEvt << std::endl
                << "Scaled Interval: " << scaledInterval << std::endl;

      if (fIntervalTpc > fIntervalEvt * fSearchMean)
         return -1;
      else
         return 1;
   }
}

void AtLinkDAQTask::Exec(Option_t *opt)
{
   ResetFlags();
   // Should already have loaded the event from iomanager. If this is the
   // first event then set the old timestamp and continue without filling
   if (fInputEventArray->GetEntriesFast() == 0)
      return;
   fRawEvent = dynamic_cast<AtRawEvent *>(fInputEventArray->At(0));

   if (kFirstEvent) {
      DoFirstEvent();
      return;
   }

   // Grab both timestamps for this event, set old timestamp and update counter
   if (!UpdateTimestamps()) {
      LOG(warn) << "Failed to update timestamps. Skipping event";
      AtRunAna::Instance()->MarkFill(false);
      kFillEvt = false;

      return;
   }

   // Check for corrupted timestamp data (All FFFFs in a word)
   ULong64_t upperInt = fTpcTimestamp.at(fTpcTimestampIndex) & 0xFFFFFFFF00000000;
   kCorruptedTimestamp = (upperInt == 0xFFFFFFFF00000000);
   if (kCorruptedTimestamp) {
      LOG(warn) << "Corrupted timestamp " << std::hex << fTpcTimestamp.at(fTpcTimestampIndex) << " correcting to "
                << (fTpcTimestamp.at(fTpcTimestampIndex) & 0x00000000FFFFFFFF) << " EVT: " << fEvtTimestamp << std::dec;

      fTpcTimestamp.at(fTpcTimestampIndex) &= 0x00000000FFFFFFFF;
   }

   // Compare both timestamps with their old ones. If it is negative, we must
   // have taken an event before the clocks cleared so reset the old TS
   // and continue without filling
   if (fEvtTimestamp < fOldEvtTimestamp ||
       fTpcTimestamp.at(fTpcTimestampIndex) < fOldTpcTimestamp.at(fTpcTimestampIndex)) {
      LOG(warn) << "Timestamp was reset between EVT event " << fEvtTreeIndex - 2 << " and " << fEvtTreeIndex - 1
                << " reseting old timestamps and "
                << " continuing.";
      LOG(warn) << "EVT TS old: " << fOldEvtTimestamp << " EVT TS new: " << fEvtTimestamp;
      LOG(warn) << "TPC TS old: " << fOldTpcTimestamp.at(fTpcTimestampIndex)
                << " TPC TS new: " << fTpcTimestamp.at(fTpcTimestampIndex);
      AtRunAna::Instance()->MarkFill(false);
      kFillEvt = false;
      return;
   }

   // Compare intervalTPC/intervalNSCL to the search mean and radius
   // If it lies within, then fill the HiRAET tree, upadate the old timestamps,
   // and return
   auto match = CheckMatch();
   // LOG(info) << match;
   if (match == 0) {
      // Record timestamps
      if (fDifferenceOffset == 0)
         fDifferenceOffset = (double)fEvtTimestamp - (double)fTpcTimestamp[fTpcTimestampIndex];
      Fill();
      return;
   }

   /***** If we have an extra EVT event *****/
   LOG(warn) << "Extra event, match number: " << match;
   if (match < 0) {

      LOG(info) << "Extra EVT event found, searching for matching event";

      // Loop through until we hit the last entry of the tree, or until
      // we can match the event interval,
      auto currentIndex = fEvtTreeIndex;
      auto maxIndex = currentIndex + 3;
      // auto currentTS = fEvtTimestamp;
      if (maxIndex > evtTree->GetEntries())
         maxIndex = evtTree->GetEntries();

      // Change to a loop until EVT timestamp is further along
      while (currentIndex < maxIndex) {
         // Get the next NSCL event and associated interval
         evtTree->GetEntry(currentIndex++);
         fEvtTimestamp = fEvtTS->GetTimestamp();

         // Check for a match
         if (CheckMatch() == 0) {
            LOG(info) << "Found match!" << std::endl;
            fEvtTreeIndex = currentIndex;
            Fill();
            return;
         }
      } // while(currentIndex < maxIndex)

      LOG(error) << "Did not find match in next three events" << std::endl;
      AtRunAna::Instance()->MarkFill(false);
      kFillEvt = false;

   } // if we had an extra EVT event

   /**** We have an extra TPC event *******/
   else {
      LOG(info) << "Extra TPC  event found, searching for matching event!";

      AtRunAna::Instance()->MarkFill(false);
      kFillEvt = false;
      // Reset the evtIndex so we're still looking at the same event
      fEvtTreeIndex--;
      return;
   }
}

void AtLinkDAQTask::Fill()
{
   // Checkl to see if we should actually fill
   if (!kFillEvt)
      return;

   // Get the EVt timestamp interval
   double evtInterval = fEvtTimestamp - fOldEvtTimestamp;
   for (int i = 0; i < fGrDataRatio.size(); ++i) {
      // Get signed scaled interval for this timestamp
      double intDiff = evtInterval;

      if (i < fTpcTimestamp.size() && i < fOldTpcTimestamp.size())
         intDiff -= static_cast<double>(fTpcTimestamp.at(i) - fOldTpcTimestamp.at(i));

      fGrDataAbs.at(i).push_back(intDiff);

      intDiff /= evtInterval;
      fGrDataRatio.at(i).push_back(intDiff);
   }
   fEvtOutputTree->Fill();
}

void AtLinkDAQTask::Finish()
{
   auto run = dynamic_cast<AtRunAna *>(FairRunAna::Instance());
   auto outFile = dynamic_cast<FairRootFileSink *>(run->GetSink());
   if (outFile) {
      LOG(info) << "Adding TTree " << fEvtOutputTree->GetName() << " as friend to output tree "
                << outFile->GetOutTree()->GetName();
      outFile->GetOutTree()->AddFriend(fEvtOutputTree);
   }

   LOG(info) << "Writing graph and tree";

   // Create Graphs and fill
   for (int index = 0; index < fGrDataRatio.size(); ++index) {
      fEvtOutputFile->cd();
      auto gr = std::make_unique<TGraph>(fGrDataRatio[index].size());
      auto gr2 = std::make_unique<TGraph>(fGrDataAbs[index].size());
      gr->SetTitle(TString::Format("Relative intercal difference. TS_%d", index));
      gr2->SetTitle(TString::Format("Abs interval difference. TS_%d", index));

      for (int i = 0; i < fGrDataRatio[index].size(); ++i) {
         gr->SetPoint(i, i, fGrDataRatio[index].at(i));
         gr2->SetPoint(i, i, fGrDataAbs[index].at(i));
      }
      gr->Write(TString::Format("Ratio_%d", index));
      gr2->Write(TString::Format("Abs_%d", index));
   }

   fEvtOutputFile->cd();
   fEvtOutputTree->Write();
   fEvtOutputFile->Close();
}

Double_t AtLinkDAQTask::GetScaledInterval(ULong64_t intervalEvt, ULong64_t fIntercalTPC)
{
   return TMath::Abs(static_cast<double>(intervalEvt) - static_cast<double>(fIntercalTPC)) / fSearchMean;
}
