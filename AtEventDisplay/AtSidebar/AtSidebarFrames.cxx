#include "AtSidebarFrames.h"

#include <FairRootManager.h>
#include <FairRunAna.h>

#include <TFile.h>
#include <TGButton.h>

#include <AtEventManagerNew.h>

void AtSidebarRunInfo::FillFrame()
{
   TString Infile = "Input file : ";
   TFile *file = FairRootManager::Instance()->GetInChain()->GetFile();
   Infile += file->GetName();
   fRunFile = new TGLabel(this, Infile.Data());
   this->AddFrame(fRunFile);

   UInt_t RunId = FairRunAna::Instance()->getRunId();
   TString run = "Run Id : ";
   run += RunId;
   fRunId = new TGLabel(this, run.Data());
   this->AddFrame(fRunId);

   TString nevent = "No of events : ";
   nevent += FairRootManager::Instance()->GetInChain()->GetEntriesFast();
   fRunLength = new TGLabel(this, nevent.Data());
   this->AddFrame(fRunLength);
}

void AtSidebarEventControl::FillFrame()
{
   fCurrentEventFrame = new TGHorizontalFrame(this);
   fCurrentEventLabel = new TGLabel(fCurrentEventFrame, "Current Event: ");

   fCurrentEventEntry = new TGNumberEntry(fCurrentEventFrame, 0., 6, -1, TGNumberFormat::kNESInteger,
                                          TGNumberFormat::kNEANonNegative, TGNumberFormat::kNELLimitMinMax, 0,
                                          FairRootManager::Instance()->GetInChain()->GetEntriesFast());

   fCurrentEventEntry->Connect("ValueSet(Long_t)", "AtEventManagerNew", AtEventManagerNew::Instance(), "SelectEvent()");
   AtEventManagerNew::Instance()->SetCurrentEvent(fCurrentEventEntry);

   fCurrentEventFrame->AddFrame(fCurrentEventLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fCurrentEventFrame->AddFrame(fCurrentEventEntry);

   fButtonFrame = new TGHorizontalFrame(this); // Navigation button frame
   {
      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));

      auto b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_left.gif"));
      fButtonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerNew", AtEventManagerNew::Instance(), "PrevEvent()");

      b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_right.gif"));
      fButtonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerNew", AtEventManagerNew::Instance(), "NextEvent()");
   }

   this->AddFrame(fCurrentEventFrame);
   this->AddFrame(fButtonFrame);
}
