#include "AtSidebarFrames.h"

#include <FairRootManager.h>
#include <FairRunAna.h>

#include <TFile.h>
#include <TGButton.h>

#include <AtViewerManager.h>

#include <iostream>

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

AtSidebarEventControl::AtSidebarEventControl(DataHandling::AtEntryNumber &entryNum,
                                             const TGWindow *p, UInt_t w, UInt_t h, UInt_t options,
                                             Pixel_t back)
   : AtVerticalSidebarFrame(p, w, h, options, back), fEntryNumber(entryNum)
{
   fEntryNumber.Attach(this);
}
AtSidebarEventControl::~AtSidebarEventControl()
{
   fEntryNumber.Detach(this);
}

void AtSidebarEventControl::Update(DataHandling::Subject *changedSubject)
{
   if (changedSubject == &fEntryNumber && fCurrentEventEntry)
      fCurrentEventEntry->SetIntNumber(fEntryNumber.Get());
}

void AtSidebarEventControl::SelectEvent()
{
   fEntryNumber.Set(fCurrentEventEntry->GetIntNumber());
   // AtViewerManager::Instance()->GotoEvent(fCurrentEventEntry->GetIntNumber());
}

void AtSidebarEventControl::FillFrame()
{
   fCurrentEventFrame = new TGHorizontalFrame(this);
   fCurrentEventLabel = new TGLabel(fCurrentEventFrame, "Current Event: ");

   fCurrentEventEntry = new TGNumberEntry(fCurrentEventFrame, 0., 6, -1, TGNumberFormat::kNESInteger,
                                          TGNumberFormat::kNEANonNegative, TGNumberFormat::kNELLimitMinMax, 0,
                                          FairRootManager::Instance()->GetInChain()->GetEntriesFast());

   fCurrentEventEntry->Connect("ValueSet(Long_t)", "AtSidebarEventControl", this, "SelectEvent()");

   fCurrentEventFrame->AddFrame(fCurrentEventLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fCurrentEventFrame->AddFrame(fCurrentEventEntry);

   fButtonFrame = new TGHorizontalFrame(this); // Navigation button frame
   {
      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));

      auto b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_left.gif"));
      fButtonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtViewerManager", AtViewerManager::Instance(), "PrevEvent()");

      b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_right.gif"));
      fButtonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtViewerManager", AtViewerManager::Instance(), "NextEvent()");
   }

   this->AddFrame(fCurrentEventFrame);
   this->AddFrame(fButtonFrame);
}

AtSidebarBranchControl::AtSidebarBranchControl(DataHandling::AtBranchName &rawEvent,
                                               DataHandling::AtBranchName &event,
                                               DataHandling::AtBranchName &patternEvent,
                                               const TGWindow *p, UInt_t w, UInt_t h,
                                               UInt_t options, Pixel_t back)
   : AtVerticalSidebarFrame(p, w, h, options, back), fRawEvent(rawEvent), fEvent(event),
     fPatternEvent(patternEvent)
{
   fRawEvent.Attach(this);
   fEvent.Attach(this);
   fPatternEvent.Attach(this);
}

AtSidebarBranchControl::~AtSidebarBranchControl()
{
   fRawEvent.Detach(this);
   fEvent.Detach(this);
   fPatternEvent.Detach(this);
}

int AtSidebarBranchControl::GetIndex(std::string val, const std::vector<TString> &vec)
{
   auto iter = std::find(begin(vec), end(vec), val);
   if (iter == vec.end())
      return -1;
   else
      return std::distance(begin(vec), iter);
}

void AtSidebarBranchControl::SelectedAtRawEvent(Int_t ind)
{
   std::cout << "Selecting " << ind << " was  " << fBranchBoxes[0]->GetSelected() << std::endl;
   if (ind < 0)
      return;
   auto name = AtViewerManager::Instance()->GetBranchNames().at("AtRawEvent").at(ind).Data();
   fBranchBoxes[0]->Select(ind);
   std::cout << "Selectring branch " << name << std::endl;
   fRawEvent.SetBranchName(name);
}

void AtSidebarBranchControl::SelectedAtEvent(Int_t ind)
{
   std::cout << "Selecting " << ind << " was  " << fBranchBoxes[1]->GetSelected() << std::endl;
   if (ind < 0)
      return;
   auto name = AtViewerManager::Instance()->GetBranchNames().at("AtEvent").at(ind).Data();
   fBranchBoxes[1]->Select(ind);
   std::cout << "Selectring branch " << name << std::endl;
   fEvent.SetBranchName(name);
}

void AtSidebarBranchControl::SelectedAtPatternEvent(Int_t ind)
{

   if (ind < 0)
      return;
   fBranchBoxes[2]->Select(ind);
   fPatternEvent.SetBranchName(
      AtViewerManager::Instance()->GetBranchNames().at("AtPatternEvent").at(ind).Data());
}

void AtSidebarBranchControl::Update(DataHandling::Subject *changedSubject)
{
   auto &branchNames = AtViewerManager::Instance()->GetBranchNames();

   if (changedSubject == &fRawEvent)
      fBranchBoxes[0]->Select(GetIndex(fRawEvent.GetBranchName(), branchNames.at("AtRawEvent")));
   if (changedSubject == &fEvent)
      fBranchBoxes[1]->Select(GetIndex(fEvent.GetBranchName(), branchNames.at("AtEvent")));
   if (changedSubject == &fPatternEvent)
      fBranchBoxes[2]->Select(
         GetIndex(fPatternEvent.GetBranchName(), branchNames.at("AtPatternEvent")));
}

/**
 *
 * Creates a frame with a dropdown with every branch that matches the type className. When a newly
 * thing is selected it calls the callback SelectedClassName(Int_t)
 */
void AtSidebarBranchControl::FillBranchFrame(std::string label, std::string className)
{
   auto frame = new TGHorizontalFrame(this);
   frame->AddFrame(new TGLabel(frame, label.data()));

   auto &branchNames = AtViewerManager::Instance()->GetBranchNames().at(className);
   fBranchBoxes.push_back(new TGComboBox(frame));
   for (int i = 0; i < branchNames.size(); ++i)
      fBranchBoxes.back()->AddEntry(branchNames[i], i);
   fBranchBoxes.back()->Connect("Selected(Int_t)", "AtSidebarBranchControl", this,
                                TString::Format("Selected%s(Int_t)", className.data()));
   fBranchBoxes.back()->Select(0);

   frame->AddFrame(fBranchBoxes.back(),
                   new TGLayoutHints(kLHintsExpandX | kLHintsCenterY | kLHintsExpandY));
   this->AddFrame(frame, new TGLayoutHints(kLHintsExpandX));
}

void AtSidebarBranchControl::FillFrame()
{
   auto &branchNames = AtViewerManager::Instance()->GetBranchNames();
   this->AddFrame(new TGLabel(this, "Selected Branches"), new TGLayoutHints(kLHintsCenterX));

   FillBranchFrame("Raw Event: ", "AtRawEvent");
   FillBranchFrame("Event: ", "AtEvent");
   FillBranchFrame("Pattern Event: ", "AtPatternEvent");
}
