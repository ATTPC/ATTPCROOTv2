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

AtSidebarEventControl::AtSidebarEventControl(DataHandling::AtTreeEntry &entryNum, const TGWindow *p, UInt_t w, UInt_t h,
                                             UInt_t options, Pixel_t back)
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
   AtViewerManager::Instance()->GotoEvent(fCurrentEventEntry->GetIntNumber());
   // fEntryNumber.Set(fCurrentEventEntry->GetIntNumber());
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

   fRedrawButton = new TGTextButton(fCurrentEventFrame, "Rerun Event");
   fRedrawButton->Connect("Clicked()", "AtSidebarEventControl", this, "SelectEvent()");
   fCurrentEventFrame->AddFrame(fRedrawButton, new TGLayoutHints(kLHintsCenterX | kLHintsCenterY, 1, 1, 1, 1));

   fButtonFrame = new TGHorizontalFrame(this); // Navigation button frame
   {
      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));

      TGButton *b = nullptr;
      if (kUsePictureButtons)
         b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_left.gif"));
      else
         b = new TGTextButton(fButtonFrame, "Prev");

      fButtonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtViewerManager", AtViewerManager::Instance(), "PrevEvent()");

      if (kUsePictureButtons)
         b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_right.gif"));
      else
         b = new TGTextButton(fButtonFrame, "Next");
      fButtonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtViewerManager", AtViewerManager::Instance(), "NextEvent()");
   }

   this->AddFrame(fCurrentEventFrame, new TGLayoutHints(kLHintsExpandX));
   this->AddFrame(fButtonFrame);
}

AtSidebarBranchControl::AtSidebarBranchControl(DataHandling::AtBranch &rawEvent, DataHandling::AtBranch &event,
                                               DataHandling::AtBranch &patternEvent, const TGWindow *p, UInt_t w,
                                               UInt_t h, UInt_t options, Pixel_t back)
   : AtVerticalSidebarFrame(p, w, h, options, back)
{
   fBranches.insert({"AtRawEvent", rawEvent});
   fBranches.insert({"AtEvent", event});
   fBranches.insert({"AtPatternEvent", patternEvent});

   for (auto &[className, branch] : fBranches)
      branch.Attach(this);
}

AtSidebarBranchControl::~AtSidebarBranchControl()
{
   for (auto &[className, branch] : fBranches)
      branch.Detach(this);
}

int AtSidebarBranchControl::GetIndex(TString val, const std::vector<TString> &vec)
{
   auto iter = std::find(begin(vec), end(vec), val);
   if (iter == vec.end())
      return -1;
   else
      return std::distance(begin(vec), iter);
}

void AtSidebarBranchControl::SelectEvent(Int_t ind, TString className)
{
   if (ind < 0)
      return;
   auto name = AtViewerManager::Instance()->GetBranchNames().at(className.Data()).at(ind);
   fBranchBoxes[className]->Select(ind);
   fBranches.at(className).SetBranchName(name);
}

void AtSidebarBranchControl::SelectedAtRawEvent(Int_t ind)
{
   SelectEvent(ind, "AtRawEvent");
}

void AtSidebarBranchControl::SelectedAtEvent(Int_t ind)
{
   SelectEvent(ind, "AtEvent");
}

void AtSidebarBranchControl::SelectedAtPatternEvent(Int_t ind)
{
   SelectEvent(ind, "AtPatternEvent");
}

void AtSidebarBranchControl::Update(DataHandling::Subject *changedSubject)
{
   auto &branchNames = AtViewerManager::Instance()->GetBranchNames();

   for (auto &[name, branch] : fBranches)
      if (changedSubject == &branch)
         fBranchBoxes[name]->Select(GetIndex(branch.GetBranchName(), branchNames.at(name)));
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
   fBranchBoxes[className] = new TGComboBox(frame);
   for (int i = 0; i < branchNames.size(); ++i)
      fBranchBoxes[className]->AddEntry(branchNames[i], i);
   fBranchBoxes[className]->Connect("Selected(Int_t)", "AtSidebarBranchControl", this,
                                    TString::Format("Selected%s(Int_t)", className.data()));
   fBranchBoxes[className]->Select(0);

   frame->AddFrame(fBranchBoxes[className], new TGLayoutHints(kLHintsExpandX | kLHintsCenterY | kLHintsExpandY));
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
