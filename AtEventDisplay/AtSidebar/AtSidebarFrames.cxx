#include "AtSidebarFrames.h"
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtMap.h"           // for AtMap
#include "AtPadReference.h"  // for AtPadReference
#include "AtViewerManager.h" // for AtViewerManager

#include <FairRootManager.h> // for FairRootManager
#include <FairRunAna.h>      // for FairRunAna

#include <TChain.h>        // for TChain
#include <TFile.h>         // for TFile
#include <TGButton.h>      // for TGTextButton, TGButton, TGPictureButton
#include <TGClient.h>      // for TGClient, gClient
#include <TGComboBox.h>    // for TGComboBox
#include <TGLabel.h>       // for TGLabel
#include <TGLayout.h>      // for TGLayoutHints, kLHintsExpandX, kLHints...
#include <TGNumberEntry.h> // for TGNumberEntry, TGNumberFormat, TGNumbe...
#include <TSystem.h>       // for TSystem, gSystem

#include <algorithm> // for find
#include <iterator>  // for begin, distance, end
#include <memory>    // for allocator, allocator_traits<>::value_type

class TGWindow;
namespace DataHandling {
class AtSubject;
}

TString AtSidebarRunInfo::GetFileName(TString filePath)
{
   TString tok;
   Ssiz_t loc = 0;
   TString name;
   while (filePath.Tokenize(tok, loc, "/")) {
      name = tok;
      continue;
   }

   return name;
}
void AtSidebarRunInfo::FillFrame()
{
   TString Infile = "Input file : ";
   TFile *file = FairRootManager::Instance()->GetInChain()->GetFile();

   fRunFile = new TGLabel(this, (Infile + file->GetName()).Data());

   // If the file path is too long to fit in the sidebar, just grab the file name
   if (fRunFile->GetWidth() > this->GetWidth()) {
      delete fRunFile;
      TString fileName = GetFileName(file->GetName());
      fRunFile = new TGLabel(this, (Infile + fileName).Data());
   }

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
AtSidebarPadControl::AtSidebarPadControl(DataHandling::AtPadNum &entryNum, const TGWindow *p, UInt_t w, UInt_t h,
                                         UInt_t options, Pixel_t back)
   : AtVerticalSidebarFrame(p, w, h, options, back), fPadNum(entryNum)
{
   fPadNum.Attach(this);
}
AtSidebarPadControl::~AtSidebarPadControl()
{
   fPadNum.Detach(this);
}

void AtSidebarPadControl::FillFrame()
{
   /**** Pad Selection *****/
   fCurrentPadFrame = new TGHorizontalFrame(this);
   fCurrentPadLabel = new TGLabel(fCurrentPadFrame, "Current Pad: ");

   fCurrentPadEntry =
      new TGNumberEntry(fCurrentPadFrame, 0., 6, -1, TGNumberFormat::kNESInteger, TGNumberFormat::kNEAAnyNumber,
                        TGNumberFormat::kNELLimitMinMax, -1, AtViewerManager::Instance()->GetMap()->GetNumPads());

   fCurrentPadEntry->Connect("ValueSet(Long_t)", "AtSidebarPadControl", this, "SelectPad()");

   fCurrentPadFrame->AddFrame(fCurrentPadLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fCurrentPadFrame->AddFrame(fCurrentPadEntry);

   fRedrawPadButton = new TGTextButton(fCurrentPadFrame, "Redraw Pad");
   fRedrawPadButton->Connect("Clicked()", "AtSidebarPadControl", this, "SelectPad()");
   fCurrentPadFrame->AddFrame(fRedrawPadButton, new TGLayoutHints(kLHintsCenterY, 1, 1, 1, 1));
   this->AddFrame(fCurrentPadFrame, new TGLayoutHints());

   fCurrentPadId = new TGLabel(this, fPadRefString);
   this->AddFrame(fCurrentPadId, new TGLayoutHints(kLHintsCenterX | kLHintsExpandX));
}

void AtSidebarPadControl::Update(DataHandling::AtSubject *changedSubject)
{
   if (changedSubject == &fPadNum && fCurrentPadEntry) {
      fCurrentPadEntry->SetIntNumber(fPadNum.Get());
      auto ref = AtViewerManager::Instance()->GetMap()->GetPadRef(fPadNum.Get());
      fCurrentPadId->SetText(TString::Format(fPadRefString, ref.cobo, ref.asad, ref.aget, ref.ch));
      // this->Layout();
   }
}
void AtSidebarPadControl::SelectPad()
{
   fPadNum.Set(fCurrentPadEntry->GetIntNumber(), false);
   fPadNum.Notify();
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

void AtSidebarEventControl::Update(DataHandling::AtSubject *changedSubject)
{
   if (changedSubject == &fEntryNumber && fCurrentEventEntry)
      fCurrentEventEntry->SetIntNumber(fEntryNumber.Get());
}

void AtSidebarEventControl::SelectEvent()
{
   fEntryNumber.Set(fCurrentEventEntry->GetIntNumber());
}

void AtSidebarEventControl::RedrawEvent()
{
   fEntryNumber.Set(fEntryNumber.Get());
}

void AtSidebarEventControl::FillFrame()
{
   /**** Event Selection *****/
   fCurrentEventFrame = new TGHorizontalFrame(this);
   fCurrentEventLabel = new TGLabel(fCurrentEventFrame, "Current Event: ");

   fCurrentEventEntry = new TGNumberEntry(fCurrentEventFrame, 0., 6, -1, TGNumberFormat::kNESInteger,
                                          TGNumberFormat::kNEANonNegative, TGNumberFormat::kNELLimitMinMax, 0,
                                          FairRootManager::Instance()->GetInChain()->GetEntriesFast());

   fCurrentEventEntry->Connect("ValueSet(Long_t)", "AtSidebarEventControl", this, "SelectEvent()");

   fCurrentEventFrame->AddFrame(fCurrentEventLabel, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   fCurrentEventFrame->AddFrame(fCurrentEventEntry);

   this->AddFrame(fCurrentEventFrame, new TGLayoutHints(kLHintsExpandX));

   /*** Button Frame ****/
   fButtonFrame = new TGHorizontalFrame(this); // Navigation button frame
   {
      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));

      TGButton *b = nullptr;
      if (kUsePictureButtons)
         b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_left.gif"));
      else
         b = new TGTextButton(fButtonFrame, "Prev");

      fButtonFrame->AddFrame(b, new TGLayoutHints(kLHintsCenterY, 1, 1, 1, 1));
      b->Connect("Clicked()", "AtViewerManager", AtViewerManager::Instance(), "PrevEvent()");

      if (kUsePictureButtons)
         b = new TGPictureButton(fButtonFrame, gClient->GetPicture(icondir + "arrow_right.gif"));
      else
         b = new TGTextButton(fButtonFrame, "Next");
      fButtonFrame->AddFrame(b, new TGLayoutHints(kLHintsCenterY, 1, 1, 1, 1));
      b->Connect("Clicked()", "AtViewerManager", AtViewerManager::Instance(), "NextEvent()");

      fRerunButton = new TGTextButton(fButtonFrame, "Rerun Event");
      fRerunButton->Connect("Clicked()", "AtSidebarEventControl", this, "RedrawEvent()");
      fButtonFrame->AddFrame(fRerunButton, new TGLayoutHints(kLHintsCenterY, 1, 1, 1, 1));
   }
   this->AddFrame(fButtonFrame, new TGLayoutHints(kLHintsCenterX));
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

void AtSidebarBranchControl::FillFrame()
{
   // this->AddFrame(new TGLabel(this, "Selected Branches"), new TGLayoutHints(kLHintsCenterX));
   auto frame = new TGHorizontalFrame(this);

   fLabels = new TGVerticalFrame(frame);
   fBoxes = new TGVerticalFrame(frame);

   FillBranchFrame("Raw Event: ", "AtRawEvent");
   FillBranchFrame("Event: ", "AtEvent");
   FillBranchFrame("Pattern Event: ", "AtPatternEvent");

   // Resize boxes
   for (auto &[name, box] : fBranchBoxes)
      box->SetWidth(125);

   frame->AddFrame(fLabels);
   frame->AddFrame(fBoxes);
   this->AddFrame(frame, new TGLayoutHints(kLHintsExpandX));
}

/**
 *
 * Creates a frame with a dropdown with every branch that matches the type className. When a newly
 * thing is selected it calls the callback SelectedClassName(Int_t)
 */
void AtSidebarBranchControl::FillBranchFrame(std::string label, std::string className)
{
   auto &branchMap = AtViewerManager::Instance()->GetBranchNames();
   if (branchMap.find(className) == branchMap.end())
      return;

   auto labelf = new TGLabel(fLabels, label.data());
   fLabels->AddFrame(labelf, new TGLayoutHints(kLHintsRight));

   auto &branchNames = branchMap.at(className);
   fBranchBoxes[className] = new TGComboBox(fBoxes);
   for (int i = 0; i < branchNames.size(); ++i)
      fBranchBoxes[className]->AddEntry(branchNames[i], i);
   fBranchBoxes[className]->Connect("Selected(Int_t)", "AtSidebarBranchControl", this,
                                    TString::Format("Selected%s(Int_t)", className.data()));
   fBranchBoxes[className]->Select(0);

   fBranchBoxes[className]->SetHeight(labelf->GetHeight());
   fBoxes->AddFrame(fBranchBoxes[className], new TGLayoutHints(kLHintsExpandX));
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

void AtSidebarBranchControl::Update(DataHandling::AtSubject *changedSubject)
{
   auto &branchNames = AtViewerManager::Instance()->GetBranchNames();

   for (auto &[name, branch] : fBranches)
      if (changedSubject == &branch)
         fBranchBoxes[name]->Select(GetIndex(branch.GetBranchName(), branchNames.at(name)));
}
