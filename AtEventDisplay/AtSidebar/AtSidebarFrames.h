#ifndef ATSIDEBARFRAMES_H
#define ATSIDEBARFRAMES_H

#include "AtDataObserver.h"
#include "AtViewerManagerSubject.h"

#include <TGComboBox.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>

/**
 * Base class something that can be added to the sidebar. It is a frame that will be added to the
 * sidebar in the order in which they are added to the AtEventSidebar.
 */
class AtSidebarFrame : public TGCompositeFrame {
protected:
   // Protected constructor because this can only be instantiated as a base class
   AtSidebarFrame(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                  Pixel_t back = GetDefaultFrameBackground())
      : TGCompositeFrame(p, w, h, options | kChildFrame, back)
   {
   }

public:
   /**
    * Called at the end of the Init stage. Is what will create all of the components in the sidebar
    * frame.
    */
   virtual void FillFrame() = 0;
};

class AtVerticalSidebarFrame : public AtSidebarFrame {
public:
   AtVerticalSidebarFrame(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                          Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarFrame(p, w, h, options | kVerticalFrame, back)
   {
   }
};
class AtHorizontalSidebarFrame : public AtSidebarFrame {
public:
   AtHorizontalSidebarFrame(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                            Pixel_t back = GetDefaultFrameBackground())
      : AtSidebarFrame(p, w, h, options | kHorizontalFrame, back)
   {
   }
};

/**
 * Sidebar component with run info
 */
class AtSidebarRunInfo : public AtVerticalSidebarFrame {
private:
   TGLabel *fRunFile{nullptr};
   TGLabel *fRunId{nullptr};
   TGLabel *fRunLength{nullptr};

public:
   AtSidebarRunInfo(const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1, UInt_t options = 0,
                    Pixel_t back = GetDefaultFrameBackground())
      : AtVerticalSidebarFrame(p, w, h, options, back)
   {
   }

   void FillFrame() override;
};

class AtSidebarEventControl : public AtVerticalSidebarFrame, public DataHandling::Observer {
private:
   DataHandling::AtTreeEntry &fEntryNumber;

   TGHorizontalFrame *fButtonFrame{nullptr};
   TGHorizontalFrame *fCurrentEventFrame{nullptr};
   TGLabel *fCurrentEventLabel{nullptr};
   TGNumberEntry *fCurrentEventEntry{nullptr};

public:
   AtSidebarEventControl(DataHandling::AtTreeEntry &entryNum, const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1,
                         UInt_t options = 0, Pixel_t back = GetDefaultFrameBackground());
   ~AtSidebarEventControl();

   void Update(DataHandling::Subject *changedSubject) override;

   void SelectEvent();

   void FillFrame() override;
};

class AtSidebarBranchControl : public AtVerticalSidebarFrame, public DataHandling::Observer {
private:
   std::map<TString, DataHandling::AtBranch &> fBranches;
   std::map<TString, TGComboBox *> fBranchBoxes;
   // std::vector<TGComboBox *> fBranchBoxes;

public:
   AtSidebarBranchControl(DataHandling::AtBranch &rawEvent, DataHandling::AtBranch &event,
                          DataHandling::AtBranch &patternEvent, const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1,
                          UInt_t options = 0, Pixel_t back = GetDefaultFrameBackground());

   ~AtSidebarBranchControl();

   void Update(DataHandling::Subject *changedSubject) override;

   void SelectedAtRawEvent(Int_t);
   void SelectedAtEvent(Int_t);
   void SelectedAtPatternEvent(Int_t);

   void SelectEvent(Int_t ind, TString className);

   void FillFrame() override;

private:
   void FillBranchFrame(std::string name, std::string className);
   int GetIndex(TString, const std::vector<TString> &vec);
};

#endif
