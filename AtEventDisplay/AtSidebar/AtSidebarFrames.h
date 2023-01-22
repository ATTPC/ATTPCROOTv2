#ifndef ATSIDEBARFRAMES_H
#define ATSIDEBARFRAMES_H

#include "AtDataObserver.h"
#include "AtViewerManagerSubject.h"

#include <TGComboBox.h>
#include <TGFrame.h>
#include <TGLabel.h>
#include <TGNumberEntry.h>

class TGTableLayout;

/**
 * Base class something that can be added to the sidebar. It is a frame that will be added to the
 * sidebar in the order in which they are added to the AtEventSidebar.
 */
class AtSidebarFrame : public TGCompositeFrame {
protected:
   /// Option to disable picture buttons since they break some machines
   bool kUsePictureButtons{true};

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

   /// @brief Use text only buttons instead of picture buttons.
   void UsePictureButtons(bool val = true) { kUsePictureButtons = val; }
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

private:
   TString GetFileName(TString filePath);
};
class AtSidebarPadControl : public AtVerticalSidebarFrame, public DataHandling::Observer {
   DataHandling::AtPadNum &fPadNum;

   TGHorizontalFrame *fCurrentPadFrame{nullptr};
   TGLabel *fCurrentPadLabel{nullptr};
   TGNumberEntry *fCurrentPadEntry{nullptr};
   TGTextButton *fRedrawPadButton{nullptr};

   TGLabel *fCurrentPadId{nullptr};
   static constexpr char fPadRefString[] = "Pad Ref:[%d,%d,%d,%d]";

public:
   AtSidebarPadControl(DataHandling::AtPadNum &padNum, const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1,
                       UInt_t options = 0, Pixel_t back = GetDefaultFrameBackground());
   ~AtSidebarPadControl();

   void Update(DataHandling::Subject *changedSubject) override;
   void FillFrame() override;

   void SelectPad(); //< Pad TGNumberEntry/Button callback
};

class AtSidebarEventControl : public AtVerticalSidebarFrame,
                              public DataHandling::Observer {
private:
   DataHandling::AtTreeEntry &fEntryNumber;

   TGHorizontalFrame *fCurrentEventFrame{nullptr};
   TGLabel *fCurrentEventLabel{nullptr};
   TGNumberEntry *fCurrentEventEntry{nullptr};
   TGTextButton *fRerunButton{nullptr};

   TGHorizontalFrame *fButtonFrame{nullptr};

public:
   AtSidebarEventControl(DataHandling::AtTreeEntry &entryNum, const TGWindow *p = nullptr, UInt_t w = 1, UInt_t h = 1,
                         UInt_t options = 0, Pixel_t back = GetDefaultFrameBackground());
   ~AtSidebarEventControl();

   void Update(DataHandling::Subject *changedSubject) override;
   void FillFrame() override;

   void SelectEvent(); //< Event TGNumberEntry callback
   void RedrawEvent(); //< Event TGNumberEntry callback
};

class AtSidebarBranchControl : public AtVerticalSidebarFrame, public DataHandling::Observer {
private:
   TGVerticalFrame *fLabels;
   TGVerticalFrame *fBoxes;
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
