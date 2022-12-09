/**
 * @brief Event display task
 * Used to draw AT-TPC events
 * @author JungWoo Lee (Korea Univ.)
 *         Adapted for AtTPCROOT by Yassid Ayyad (NSCL)
 */
#ifndef AtEventDrawTaskNewNEW_H
#define AtEventDrawTaskNewNEW_H

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <memory>
#include <vector>        // for vector
class AtEventManagerNew; // lines 17-17
class AtHit;             // lines 18-18
class AtMap;             // lines 24-24
class AtRawEvent;        // lines 22-22
class TBuffer;
class TCanvas; // lines 30-30
class TClass;
class TClonesArray;    // lines 31-31
class TEveBoxSet;      // lines 33-33
class TEveLine;        // lines 34-34
class TEvePointSet;    // lines 32-32
class TEveRGBAPalette; // lines 44-44
class TGraph;          // lines 35-35
class TH1D;            // lines 37-37
class TH1F;            // lines 38-38
class TH1I;            // lines 36-36
class TH2F;            // lines 40-40
class TH2Poly;         // lines 41-41
class TH3F;            // lines 42-42
class TMemberInspector;
class TPaletteAxis; // lines 43-43
class TEveElement;

class AtEventDrawTaskNew : public FairTask {
protected:
   TString fRawEventBranchName;
   TString fEventBranchName;

   TClonesArray *fEventArray;
   TClonesArray *fRawEventArray{};

   AtEventManagerNew *fEventManager;
   AtRawEvent *fRawevent;

   std::shared_ptr<AtMap> fDetmap;

   Int_t fThreshold;
   TString fMap;

   TEvePointSet *fHitSet;

   TPaletteAxis *fPadPlanePal;

   Color_t fHitColor;
   Size_t fHitSize;
   Style_t fHitStyle;

   TCanvas *fCvsPadPlane;
   TH2Poly *fPadPlane;
   TCanvas *fCvsPadWave;
   TH1I *fPadWave;

   Int_t fMultiHit{10};
   Int_t fTaskNumber;
   Bool_t fIsRawData;

   // std::vector<TEveLine*> fLineArray;
   std::vector<std::unique_ptr<TEveLine>> fPatternLines;

   TEveRGBAPalette *fRGBAPalette;

public:
   AtEventDrawTaskNew();
   AtEventDrawTaskNew(TString modes);

   ~AtEventDrawTaskNew();

   InitStatus Init();
   void Exec(Option_t *option);
   void Reset();

   // void Set2DPlotRange(Int_t uaIdx);
   void SetMap(std::shared_ptr<AtMap> map) { fDetmap = map; }
   void SetThreshold(Int_t val) { fThreshold = val; }
   void SetHitAttributes(Color_t, Size_t, Style_t);
   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);
   static void SelectPad(const char *rawevt);
   void DrawPad(Int_t TaskNum, Int_t PadNum);
   void SetMultiHit(Int_t hitMax);
   void SetTaskNumber(Int_t taskNum) { fTaskNumber = taskNum; }

private:
   void DrawPadPlane();
   void DrawPadWave();

   AtMap *fAtMapPtr;
   void UpdateCvsPadPlane();
   void UpdateCvsPadWave();

   // Functions for drawing hits
   void DrawHitPoints();
   void DrawRawHits();

   EColor GetTrackColor(int i);

   ClassDef(AtEventDrawTaskNew, 2);
};

#endif
