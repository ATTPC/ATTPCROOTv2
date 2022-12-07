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
#include <vector>     // for vector
class AtEventManagerNew; // lines 17-17
class AtHit;          // lines 18-18
class AtMap;          // lines 24-24
class AtRawEvent;     // lines 22-22
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
class S800Calc;

class AtEventDrawTaskNew : public FairTask {
protected:
   Bool_t fIs2DPlotRange;
   Bool_t fUnpackHough;
   static const Int_t fNumPads = 1000; // Maximum number of pads to draw for DrawAllPads option

   TString fRawEventBranchName;
   TString fEventBranchName;
   TString fCorrectedEventBranchName;
   TString fPatternEventBranchName;

   TClonesArray *fEventArray;
   TClonesArray *fCorrectedEventArray{};
   TClonesArray *fRawEventArray{};
   TClonesArray *fPatternEventArray{};

   AtEventManagerNew *fEventManager;
   AtRawEvent *fRawevent;

   std::shared_ptr<AtMap> fDetmap;

   Int_t fThreshold;
   TString fMap;

   TEvePointSet *fHitSet;
   TEvePointSet *fCorrectedHitSet;

   TEveBoxSet *fhitBoxSet;

   TPaletteAxis *fPadPlanePal;

   Color_t fHitColor;
   Size_t fHitSize;
   Style_t fHitStyle;
   Size_t fVertexSize;
   Size_t fVertexStyle;

   TCanvas *fCvsPadPlane;
   TH2Poly *fPadPlane;
   TCanvas *fCvsPadWave;
   TH1I *fPadWave;

   TCanvas *fCvsMC_XY{};
   TGraph *fMC_XY{};
   TGraph *fMC_XY_exp{};
   TGraph *fMC_XY_int{};
   TGraph *fMC_XY_back{};
   TCanvas *fCvsMC_Z{};
   TGraph *fMC_ZX{};
   TGraph *fMC_ZX_int{};
   TGraph *fMC_ZX_back{};
   TGraph *fMC_ZY{};
   TGraph *fMC_ZY_int{};
   TGraph *fMC_ZY_back{};

   Int_t fMinZ;
   Int_t fMaxZ;
   Int_t fMinX;
   Int_t fMaxX;

   Int_t f3DHitStyle;
   Int_t fMultiHit{10};
   Bool_t fSaveTextData;
   Float_t f3DThreshold;

   Bool_t fIsRawData;
   Bool_t fDrawVertexFromLines{false};

   AtHit const *fIniHit;
   AtHit const *fIniHitRansac;

   // std::vector<TEveLine*> fLineArray;
   std::vector<std::unique_ptr<TEveLine>> fPatternLines;

   Int_t fTrackNum;

   Int_t fMinTracksPerVertex;
   /*
      std::vector<std::unique_ptr<TEvePointSet>> fHitSetTFHC;  // for TrackFinderHC
      std::vector<std::unique_ptr<TEveBoxSet>> fHitClusterSet; // Track clusterization
      std::vector<std::unique_ptr<TEveLine>> fHitLine;         // Track line
   */
   std::vector<TEvePointSet *> fHitSetTFHC;  // for TrackFinderHC
   std::vector<TEveBoxSet *> fHitClusterSet; // Track clusterization
   std::vector<TEveElement *> fHitLine;      // Track line
   std::vector<TEvePointSet *> fVertex;      // Vertex line

   TEveRGBAPalette *fRGBAPalette;

   TCanvas *fCvsPID;
   TH2F *fPID;
   TCanvas *fCvsPID2;
   TH2F *fPID2;

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
   void UnpackHoughSpace() { fUnpackHough = kTRUE; }
   void SetHitAttributes(Color_t, Size_t, Style_t);
   void Set3DHitStyleBar();
   void Set3DHitStyleBox();
   void SetSaveTextData();
   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);
   void SetCorrectedEventBranch(TString branchName) { fCorrectedEventBranchName = branchName; }
   void SetPatternEventBranch(TString branchName) { fPatternEventBranchName = branchName; }
   void SetMinTracksPerVertex(Int_t val)
   {
      fMinTracksPerVertex = val;
   } // fVertexMod=0 one track vertex , fVertexMod=1 multi trakcs vertex
   static void SelectPad(const char *rawevt);
   void DrawWave(Int_t PadNum);
   void SetMultiHit(Int_t hitMax);
   void SetDrawVertexFromLines(bool val = true) { fDrawVertexFromLines = val; }

private:
   // S800Ana fS800Ana;
   std::vector<Double_t> fTofObjCorr;
   std::vector<Double_t> fMTDCObjRange;
   std::vector<Double_t> fMTDCXfRange;

   void DrawPadPlane();
   void DrawPadWave();

   AtMap *fAtMapPtr;
   void UpdateCvsPadPlane();
   void UpdateCvsPadWave();

   // Functions for drawing hits
   void DrawHitPoints();
   void DrawRawHits();
   void DrawRecoHits();

   EColor GetTrackColor(int i);

   ClassDef(AtEventDrawTaskNew, 2);
};

#endif
