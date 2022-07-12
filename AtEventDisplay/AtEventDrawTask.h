/**
 * @brief Event display task
 * Used to draw AT-TPC events
 * @author JungWoo Lee (Korea Univ.)
 *         Adapted for AtTPCROOT by Yassid Ayyad (NSCL)
 */
#ifndef ATEVENTDRAWTASK_H
#define ATEVENTDRAWTASK_H

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <memory>
#include <vector>     // for vector
class AtEventManager; // lines 17-17
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

class AtEventDrawTask : public FairTask {
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

   AtEventManager *fEventManager;
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

   TCanvas *fCvsPadPlane;
   TH2Poly *fPadPlane;
   TCanvas *fCvsPadWave;
   TH1I *fPadWave;
   TCanvas *fCvsPadAll;
   TH1I *fPadAll[fNumPads]{};
   TCanvas *fCvsQEvent;
   TH1D *fQEventHist;
   TH1D *fQEventHist_H;
   TCanvas *fCvsHoughSpace;
   TH2F *fHoughSpace;
   TCanvas *fCvsRhoVariance;
   TH1D *fRhoVariance;
   TCanvas *fCvsPhi;
   TH1D *fPhiDistr[5]{};
   TCanvas *fCvsMesh;
   TH1F *fMesh;
   TCanvas *fCvs3DHist;
   TH3F *f3DHist;
   TCanvas *fCvsRad;
   TH2F *fRadVSTb;
   TCanvas *fCvsTheta;
   TH2F *fTheta;
   TCanvas *fCvsThetaxPhi{};
   TH2F *fThetaxPhi{};
   TCanvas *fCvsQuadrant1{};
   TH2F *fQuadrant1{};
   TCanvas *fCvsQuadrant2{};
   TH2F *fQuadrant2{};
   TCanvas *fCvsQuadrant3{};
   TH2F *fQuadrant3{};
   TCanvas *fCvsQuadrant4{};
   TH2F *fQuadrant4{};
   TH1F *fAuxChannels[9]{};
   TCanvas *fCvsAux{};

   TH2F *fThetaxPhi_Ini{};
   TH2F *fThetaxPhi_Ini_RANSAC{};

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
   AtHit const *fIniHit;
   AtHit const *fIniHitRansac;

   // std::vector<TEveLine*> fLineArray;
   std::vector<std::unique_ptr<TEveLine>> fPatternLines;

   Int_t fTrackNum;
   /*
      std::vector<std::unique_ptr<TEvePointSet>> fHitSetTFHC;  // for TrackFinderHC
      std::vector<std::unique_ptr<TEveBoxSet>> fHitClusterSet; // Track clusterization
      std::vector<std::unique_ptr<TEveLine>> fHitLine;         // Track line
   */
   std::vector<TEvePointSet *> fHitSetTFHC;  // for TrackFinderHC
   std::vector<TEveBoxSet *> fHitClusterSet; // Track clusterization
   std::vector<TEveElement *> fHitLine;      // Track line

   TEveRGBAPalette *fRGBAPalette;

public:
   AtEventDrawTask();
   AtEventDrawTask(TString modes);

   ~AtEventDrawTask();

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
   static void SelectPad(const char *rawevt);
   void DrawWave(Int_t PadNum);
   void SetMultiHit(Int_t hitMax);

private:
   void DrawPadPlane();
   void DrawPadWave();
   void DrawPadAll();
   void DrawQEvent();
   void DrawRhoVariance();
   void DrawHoughSpace();
   void DrawPhiReco();
   void DrawMesh();
   void Draw3DHist();
   void DrawRad();
   void DrawTheta();
   void DrawThetaxPhi();
   void DrawMC();
   void DrawAux();

   AtMap *fAtMapPtr;
   void UpdateCvsPadPlane();
   void UpdateCvsPadWave();
   void UpdateCvsPadAll();
   void UpdateCvsQEvent();
   void UpdateCvsRhoVariance();
   void UpdateCvsHoughSpace();
   void UpdateCvsPhi();
   void UpdateCvsMesh();
   void UpdateCvs3DHist();
   void UpdateCvsRad();
   void UpdateCvsTheta();
   void UpdateCvsThetaxPhi();
   void UpdateCvsQuadrants();
   void UpdateCvsMC();
   void UpdateCvsAux();

   void ResetPadAll();
   void ResetPhiDistr();

   // Functions for drawing hits
   void DrawHitPoints();
   void DrawRawHits();
   void DrawRecoHits();
   void DrawAuxChannels();

   EColor GetTrackColor(int i);

   ClassDef(AtEventDrawTask, 2);
};

#endif
