/**
 * @brief Event display task
 * @author JungWoo Lee (Korea Univ.)
 *         Adapted for ATTPCROOT by Yassid Ayyad (NSCL)
 */
#ifndef ATEVENTDRAWTASK_H
#define ATEVENTDRAWTASK_H


// FairRoot classes
#include "FairTask.h"
#include "FairLogger.h"

// ROOT classes
#include "TEvePointSet.h"
#include "TEveGeoShape.h"
#include "TEveBoxSet.h"
#include "TEveLine.h"
#include "TClonesArray.h"
#include "TVector3.h"
#include "TPaletteAxis.h"

#include "TCanvas.h"
#include "TH2.h"
#include "TH1.h"
#include "TH3.h"
#include "TGraph.h"
#include "TH2Poly.h"

#include "ATEventManager.hh"
#include "ATRawEvent.hh"
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATPatternEvent.hh"
#include "ATTrackingEventAna.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpaceCircle.hh"
#include "ATHoughSpace.hh"
#include "ATRansac.hh"
#include "ATTrackFinderHC.hh"
#include "ATHit.hh"
#include "AtTpcMap.h"
#include "ATProtoQuadrant.hh"
#include <fstream>

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

class ATEventDrawTask : public FairTask
{
  public :
    ATEventDrawTask();
    ATEventDrawTask(TString modes);

    virtual ~ATEventDrawTask();

    virtual InitStatus Init();
    virtual void Exec(Option_t* option);
    void Reset();

    //void Set2DPlotRange(Int_t uaIdx);
    void SetThreshold(Int_t val) { fThreshold=val; }
    void UnpackHoughSpace()      { fUnpackHough=kTRUE; }
    void SetHitAttributes(Color_t, Size_t, Style_t);
    void Set3DHitStyleBar();
    void Set3DHitStyleBox();
    void SetSaveTextData();
    void SetLine(double t, std::vector<Double_t> p, double &x, double &y, double &z);
    //void SetHitClusterAttributes(Color_t, Size_t, Style_t);
    //void SetRiemannAttributes(Color_t, Size_t, Style_t);

    static void SelectPad(const char *rawevt);
    void DrawWave(Int_t PadNum);

    void SetGeoOption(Option_t *option) {fGeoOption = option;}

    void SetProtoMap(TString map) {fMap = map;}

    void SetMultiHit(Int_t hitMax);

  protected :
    virtual void DrawPadPlane();
    virtual void DrawPadWave();
    virtual void DrawPadAll();
    virtual void DrawQEvent();
    virtual void DrawRhoVariance();
    virtual void DrawHoughSpace();
    virtual void DrawHoughSpaceProto();
    virtual void DrawPhiReco();
    virtual void DrawMesh();
    virtual void Draw3DHist();
    virtual void DrawRad();
    virtual void DrawTheta();
    virtual void DrawThetaxPhi();
    virtual void DrawMC();

    AtTpcMap *fAtMapPtr;
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



    void ResetPadAll();
    void ResetPhiDistr();


    void DrawHitPoints();
    void DrawHSpace();
    void DrawProtoSpace();
    void DrawMeshSpace();
    //void DrawHitClusterPoints();
    //void DrawRiemannHits();

    EColor GetTrackColor(int i);

    Bool_t fIs2DPlotRange;
    Bool_t fUnpackHough;
    Bool_t fIsCircularHough;
    Bool_t fIsLinearHough;

    TClonesArray* fHitArray;
    TClonesArray* fRawEventArray;
    TClonesArray* fHoughSpaceArray;
    TClonesArray* fProtoEventArray;
    TClonesArray* fRansacArray;
    TClonesArray* fTrackFinderHCArray;
    TClonesArray* fTrackingEventAnaArray;
    TClonesArray* fPatternEventArray;

    ATHoughSpaceLine*               fHoughSpaceLine_buff;
    ATHoughSpaceCircle*             fHoughSpaceCircle_buff;
    ATRANSACN::ATRansac*            fRansac;
    ATTrackingEventAna*             fTrackingEventAna;
    ATPATTERN::ATTrackFinderHC*     fTrackFinderHC;

    ATEventManager* fEventManager;
    ATRawEvent* fRawevent;

    AtTpcMap *fDetmap;

    Int_t fThreshold;
    Option_t* fGeoOption; //Chose Geometry of the detector: ATTPC (Default)-  Prototype
    TString fMap;

    TEvePointSet* fHitSet;
    TEvePointSet* fHitSetMin;

    TEvePointSet* fHitSetMC[5];// For MC results
    TEvePointSet* fHitSetTFHC[10];//for TrackFinderHC

   // TEveGeoShape* x;
   // std::vector<TEveGeoShape*> hitSphereArray;

    TEveBoxSet* fhitBoxSet;

    TPaletteAxis *fPadPlanePal;

    Color_t fHitColor;
    Size_t  fHitSize;
    Style_t fHitStyle;

    /*TEvePointSet* fHitClusterSet;
    Color_t fHitClusterColor;
    Size_t  fHitClusterSize;
    Style_t fHitClusterStyle;*/

    /*vector<TEvePointSet*> fRiemannSetArray;
    Color_t fRiemannColor;
    Size_t  fRiemannSize;
    Style_t fRiemannStyle;*/

    TCanvas* fCvsPadPlane;
    TH2Poly* fPadPlane;
    TCanvas* fCvsPadWave;
    TH1I*  fPadWave;
    TCanvas* fCvsPadAll;
    TH1I*  fPadAll[300];
    TCanvas* fCvsQEvent;
    TH1D* fQEventHist;
    TH1D* fQEventHist_H;
    TCanvas* fCvsHoughSpace;
    TH2F* fHoughSpace;
    TCanvas* fCvsRhoVariance;
    TH1D* fRhoVariance;
    TCanvas* fCvsPhi;
    TH1D* fPhiDistr[5];
    TCanvas* fCvsMesh;
    TH1F* fMesh;
    TCanvas* fCvs3DHist;
    TH3F* f3DHist;
    TCanvas* fCvsRad;
    TH2F *fRadVSTb;
    TCanvas* fCvsTheta;
    TH2F* fTheta;
    TCanvas* fCvsThetaxPhi;
    TH2F *fThetaxPhi;
    TCanvas* fCvsQuadrant1;
    TH2F* fQuadrant1;
    TCanvas* fCvsQuadrant2;
    TH2F* fQuadrant2;
    TCanvas* fCvsQuadrant3;
    TH2F* fQuadrant3;
    TCanvas* fCvsQuadrant4;
    TH2F* fQuadrant4;

    TH2F* fThetaxPhi_Ini;
    TH2F* fThetaxPhi_Ini_RANSAC;

    TCanvas* fCvsMC_XY;
    TGraph* fMC_XY;
    TGraph* fMC_XY_exp;
    TGraph* fMC_XY_int;
    TGraph* fMC_XY_back;
    TCanvas* fCvsMC_Z;
    TGraph* fMC_ZX;
    TGraph* fMC_ZX_int;
    TGraph* fMC_ZX_back;
    TGraph* fMC_ZY;
    TGraph* fMC_ZY_int;
    TGraph* fMC_ZY_back;



    Int_t fNQuads;


    Int_t fMinZ;
    Int_t fMaxZ;
    Int_t fMinX;
    Int_t fMaxX;

    Int_t f3DHitStyle;
    Int_t fMultiHit;
    Bool_t fSaveTextData;
    Float_t f3DThreshold;
    Bool_t fIsRawData;

    TF1 *fHoughLinearFit;
    TF1 *fRansacLinearFit;
    ATHit const *fIniHit;
    ATHit const *fIniHitRansac;


    //std::vector<TEveLine*> fLineArray;
    TEveLine* fLineArray[5];
    Int_t fLineNum;
    Int_t fTrackNum;
    //TEveLine* fLine;

    ClassDef(ATEventDrawTask,1);
};

#endif
