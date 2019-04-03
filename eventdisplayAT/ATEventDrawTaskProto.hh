#ifndef ATEVENTDRAWTASKPROTO_H
#define ATEVENTDRAWTASKPROTO_H


// FairRoot classes
#include "FairTask.h"
#include "FairLogger.h"

// ROOT classes
#include "TEvePointSet.h"
#include "TEveGeoShape.h"
#include "TEveBoxSet.h"
#include "TClonesArray.h"
#include "TVector3.h"
#include "TPaletteAxis.h"

#include "TCanvas.h"
#include "TH2.h"
#include "TH1.h"
#include "TH3.h"
#include "TGraph.h"
#include "TH2Poly.h"
#include "TEveLine.h"

#include "ATEventManagerProto.hh"
#include "ATRawEvent.hh"
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoEventAna.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpaceCircle.hh"
#include "ATHoughSpace.hh"
#include "ATHit.hh"
#include "AtTpcMap.h"
#include "ATProtoQuadrant.hh"
#include "ATPatternEvent.hh"
#include <fstream>

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

class ATEventDrawTaskProto : public FairTask
{
  public :
    ATEventDrawTaskProto();

    virtual ~ATEventDrawTaskProto();

    virtual InitStatus Init();
    virtual void Exec(Option_t* option);
    void Reset();
    void SetHitAttributes(Color_t, Size_t, Style_t);
    void Set3DHitStyleBar();
    void Set3DHitStyleBox();

    static void SelectPad(const char *rawevt);

    void SetProtoMap(TString map) {fMap = map;}

  protected:
    virtual void DrawPadWave();
    virtual void DrawPadPlane();
    virtual void DrawPadAll();
    virtual void DrawMesh();
    virtual void DrawProtoSpace();
    virtual void DrawProtoEL();
    virtual void DrawProtoHough();
    virtual void DrawProtoELAna();
    virtual void DrawProtoVertex();
    virtual void DrawProtoKine();
    virtual void DrawProtoAux();


    void DrawHitPoints();
    void DrawProtoPattern();
    void DrawProtoPatternAna();

    void UpdateCvsPadWave();
    void UpdateCvsPadPlane();
    void UpdateCvsPadAll();
    void UpdateCvsMesh();
    void UpdateCvsProtoQ();
    void UpdateCvsProtoEL();
    void UpdateCvsProtoVertex();
    void UpdateCvsProtoKine();
    void UpdateCvsProtoAux();

    EColor GetTrackColor(int i);


     //Basic types

    Int_t fMultiHit;
    Int_t f3DHitStyle;
    Bool_t fUnpackHough;
    Bool_t fIsCircularHough;
    Bool_t fIsLinearHough;
    Bool_t fIsRawData;
    Color_t fHitColor;
    Size_t  fHitSize;
    Style_t fHitStyle;

    // ROOT Objects
    TPaletteAxis *fPadPlanePal;


    TH3F* f3DHist;

    TH1I*    fPadAll[2015];
    TH1D*    fPhiDistr[5];
    TH1I*    fPadWave;
    TH2Poly* fPadPlane;
    TH1F*    fMesh;
    TGraph*  fQHitPattern[4];
    TGraph*  fQELossPattern[4];
    TF1*     fHoughFit[4];
    TGraph*  fQELossPatternAna[4];
    TF1*     fFit[4];
    TH2F*    fQVertex[4];
    TH2F*    fQKine[4];
    TH1F*    fAuxChannels[9];

    TCanvas* fCvsPadWave;
    TCanvas* fCvsPadPlane;
    TCanvas* fCvsPadAll;
    TCanvas* fCvsMesh;
    TCanvas* fCvsQuadrant1;
    TCanvas* fCvsQuadrant2;
    TCanvas* fCvsQuadrant3;
    TCanvas* fCvsQuadrant4;
    TCanvas* fCvsELQuadrant1;
    TCanvas* fCvsELQuadrant2;
    TCanvas* fCvsELQuadrant3;
    TCanvas* fCvsELQuadrant4;
    TCanvas* fCvsVertex;
    TCanvas* fCvsKineAA;
    TCanvas* fCvsAux;


    TF1 *fHoughLinearFit;
    TString fMap;

    TClonesArray*                   fHitArray;
    TClonesArray*                   fRawEventArray;
    TClonesArray*                   fHoughSpaceArray;
    TClonesArray*                   fProtoEventArray;
    TClonesArray*                   fProtoEventAnaArray;
    TClonesArray*                   fPatternEventArray;

    TEvePointSet* fHitSet;
    TEveBoxSet* fhitBoxSet;
    TEvePointSet* fHitSetPR[10];

    /// ATTPCROOT objects

    ATEventManagerProto*  fEventManager;
    ATHit*                fIniHit;
    AtTpcMap*             fDetmap;
    ATRawEvent*           fRawevent;
    ATHoughSpaceLine*     fHoughSpaceLine;

    TEveLine* fLineArray[10];
    Int_t fLineNum;
    Int_t fTrackNum;

    Bool_t kIsPRDrawn;
    Bool_t fSaveTextData;




    ClassDef(ATEventDrawTaskProto,1);
  };

  #endif
