#ifndef ATTrigger_H
#define ATTrigger_H

#include <fstream>
#include <iostream>
#include <limits>
#include <algorithm> //std::min

#include "TObject.h"
#include "TString.h"
#include "TClonesArray.h"
#include "TMath.h"
#include "TClonesArray.h"
#include "TStyle.h"
#include "TFileCollection.h"
#include "TError.h"
#include "TMinuit.h"
#include "ATRawEvent.hh"
#include "ATEvent.hh"
#include "ATHit.hh"
#include "TMatrixD.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

class ATTrigger : public TObject {
  public:
      ATTrigger();
      ~ATTrigger();


      void SetAtMap(TString mapPath);
      void SetTriggerParameters(Double_t read, Double_t write, Double_t MSB,
      Double_t LSB, Double_t width, Double_t fraction, Double_t threshold, Double_t window, Double_t height);

      Bool_t ImplementTrigger(ATRawEvent *rawEvent, ATEvent *event);



    protected:
      Bool_t fValidPad;

      Double_t* fRawAdc;
      Int_t fPadNum;

      Double_t fMultiplicity_threshold;
      Double_t fMultiplicity_window;
      Double_t fTrigger_height;
      Double_t fTime_factor;
      Double_t fTrigger_width;
      Double_t fPad_threshold;
      Double_t fTime_window;

      ATRawEvent* fRawEvent;
      ATEvent* fEvent;
      ATHit fHit;
      ATPad* fPad;

      Int_t fTbIdx;
      Int_t fCobo;
      Int_t fCoboNumArray[10240];

    	Int_t fCount =0.0;
    	Int_t fMaxRawADC = 0.0;
    	//vector<TH2I *> fHRawPulse;

    	Int_t fMinIdx;
      Int_t fMaxIdx;
      Int_t fAccum;
    	Bool_t fTrigger;


    ClassDef(ATTrigger, 1);
};
#endif
