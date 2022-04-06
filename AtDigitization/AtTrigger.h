#ifndef AtTrigger_H
#define AtTrigger_H

#include <Rtypes.h>
#include <RtypesCore.h>

#include "TObject.h"
#include "TString.h"
#include "AtHit.h"

class AtEvent;
class AtPad;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

class AtTrigger : public TObject {
public:
   AtTrigger();
   ~AtTrigger();

   void SetAtMap(TString mapPath);
   void SetTriggerParameters(Double_t read, Double_t write, Double_t MSB, Double_t LSB, Double_t width,
                             Double_t fraction, Double_t threshold, Double_t window, Double_t height);

   Bool_t ImplementTrigger(AtRawEvent *rawEvent, AtEvent *event);

protected:
   Bool_t fValidPad;

   Int_t fPadNum;

   Double_t fMultiplicity_threshold;
   Double_t fMultiplicity_window;
   Double_t fTrigger_height;
   Double_t fTime_factor;
   Double_t fTrigger_width;
   Double_t fPad_threshold;
   Double_t fTime_window;

   AtRawEvent *fRawEvent;
   AtEvent *fEvent;
   AtHit fHit;
   AtPad *fPad;

   Int_t fTbIdx;
   Int_t fCobo;
   Int_t fCoboNumArray[10240];

   Int_t fCount = 0.0;
   Int_t fMaxRawADC = 0.0;
   // vector<TH2I *> fHRawPulse;

   Int_t fMinIdx;
   Int_t fMaxIdx;
   Int_t fAccum;
   Bool_t fTrigger;

   ClassDef(AtTrigger, 2);
};
#endif
