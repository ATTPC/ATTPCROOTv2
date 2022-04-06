
#ifndef AtTRIGGERPAR_H
#define AtTRIGGERPAR_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <vector>

// FAIRROOT classes
#include <FairParGenericSet.h>
// ROOT classes
#include <TString.h>

class FairLogger;
class FairParamList;
class TBuffer;
class TClass;
class TMemberInspector;

class AtTriggerPar : public FairParGenericSet {
public:
   // Constructors and Destructors
   AtTriggerPar(const Char_t *name, const Char_t *title, const Char_t *context);
   ~AtTriggerPar();

   virtual Bool_t getParams(FairParamList *paramList);

   Double_t GetWrite_Clock();
   Double_t GetRead_Clock();
   Double_t GetMaster_Clock();
   Double_t GetPad_thres_MSB();
   Double_t GetPad_thres_LSB();
   Double_t GetTrigger_signal_width();
   Double_t GetTrigger_discriminator_fraction();
   Double_t GetMultiplicity_threshold();
   Double_t GetMultiplicity_window();
   Double_t GetTrigger_height();

   TString GetFile(Int_t fileNum);

   virtual void putParams(FairParamList *paramList);

private:
   FairLogger *fLogger;
   Double_t fWrite_clock;
   Double_t fRead_clock;
   Double_t fMaster_clock;
   Double_t fPad_thresh_MSB;
   Double_t fPad_thresh_LSB;
   Double_t fTrigger_signal_width;
   Double_t fTrigger_discriminator_fraction;
   Double_t fMultiplicity_threshold;
   Double_t fMultiplicity_window;
   Double_t fTrigger_height;
   std::vector<Double_t> fParameters;
   Bool_t fInitialized;
   Bool_t fIsPersistent;

   ClassDef(AtTriggerPar, 1);
};

#endif
