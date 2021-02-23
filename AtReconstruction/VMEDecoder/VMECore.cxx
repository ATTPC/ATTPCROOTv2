#include <iostream>
#include <fstream>
#include <cmath>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "TH1.h"

#include "VMECore.h"
#include "VMEDecoder.h"
#include "AtRawIC.h"

ClassImp(VMECore);

VMECore::VMECore()
{
   Initialize();
}

VMECore::~VMECore()
{
   delete fVMEDecoderPtr;
}

void VMECore::Initialize()
{

   fIsData = kFALSE;
   fVMEDecoderPtr = new VMEDecoder();
   fICChannel = -1;
   fMeshChannel = -1;
   fTriggerChannel = -1;
   fVMERawEventPtr = NULL;
}

Bool_t VMECore::AddData(TString filename)
{
   return fVMEDecoderPtr->AddData(filename);
}

Bool_t VMECore::SetData(Int_t value)
{
   fIsData = fVMEDecoderPtr->SetData(value);

   fPrevEventNo = -1;
   fCurrEventNo = -1;

   return fIsData;
}

VMERawEvent *VMECore::GetRawVMEEvent(Int_t eventID)
{

   // TH1I* test = new TH1I("test","test",512,0,511);

   if (!fIsData) {
      std::cout << "== VMECore -  Data file is not set" << std::endl;
      return NULL;
   }

   if (fVMERawEventPtr != NULL)
      delete fVMERawEventPtr;

   fVMERawEventPtr = new VMERawEvent();
   fVMERawEventPtr->SetEventID(eventID);

   std::cout << " Event Number : " << eventID << std::endl;
   Int_t evttype = fVMEDecoderPtr->GetNextEvent();
   if (evttype == 0)
      return NULL;
   Int_t *rawICadc = fVMEDecoderPtr->GetRawfADC(fICChannel);
   AtRawIC *ic = new AtRawIC();
   // AtRawIC ic;

   for (Int_t iTb = 0; iTb < 512; iTb++) {

      // test->SetBinContent(iTb,rawICadc[iTb]);
      ic->SetRawfADC(iTb, rawICadc[iTb]);
      // ic.SetRawfADC(iTb, rawICadc[iTb]);
   }

   fVMERawEventPtr->SetRawIC(ic);

   // test->Draw();

   return fVMERawEventPtr;
}
