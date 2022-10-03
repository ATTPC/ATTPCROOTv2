#ifndef ATSCACORRECT_H
#define ATSCACORRECT_H

/**
 * A filter to perform baseline subtraction using the average baseline and average phase effect stored as
 * AtRawEvent objects in a separate file or files.
 *
 */

#include "AtFilter.h"

#include "TFile.h"
#include "TString.h"

#include <iostream>

class AtMap;

using AtMapPtr = std::shared_ptr<AtMap>;

class AtSCACorrect : public AtFilter {
private:
   AtMapPtr fMap;

   AtRawEvent *fBaseline;
   AtRawEvent *fPhase;

public:
   AtSCACorrect(AtMapPtr map, TString baselineFilename, TString baselineEventName, TString phaseFilename,
                TString phaseEventName)
      : AtFilter(), fMap(map)
   {
      TFile *f1 = new TFile(baselineFilename.Data());
      f1->GetObject(baselineEventName.Data(), fBaseline);
      f1->Close();
      if (fBaseline != nullptr) {
         std::cout << "baseline opened" << std::endl;
      }
      TFile *f2 = new TFile(phaseFilename.Data());
      f2->GetObject(phaseEventName.Data(), fPhase);
      f2->Close();
      if (fPhase != nullptr) {
         std::cout << "phase opened" << std::endl;
      }
   }

   virtual void Init() override {}
   virtual void InitEvent(AtRawEvent *) override {}
   virtual bool IsGoodEvent() override { return true; }

   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;

private:
   void removeBaseline(AtPad *pad, AtPadReference *padReference);
   void removePhase(AtPad *pad, AtPadReference *padReference);
};

#endif //#ifndef ATSCACORRECT_H
