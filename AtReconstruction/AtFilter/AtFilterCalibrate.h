#ifndef ATFILTERCALIBRATE_H
#define ATFILTERCALIBRATE_H

#include <unordered_map>
#include <iosfwd>
#include <memory>

#include "AtFilter.h"
#include <TString.h>

class AtPad;
class AtRawEvent;

using calibrationMap = std::unordered_map<int, float>;
using filePtr = std::unique_ptr<std::ifstream>;

class AtFilterCalibrate : public AtFilter {
private:
   calibrationMap fIntercept;
   calibrationMap fSlope;

   TString fCalibrationFile;

   void openFileAndReadContents();
   filePtr openFile();
   void readContents(std::ifstream &file);

public:
   AtFilterCalibrate();

   TString GetCalibrationFile() { return fCalibrationFile; }
   void SetCalibrationFile(TString fileName) { fCalibrationFile = fileName; }

   virtual void Init() override;
   virtual void InitEvent(AtRawEvent *event) override {}
   virtual void Filter(AtPad *pad) override;
   virtual bool IsGoodEvent() override { return true; }
};

#endif //#define ATFILTERCALIBRATE_H
