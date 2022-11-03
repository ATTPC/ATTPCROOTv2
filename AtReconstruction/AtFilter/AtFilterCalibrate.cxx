#include "AtFilterCalibrate.h"

#include "AtPad.h"

#include <FairLogger.h>

#include <TString.h>

#include <array>
#include <fstream>

AtFilterCalibrate::AtFilterCalibrate() = default;

void AtFilterCalibrate::Init()
{
   openFileAndReadContents();
}

void AtFilterCalibrate::Filter(AtPad *pad, AtPadReference *padReference)
{
   auto padNum = pad->GetPadNum();
   auto &adc = pad->GetADC();
   auto intercept = fIntercept.find(padNum);

   if (intercept == fIntercept.end()) {
      pad->SetValidPad(false);
      LOG(debug) << "Missing calibration for pad: " << padNum;
   } else
      for (int tb = 0; tb < 512; tb++)
         pad->SetADC(tb, fIntercept[padNum] + adc[tb] * fSlope[padNum]);
}

void AtFilterCalibrate::openFileAndReadContents()
{
   auto file = openFile();
   readContents(*file);
   LOG(info) << "Loaded calibration parameters for: " << fIntercept.size() << " pads.";
}

filePtr AtFilterCalibrate::openFile()
{
   filePtr file = std::make_unique<std::ifstream>(fCalibrationFile.Data());
   if (!file->is_open())
      LOG(fatal) << "Failed to open calibration file: " << fCalibrationFile;

   LOG(info) << "Opened calibration file: " << fCalibrationFile;
   return file;
}

void AtFilterCalibrate::readContents(std::ifstream &file)
{
   int padNumber;
   float intercept, slope;
   while (!file.eof()) {
      file >> padNumber >> intercept >> slope;
      fIntercept.emplace(padNumber, intercept);
      fSlope.emplace(padNumber, slope);
   }
}
