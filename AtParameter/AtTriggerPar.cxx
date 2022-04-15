#include "AtTriggerPar.h"

#include <FairParGenericSet.h>
#include <FairParamList.h>
#include <TString.h>
#include <TSystem.h>
#include <FairLogger.h>
#include <fstream>

ClassImp(AtTriggerPar);

AtTriggerPar::AtTriggerPar(const Char_t *name, const Char_t *title, const Char_t *context)
   : FairParGenericSet("AtTriggerPar", "AtTPC Parameter Container", ""), fInitialized(kFALSE)
{
}

// Getters

Double_t AtTriggerPar::GetWrite_Clock()
{
   return fWrite_clock;
}
Double_t AtTriggerPar::GetRead_Clock()
{
   return fRead_clock;
}
Double_t AtTriggerPar::GetMaster_Clock()
{
   return fMaster_clock;
}
Double_t AtTriggerPar::GetPad_thres_MSB()
{
   return fPad_thresh_MSB;
}
Double_t AtTriggerPar::GetPad_thres_LSB()
{
   return fPad_thresh_LSB;
}
Double_t AtTriggerPar::GetTrigger_signal_width()
{
   return fTrigger_signal_width;
}
Double_t AtTriggerPar::GetTrigger_discriminator_fraction()
{
   return fTrigger_discriminator_fraction;
}
Double_t AtTriggerPar::GetMultiplicity_threshold()
{
   return fMultiplicity_threshold;
}
Double_t AtTriggerPar::GetMultiplicity_window()
{
   return fMultiplicity_window;
}
Double_t AtTriggerPar::GetTrigger_height()
{
   return fTrigger_height;
}

Bool_t AtTriggerPar::getParams(FairParamList *paramList)
{
   if (!paramList) {
      LOG(fatal) << "Parameter list doesn't exist!";
      return kFALSE;
   }

   if (!fInitialized) {
      if (!(paramList->fill("Write_clock", &fWrite_clock))) {
         LOG(fatal) << "Cannot find write_clock parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Read_clock", &fRead_clock))) {
         LOG(fatal) << "Cannot find read_clock parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Master_clock", &fMaster_clock))) {
         LOG(fatal) << "Cannot find master_clock parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Pad_thresh_MSB", &fPad_thresh_MSB))) {
         LOG(fatal) << "Cannot find pad_thresh_MSB parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Pad_thresh_LSB", &fPad_thresh_LSB))) {
         LOG(fatal) << "Cannot find pad_thresh_LSB parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Trigger_signal_width", &fTrigger_signal_width))) {
         LOG(fatal) << "Cannot find trigger_signal_width parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Trigger_discriminator_fraction", &fTrigger_discriminator_fraction))) {
         LOG(fatal) << "Cannot find trigger_discriminator_fraction parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Multiplicity_threshold", &fMultiplicity_threshold))) {
         LOG(fatal) << "Cannot find multiplicity_threshold parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Multiplicity_window", &fMultiplicity_window))) {
         LOG(fatal) << "Cannot find multiplicity_window parameter!";
         return kFALSE;
      }
      if (!(paramList->fill("Trigger_height", &fTrigger_height))) {
         LOG(fatal) << "Cannot find trigger_height parameter!";
         return kFALSE;
      }
   }
   return kTRUE;
}

void AtTriggerPar::putParams(FairParamList *paramList)
{
   if (!paramList) {
      LOG(fatal) << "Parameter list doesn't exist!";
      return;
   }

   paramList->add("Write_clock", fWrite_clock);
   paramList->add("Read_clock", fRead_clock);
   paramList->add("Master_clock", fMaster_clock);
   paramList->add("Pad_thresh_LSB", fPad_thresh_LSB);
   paramList->add("Pad_thresh_MSB", fPad_thresh_MSB);
   paramList->add("Trigger_signal_width", fTrigger_signal_width);
   paramList->add("Trigger_discriminator_fraction", fTrigger_discriminator_fraction);
   paramList->add("Tultiplicity_threshold", fMultiplicity_threshold);
   paramList->add("Tultiplicity_window", fMultiplicity_window);
   paramList->add("Trigger_height", fTrigger_height);
}

TString AtTriggerPar::GetFile(Int_t fileNum)
{
   std::ifstream fileList;
   TString sysFile = gSystem->Getenv("VMCWORKDIR");
   TString parFile = sysFile + "/parameters/AT.files.par";
   fileList.open(parFile.Data());

   if (!fileList) {
      LOG(fatal) << Form("File %s not found!", parFile.Data());

      throw;
   }

   std::string buffer;
   for (Int_t iFileNum = 0; iFileNum < fileNum + 1; ++iFileNum) {
      if (fileList.eof()) {
         LOG(fatal) << Form("Did not find string #%d in file %s.", fileNum, parFile.Data());

         throw;
      }

      std::getline(fileList, buffer);
   }

   fileList.close();

   return TString(sysFile + "/" + buffer);
}
