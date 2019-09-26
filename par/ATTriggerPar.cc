#include "ATTriggerPar.hh"


ClassImp(ATTriggerPar)

ATTriggerPar::ATTriggerPar(const Char_t *name, const Char_t *title, const Char_t *context)
:FairParGenericSet("ATTriggerPar", "ATTPC Parameter Container", "")
{
  fInitialized = kFALSE;
}

ATTriggerPar::~ATTriggerPar()
{
}

// Getters

Double_t  ATTriggerPar::GetWrite_Clock()                         { return fWrite_clock; }
Double_t  ATTriggerPar::GetRead_Clock()                          { return fRead_clock; }
Double_t  ATTriggerPar::GetMaster_Clock()                        { return fMaster_clock; }
Double_t  ATTriggerPar::GetPad_thres_MSB()                       { return fPad_thresh_MSB; }
Double_t  ATTriggerPar::GetPad_thres_LSB()                       { return fPad_thresh_LSB; }
Double_t  ATTriggerPar::GetTrigger_signal_width()                { return fTrigger_signal_width;}
Double_t  ATTriggerPar::GetTrigger_discriminator_fraction()      { return fTrigger_discriminator_fraction;}
Double_t  ATTriggerPar::GetMultiplicity_threshold() 	           { return fMultiplicity_threshold;}
Double_t  ATTriggerPar::GetMultiplicity_window() 	               { return fMultiplicity_window;}
Double_t  ATTriggerPar::GetTrigger_height()                      { return fTrigger_height;}


Bool_t ATTriggerPar::getParams(FairParamList *paramList) 
{
  if (!paramList) {
    fLogger -> Fatal(MESSAGE_ORIGIN, "Parameter list doesn't exist!");
    return kFALSE;
  }

  if (!fInitialized) {
    if (!(paramList -> fill("Write_clock", &fWrite_clock))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find write_clock parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Read_clock", &fRead_clock))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find read_clock parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Master_clock", &fMaster_clock))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find master_clock parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Pad_thresh_MSB", &fPad_thresh_MSB))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find pad_thresh_MSB parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Pad_thresh_LSB", &fPad_thresh_LSB))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find pad_thresh_LSB parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Trigger_signal_width", &fTrigger_signal_width))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find trigger_signal_width parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Trigger_discriminator_fraction", &fTrigger_discriminator_fraction))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find trigger_discriminator_fraction parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Multiplicity_threshold", &fMultiplicity_threshold))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find multiplicity_threshold parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Multiplicity_window", &fMultiplicity_window))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find multiplicity_window parameter!");
      return kFALSE;
    }
    if (!(paramList -> fill("Trigger_height", &fTrigger_height))) {
      fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find trigger_height parameter!");
      return kFALSE;
    }

  }
  return kTRUE;
}


void ATTriggerPar::putParams(FairParamList *paramList)
{
  if (!paramList) {
    fLogger -> Fatal(MESSAGE_ORIGIN, "Parameter list doesn't exist!");
    return;
  }

  paramList -> add("Write_clock", fWrite_clock);
  paramList -> add("Read_clock", fRead_clock);
  paramList -> add("Master_clock", fMaster_clock);
  paramList -> add("Pad_thresh_LSB", fPad_thresh_LSB);
  paramList -> add("Pad_thresh_MSB", fPad_thresh_MSB);
  paramList -> add("Trigger_signal_width", fTrigger_signal_width);
  paramList -> add("Trigger_discriminator_fraction", fTrigger_discriminator_fraction);
  paramList -> add("Tultiplicity_threshold", fMultiplicity_threshold);
  paramList -> add("Tultiplicity_window", fMultiplicity_window);
  paramList -> add("Trigger_height", fTrigger_height);


}

TString ATTriggerPar::GetFile(Int_t fileNum)
{
  std::ifstream fileList;
  TString sysFile = gSystem -> Getenv("VMCWORKDIR");
  TString parFile = sysFile + "/parameters/AT.files.par";
  fileList.open(parFile.Data());

  if(!fileList) { fLogger -> Fatal(MESSAGE_ORIGIN, Form("File %s not found!", parFile.Data()));

    throw;
  }

  Char_t buffer[256];
  for(Int_t iFileNum = 0; iFileNum < fileNum + 1; ++iFileNum){
    if(fileList.eof()) {
      fLogger -> Fatal(MESSAGE_ORIGIN, Form("Did not find string #%d in file %s.", fileNum, parFile.Data()));

      throw;
    }

    fileList.getline(buffer, 256);
  }

  fileList.close();

  return TString(sysFile + "/" + buffer);
}
