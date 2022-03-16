// =================================================
//  AtCore Class
//  Original author : Genie Jhang ( geniejhang@majimak.com )
//  Adapted for AtTPCROOT by Y. Ayyad (ayyadlim@nscl.msu.edu)
// =================================================

#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "AtCoreSpecMAT.h"

#include "GETCoboFrame.h"
#include "GETLayeredFrame.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtCoreSpecMAT);

AtCoreSpecMAT::AtCoreSpecMAT() : AtPadCoordArr(boost::extents[3174][3][2]), kOpt(0)
{
   Initialize();
}

AtCoreSpecMAT::AtCoreSpecMAT(Int_t opt) : AtPadCoordArr(boost::extents[3174][3][2]), kOpt(0)
{
   kOpt = opt;
   Initialize();
   SetNumTbs(512);
}

AtCoreSpecMAT::AtCoreSpecMAT(TString filename, Int_t opt) : AtPadCoordArr(boost::extents[3174][3][2]), kOpt(0)
{

   kOpt = opt;
   Initialize();
   AddData(filename);
   SetNumTbs(512);
}

AtCoreSpecMAT::AtCoreSpecMAT(TString filename, Int_t numTbs, Int_t windowNumTbs, Int_t windowStartTb)
   : AtPadCoordArr(boost::extents[3174][3][2]), kOpt(0)
{
   Initialize();
   AddData(filename);
   SetNumTbs(numTbs);
}

AtCoreSpecMAT::~AtCoreSpecMAT()
{
   /*for(Int_t i=0;i<10;i++)
   {
     delete fDecoderPtr[i];
     delete fPedestalPtr[i];
   }
   delete fAtMapPtr;*/
}

void AtCoreSpecMAT::Initialize()
{
   fRawEventPtr = new AtRawEvent();

   fAtMapPtr = new AtSpecMATMap(3174);

   fPedestalPtr[0] = new AtPedestal();
   for (Int_t iCobo = 1; iCobo < 16; iCobo++)
      fPedestalPtr[iCobo] = NULL;

   // fPlotPtr = NULL;

   fDecoderPtr[0] = new GETDecoder2();
   //  fDecoderPtr[0] -> SetDebugMode(1);
   fPadArray = new TClonesArray("AtPad", 3174);

   fIsData = kFALSE;
   fFPNSigmaThreshold = 5;

   // fGainCalibrationPtr = new STGainCalibration();
   // fIsGainCalibrationData = kFALSE;

   fNumTbs = 512;

   fTargetFrameID = -1;
   memset(fCurrentEventID, 0, sizeof(Int_t) * 40);

   fIsSeparatedData = kFALSE;
   kEnableAuxChannel = kFALSE;
   fAuxChannels.clear();

   fNumCobo = 4;

   for (Int_t i = 0; i < fNumCobo; i++) {
      fIsPadPlaneCobo[i] = kTRUE;
      fIsNegativePolarity[i] = kTRUE;
   }
}

Bool_t AtCoreSpecMAT::AddData(TString filename, Int_t coboIdx)
{
   fFileName = filename;
   return fDecoderPtr[coboIdx]->AddData(filename);
}

void AtCoreSpecMAT::SetPositivePolarity(Bool_t *value)
{
   for (Int_t i = 0; i < fNumCobo; i++) {
      fIsNegativePolarity[i] = !(value[i]);
   }
}

Bool_t AtCoreSpecMAT::SetData(Int_t value)
{
   fIsData = fDecoderPtr[0]->SetData(value);
   GETDecoder2::EFrameType frameType = fDecoderPtr[0]->GetFrameType();

   if (fIsSeparatedData) {
      for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++) {
         if (fPedestalPtr[iCobo] == NULL)
            fPedestalPtr[iCobo] = new AtPedestal();

         fIsData &= fDecoderPtr[iCobo]->SetData(value);
         frameType = fDecoderPtr[iCobo]->GetFrameType();
         /*
               if (frameType != GETDecoder2::kCobo || frameType !=
            GETDecoder2::kBasic) { std::cout << cRED << "== [AtCore] When using
            separated data, only accepted are not merged frame data files!" <<
            cNORMAL << std::endl;

                 fIsData = kFALSE;
                 return fIsData;
               }
               */
         fIsData = kTRUE;
      }
   }

   fTargetFrameID = -1;
   memset(fCurrentEventID, 0, sizeof(Int_t) * 40);

   return fIsData;
}

Int_t AtCoreSpecMAT::GetNumData(Int_t coboIdx)
{
   return fDecoderPtr[coboIdx]->GetNumData();
}

TString AtCoreSpecMAT::GetDataName(Int_t index, Int_t coboIdx)
{
   return fDecoderPtr[coboIdx]->GetDataName(index);
}

void AtCoreSpecMAT::SetNumTbs(Int_t value)
{
   fNumTbs = value;
   fDecoderPtr[0]->SetNumTbs(value);

   if (fIsSeparatedData)
      for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++)
         fDecoderPtr[iCobo]->SetNumTbs(value);
}

void AtCoreSpecMAT::SetFPNPedestal(Double_t sigmaThreshold)
{
   fFPNSigmaThreshold = sigmaThreshold;

   std::cout << "== [AtCore] Using FPN pedestal is set!" << std::endl;
}

Bool_t AtCoreSpecMAT::SetAtTpcMap(Char_t const *lookup)
{

   dynamic_cast<AtSpecMATMap *>(fAtMapPtr)->GenerateAtTpc();
   // NOTE: In the case of the AtTPC Map we need to generate
   // the coordinates to calculate the Pad Center

   Bool_t MapIn = fAtMapPtr->ParseXMLMap(lookup);
   if (!MapIn)
      return false;
   // Bool_t kIsIniParsed = fAtMapPtr->ParseInhibitMap(fIniMap, fLowgMap,
   // fXtalkMap);

   // AtPadCoordArr = fAtMapPtr->GetPadCoordArr();//TODO Use a pointer to a
   // simpler container
   //**** For debugging purposes only! ******//
   // fAtMapPtr->SetGUIMode();
   // fAtMapPtr->GetAtTPCPlane();
   return true;
}

Bool_t AtCoreSpecMAT::SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap)
{
   fIniMap = inimap;
   fLowgMap = lowgmap;
   fXtalkMap = xtalkmap;
   return kTRUE;
}

void AtCoreSpecMAT::SetIsPadPlaneCobo(Bool_t *IsPadPlane)
{
   for (Int_t i = 0; i < fNumCobo; i++) {
      fIsPadPlaneCobo[i] = IsPadPlane[i];
   }
}

// type = 0 for padplane pads
// type = 1 for scintillators
// for padplane the channels 11,22,45 & 56 are used as fpn channels
// for scintillators 43,44,46 & 47 are used
void AtCoreSpecMAT::GetFPNChannelsFromROOTFILE(Long64_t eventID)
{

   Int_t type{0};
   std::vector<int> ChannelsFPNpp = {11, 22, 45, 56}; // fpn channels for
                                                      // padplane
   std::vector<int> ChannelsFPNsc = {43, 44, 46, 47}; // fpn channels for scintillators
   Int_t Nr_fpn_found{0};

   TFile *RawDataTreeFile = new TFile(fFileName, "READ");
   if (!RawDataTreeFile) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File containing tree not found, check if "
                   "input file name is correct ("
                << fFileName << ")" << cNORMAL << std::endl;
      return;
   }
   TTree *RawDataTree = (TTree *)RawDataTreeFile->Get("EventDataTree");
   if (!RawDataTree) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File does not contain raw data ttree (must "
                   "be named EventDataTree)"
                << cNORMAL << std::endl;
      return;
   }

   // Read the raw data tree
   TTreeReader myReader(RawDataTree);

   // setting reader addresses
   TTreeReaderValue<Int_t> myInternalEventNr(myReader, "InternalEventNr");
   TTreeReaderValue<UChar_t> myCoboNr(myReader, "CoboNr");
   TTreeReaderValue<UChar_t> myAsadNr(myReader, "AsadNr");
   TTreeReaderValue<UChar_t> myAgetNr(myReader, "AgetNr");
   TTreeReaderValue<UChar_t> myChannelNr(myReader, "ChannelNr");
   TTreeReaderArray<UShort_t> mySamples(myReader, "Samples");

   // Fill vectors
   Bool_t EventCompleted = false;
   while (myReader.Next() && (!EventCompleted)) {
      if (*myInternalEventNr == eventID) {
         std::vector<int> PadRef = {*myCoboNr, *myAsadNr, *myAgetNr, *myChannelNr};
         for (Int_t i = 0; i < 4; i++) { // loop over number of fpn channels
            if ((PadRef[3] == ChannelsFPNpp[i]) && fIsPadPlaneCobo[PadRef[0]]) {
               Nr_fpn_found++;
               // std::cout << "Added fpnchannel" << std::endl;
               // std::cout << ", corresponding to Cobo: " << PadRef[0] << "   Asad:
               // "  << PadRef[1] << "   Aget: "  << PadRef[2] << "   Ch: "  <<
               // PadRef[3] << std::endl;
               for (Int_t j = 0; j < 512; j++) {
                  fFPNChannels[i][PadRef[0]][PadRef[1]][PadRef[2]][j] = mySamples[j];
               }
            } else if ((PadRef[3] == ChannelsFPNsc[i]) && !(fIsPadPlaneCobo[PadRef[0]])) {
               Nr_fpn_found++;
               // std::cout << "Added fpnchannel" << std::endl;
               // std::cout << ", corresponding to Cobo: " << PadRef[0] << "   Asad:
               // "  << PadRef[1] << "   Aget: "  << PadRef[2] << "   Ch: "  <<
               // PadRef[3] << std::endl;
               for (Int_t j = 0; j < 512; j++) {
                  fFPNChannels[i][PadRef[0]][PadRef[1]][PadRef[2]][j] = mySamples[j];
               }
            }
         }

      } else if (*myInternalEventNr > eventID) {
         EventCompleted = true;
      }
   }
   std::cout << "A total of " << Nr_fpn_found << " fpn channels were found for event nr " << eventID << std::endl;
}

void AtCoreSpecMAT::ProcessROOTFILE(Long64_t eventID)
{

   fRawEventPtr->SetEventID(eventID);

   TFile *RawDataTreeFile = new TFile(fFileName, "READ");
   if (!RawDataTreeFile) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File containing tree not found, check if "
                   "input file name is correct ("
                << fFileName << ")" << cNORMAL << std::endl;
      return;
   }
   TTree *RawDataTree = (TTree *)RawDataTreeFile->Get("EventDataTree");
   if (!RawDataTree) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File does not contain raw data ttree (must "
                   "be named EventDataTree)"
                << cNORMAL << std::endl;
      return;
   }

   // Read the raw data tree
   TTreeReader myReader(RawDataTree);

   // setting reader addresses
   TTreeReaderValue<Int_t> myInternalEventNr(myReader, "InternalEventNr");
   TTreeReaderValue<UChar_t> myCoboNr(myReader, "CoboNr");
   TTreeReaderValue<UChar_t> myAsadNr(myReader, "AsadNr");
   TTreeReaderValue<UChar_t> myAgetNr(myReader, "AgetNr");
   TTreeReaderValue<UChar_t> myChannelNr(myReader, "ChannelNr");
   TTreeReaderArray<UShort_t> mySamples(myReader, "Samples");

   // Fill vectors
   Bool_t EventCompleted = false;
   while (myReader.Next() && (!EventCompleted)) {
      // std::cout << "InternalEventNr is" << *myInternalEventNr << "   eventID is
      // " << eventID << std::endl;
      if (*myInternalEventNr == eventID) {
         std::vector<int> PadRef = {*myCoboNr, *myAsadNr, *myAgetNr, *myChannelNr};
         fCurrentEventID[0] = *myInternalEventNr;
         Int_t PadRefNum = fAtMapPtr->GetPadNum(PadRef);
         // std::cout << "Fired pad nr: " << PadRefNum << std::endl;
         // std::cout << ", corresponding to Cobo: " << PadRef[0] << "   Asad: " <<
         // PadRef[1] << "   Aget: "  << PadRef[2] << "   Ch: "  << PadRef[3] <<
         // std::endl;
         std::vector<Float_t> PadCenterCoord;
         PadCenterCoord.reserve(2);
         PadCenterCoord = fAtMapPtr->CalcPadCenter(PadRefNum);
         Bool_t IsInhibited = fAtMapPtr->GetIsInhibited(PadRefNum);

         if (PadRefNum != -1 && !IsInhibited) {
            AtPad *pad = new ((*fPadArray)[PadRefNum]) AtPad(PadRefNum);
            pad->SetPadXCoord(PadCenterCoord[0]);
            pad->SetPadYCoord(PadCenterCoord[1]);
            // std::cout << "pad coordinates are (" << PadCenterCoord[0] << ", " <<
            // PadCenterCoord[1] << ")" << std::endl;

            if ((PadRefNum == -1) || IsInhibited)
               pad->SetValidPad(kFALSE);
            else
               pad->SetValidPad(kTRUE);

            Int_t rawadc[512] = {0};
            for (int i = 0; i < 512; i++) {
               rawadc[i] = mySamples[i];
            }

            for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
               pad->SetRawADC(iTb, rawadc[iTb]);

            Int_t fpnCh = GetFPNChannel(*myChannelNr);
            Double_t adc[512] = {0};
            Int_t fpn_adc[512] = {0};
            for (int i = 0; i < 512; i++) {
               fpn_adc[i] = (fFPNChannels[0][PadRef[0]][PadRef[1]][PadRef[2]][i] +
                             fFPNChannels[1][PadRef[0]][PadRef[1]][PadRef[2]][i] +
                             fFPNChannels[2][PadRef[0]][PadRef[1]][PadRef[2]][i] +
                             fFPNChannels[3][PadRef[0]][PadRef[1]][PadRef[2]][i]) /
                            4;
            }
            Bool_t good = fPedestalPtr[0]->SubtractPedestal(fNumTbs, fpn_adc, rawadc, adc, 5,
                                                            fIsNegativePolarity[PadRef[0]], 5, 20);
            // std::cout << "Is this pad good? " << good << std::endl;

            for (Int_t iTb = 0; iTb < fNumTbs; iTb++) {
               pad->SetADC(iTb, adc[iTb]);
               // if(iTb ==100) std::cout << "First time sample is:  " << rawadc[iTb]
               // <<  "after subtraction  "  << adc[iTb] << std::endl;
            }
            pad->SetPedestalSubtracted(kTRUE);
            fRawEventPtr->SetIsGood(good);

            fRawEventPtr->SetPad(pad);
         }
      } else if (*myInternalEventNr > eventID) {
         EventCompleted = true;
      }
   }
}

Bool_t AtCoreSpecMAT::SetWriteFile(TString filename, Int_t coboIdx, Bool_t overwrite)
{
   return fDecoderPtr[coboIdx]->SetWriteFile(filename, overwrite);
}

void AtCoreSpecMAT::WriteData()
{
   if (fRawEventPtr == NULL) {
      std::cout << "== [AtCore] Call this method after GetRawEvent()!" << std::endl;

      return;
   }

   if (fIsSeparatedData) {
      for (Int_t iCobo = 0; iCobo < fNumCobo; iCobo++) {
         fDecoderPtr[iCobo]->GetCoboFrame(fTargetFrameID);
         fDecoderPtr[iCobo]->WriteFrame();
      }
   } else
      fDecoderPtr[0]->WriteFrame();
}

AtRawEvent *AtCoreSpecMAT::GetRawEvent(Long64_t eventID)
{

   if (!fIsData) {
      std::cout << cRED << "== [AtCore] Data file is not set!" << cNORMAL << std::endl;

      return NULL;
   }

   fRawEventPtr->Clear();
   fPadArray->Clear("C");

   if (eventID == -1)
      fTargetFrameID++;
   else
      fTargetFrameID = eventID;

   std::cout << "Start processing event " << eventID + 1 << std::endl;
   std::cout << "Getting fpn channels " << std::endl;
   GetFPNChannelsFromROOTFILE(eventID + 1);
   std::cout << "Getting raw data " << std::endl;
   ProcessROOTFILE(eventID + 1);
   std::cout << "finished getting event from root file" << std::endl;

   fRawEventPtr->SetEventID(fCurrentEventID[0]);

   Int_t iNumPads = 3174;
   for (Int_t i = 0; i < iNumPads; i++) {
      AtPad *pad = (AtPad *)fPadArray->At(i);
      if (pad != NULL)
         fRawEventPtr->SetPad(pad);
   }

   if (fRawEventPtr->GetNumPads() == 0 && fRawEventPtr->IsGood() == kFALSE)
      return NULL;
   else
      return fRawEventPtr;

   return NULL;
}

Int_t AtCoreSpecMAT::GetEventID()
{
   return fRawEventPtr->GetEventID();
}

Int_t AtCoreSpecMAT::GetNumTbs(Int_t coboIdx)
{
   return fDecoderPtr[coboIdx]->GetNumTbs();
}

Int_t AtCoreSpecMAT::GetFPNChannel(Int_t chIdx)
{
   Int_t fpn = -1;

   if (chIdx < 17)
      fpn = 11;
   else if (chIdx < 34)
      fpn = 22;
   else if (chIdx < 51)
      fpn = 45;
   else
      fpn = 56;

   return fpn;
}

void AtCoreSpecMAT::SetPseudoTopologyFrame(Int_t asadMask, Bool_t check)
{
   for (Int_t i = 0; i < fNumCobo; i++)
      fDecoderPtr[i]->SetPseudoTopologyFrame(asadMask, check);
}

void AtCoreSpecMAT::SetAuxChannel(std::vector<Int_t> AuxCh)
{
   kEnableAuxChannel = kTRUE;
   fAuxChannels = AuxCh;

   if (AuxCh.size() == 0)
      std::cout << cRED << " AtPSAtask : No auxiliary channels found --" << cNORMAL << std::endl;
   else {
      std::cout << cGREEN << " AtPSAtask : Auxiliary pads found : " << std::endl;
      for (Int_t i = 0; i < AuxCh.size(); i++)
         std::cout << "  " << AuxCh.at(i) << std::endl;
   }
   std::cout << cNORMAL << std::endl;
}

Bool_t AtCoreSpecMAT::GetIsAuxChannel(Int_t val)
{

   return std::find(fAuxChannels.begin(), fAuxChannels.end(), val) != fAuxChannels.end();
}

void AtCoreSpecMAT::SetNumCobo(Int_t numCobo)
{
   fNumCobo = numCobo;
}
