#include "AtROOTUnpacker.h"

#include <TTreeReaderValue.h>
#include <fairlogger/Logger.h>
#include <iostream>
#include <string>
#include <utility>

#include "AtMap.h"
#include "AtPedestal.h"
#include "AtRawEvent.h"
#include <TTree.h>
#include <TTreeReader.h>
#include <TTreeReaderArray.h>
#include <TFile.h>
#include "AtPad.h"
#include "PadReference.h"
#include <Rtypes.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

AtROOTUnpacker::AtROOTUnpacker(mapPtr map, Int_t numCobo)
   : AtUnpacker(map), fNumCobo(numCobo), fIsNegativePolarity(numCobo, false), fIsPadPlaneCobo(numCobo, false),
     fFPNChannels(numCobo), fPedestal(new AtPedestal())
{
}

void AtROOTUnpacker::SetIsPadPlaneCobo(vecBool vec)
{
   if (vec.size() != fNumCobo) {
      LOG(error) << "Invalid sized vector passed to SetIsPadPlaneCobo! Require size to be " << fNumCobo;
      return;
   }
   fIsPadPlaneCobo = std::move(vec);
}
void AtROOTUnpacker::SetIsNegativePolarity(vecBool vec)
{
   if (vec.size() != fNumCobo) {
      LOG(error) << "Invalid sized vector passed to SetPosotivePolarity! Require size to be " << fNumCobo;
      return;
   }
   fIsNegativePolarity = std::move(vec);
}

void AtROOTUnpacker::FillRawEvent(AtRawEvent &event)
{
   event.Clear();
   LOG(info) << "Start processing event " << fDataEventID;
   LOG(info) << "Getting fpn channels ";
   GetFPNChannelsFromROOTFILE();
   LOG(info) << "finished getting event from root file";
   ProcessROOTFILE(event);

   event.SetEventID(fEventID);
   fEventID++;
   fDataEventID++;
}

// type = 0 for padplane pads
// type = 1 for scintillators
// for padplane the channels 11,22,45 & 56 are used as fpn channels
// for scintillators 43,44,46 & 47 are used
void AtROOTUnpacker::GetFPNChannelsFromROOTFILE()
{

   Int_t type{0};
   std::vector<int> ChannelsFPNpp = {11, 22, 45, 56}; // fpn channels for
                                                      // padplane
   std::vector<int> ChannelsFPNsc = {43, 44, 46, 47}; // fpn channels for scintillators
   Int_t Nr_fpn_found{0};

   LOG(debug) << "Opening " << fInputFileName;
   auto *RawDataTreeFile = new TFile(fInputFileName.data(), "READ"); // NOLINT (belongs to ROOT)
   if (RawDataTreeFile->IsZombie()) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File containing tree not found, check if "
                   "input file name is correct ("
                << fInputFileName << ")" << cNORMAL << std::endl;
      return;
   }
   auto *RawDataTree = dynamic_cast<TTree *>(RawDataTreeFile->Get("EventDataTree"));
   if (!RawDataTree) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File does not contain raw data ttree (must "
                   "be named EventDataTree)"
                << cNORMAL << std::endl;
      std::cout << "Input file " << fInputFileName << " contains: " << std::endl;
      RawDataTreeFile->ls();
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
      if (*myInternalEventNr == fDataEventID) {
         PadReference PadRef = {*myCoboNr, *myAsadNr, *myAgetNr, *myChannelNr};
         for (Int_t i = 0; i < 4; i++) { // loop over number of fpn channels
            if ((PadRef.ch == ChannelsFPNpp[i]) && fIsPadPlaneCobo[PadRef.cobo]) {
               Nr_fpn_found++;
               // std::cout << "Added fpnchannel" << std::endl;
               // std::cout << ", corresponding to Cobo: " << PadRef[0] << "   Asad:
               // "  << PadRef[1] << "   Aget: "  << PadRef[2] << "   Ch: "  <<
               // PadRef[3] << std::endl;
               for (Int_t j = 0; j < 512; j++) {
                  fFPNChannels[PadRef.cobo][PadRef.asad][PadRef.aget][i][j] = mySamples[j];
               }
            } else if ((PadRef.ch == ChannelsFPNsc[i]) && !(fIsPadPlaneCobo[PadRef.cobo])) {
               Nr_fpn_found++;
               // std::cout << "Added fpnchannel" << std::endl;
               // std::cout << ", corresponding to Cobo: " << PadRef[0] << "   Asad:
               // "  << PadRef[1] << "   Aget: "  << PadRef[2] << "   Ch: "  <<
               // PadRef[3] << std::endl;
               for (Int_t j = 0; j < 512; j++) {
                  fFPNChannels[PadRef.cobo][PadRef.asad][PadRef.aget][i][j] = mySamples[j];
               }
            }
         }

      } else if (*myInternalEventNr > fDataEventID) {
         EventCompleted = true;
      }
   }
   std::cout << "A total of " << Nr_fpn_found << " fpn channels were found for event nr " << fDataEventID << std::endl;
}

void AtROOTUnpacker::ProcessROOTFILE(AtRawEvent &eventToFill)
{

   auto *RawDataTreeFile = new TFile(fInputFileName.data(), "READ"); // NOLINT (belongs to ROOT)
   if (RawDataTreeFile->IsZombie()) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File containing tree not found, check if "
                   "input file name is correct ("
                << fInputFileName << ")" << cNORMAL << std::endl;
      return;
   }
   auto *RawDataTree = dynamic_cast<TTree *>(RawDataTreeFile->Get("EventDataTree"));
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
      // std::cout << "InternalEventNr is" << *myInternalEventNr << " fDataEventID is " << fDataEventID << std::endl;
      if (*myInternalEventNr == fDataEventID) {
         PadReference PadRef = {*myCoboNr, *myAsadNr, *myAgetNr, *myChannelNr};
         Int_t PadRefNum = fMap->GetPadNum(PadRef);
         // std::cout << "Fired pad nr: " << PadRefNum << std::endl;
         // std::cout << ", corresponding to Cobo: " << PadRef.cobo << "   Asad: " <<
         // PadRef.asad << "   Aget: "  << PadRef.aget << "   Ch: "  << PadRef.ch <<
         // std::endl;
         auto PadCenterCoord = fMap->CalcPadCenter(PadRefNum);
         Bool_t IsInhibited = fMap->IsInhibited(PadRefNum);

         if (PadRefNum != -1 && !IsInhibited) {
            // AtPad *pad = new ((*fPadArray)[PadRefNum]) AtPad(PadRefNum);
            AtPad *pad = eventToFill.AddPad(PadRefNum);
            pad->SetPadCoord(PadCenterCoord);

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

            for (Int_t iTb = 0; iTb < 512; iTb++)
               pad->SetRawADC(iTb, rawadc[iTb]);

            Double_t adc[512] = {0};
            Int_t fpn_adc[512] = {0};
            for (int i = 0; i < 512; i++) {
               fpn_adc[i] = (fFPNChannels[PadRef.cobo][PadRef.asad][PadRef.aget][0][i] +
                             fFPNChannels[PadRef.cobo][PadRef.asad][PadRef.aget][1][i] +
                             fFPNChannels[PadRef.cobo][PadRef.asad][PadRef.aget][2][i] +
                             fFPNChannels[PadRef.cobo][PadRef.asad][PadRef.aget][3][i]) /
                            4;
            }
            Bool_t good =
               fPedestal->SubtractPedestal(512, fpn_adc, rawadc, adc, 5, fIsNegativePolarity[PadRef.cobo], 5, 20);
            // std::cout << "Is this pad good? " << good << std::endl;

            for (Int_t iTb = 0; iTb < 512; iTb++) {
               pad->SetADC(iTb, adc[iTb]);
               // if(iTb ==100) std::cout << "First time sample is:  " << rawadc[iTb]
               //<<  "after subtraction  "  << adc[iTb] << std::endl;
            }
            pad->SetPedestalSubtracted(kTRUE);
            eventToFill.SetIsGood(good);
         }
      } else if (*myInternalEventNr > fDataEventID) {
         EventCompleted = true;
      }
   }
}

Int_t AtROOTUnpacker::GetFPNChannel(Int_t chIdx)
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

void AtROOTUnpacker::Init()
{
   fDataEventID = 1;
   fEventID = 0;
   SetNumEvents();
}
bool AtROOTUnpacker::IsLastEvent()
{
   return fEventID >= GetNumEvents();
}

void AtROOTUnpacker::SetNumEvents()
{
   auto *RawDataTreeFile = new TFile(fInputFileName.data(), "READ"); // NOLINT (belongs to ROOT)
   if (RawDataTreeFile->IsZombie()) {
      std::cout << cRED
                << "[AtCoreSpecMAT] File containing tree not found, check if "
                   "input file name is correct ("
                << fInputFileName << ")" << cNORMAL << std::endl;
      return;
   }
   auto *RawDataTree = dynamic_cast<TTree *>(RawDataTreeFile->Get("EventDataTree"));
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
   while (myReader.Next())
      if (*myInternalEventNr > fNumEvents)
         fNumEvents = *myInternalEventNr;

   --fNumEvents; // Correct for off by one
}

ClassImp(AtROOTUnpacker);
