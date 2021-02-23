#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <arpa/inet.h>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "TSystem.h"
#include "TObjArray.h"
#include "TObjString.h"
#include "TString.h"
#include "TH1.h"

#include "VMEDecoder.h"

ClassImp(VMEDecoder);

VMEDecoder::VMEDecoder()
{
   Initialize();
}

VMEDecoder::VMEDecoder(TString filename)
{
   Initialize();
   // AddData(filename);
   // SetData(0);
}

VMEDecoder::~VMEDecoder() {}

void VMEDecoder::Initialize()
{

   fFileSize = 0;
   fEOF = kFALSE;
   fCurrentDataID = -1;
   fStackID = 0;
   fContinuationBit = 0;
   fDebug = kFALSE;
   fIsScaler = kFALSE;
   memset(fRawAdc, 0, sizeof(Int_t) * 8 * 512);
   fCoinReg = 0;
   fTimeStamp = 0;
}

Bool_t VMEDecoder::AddData(TString filename)
{
   /**
    * Check if there is a file named `filename`. If exists, add it to the list.
    **/

   TString nextData = filename;

   TObjArray *pathElements = 0;
   pathElements = nextData.Tokenize("/");

   Int_t numElements = pathElements->GetLast();

   TString path = "";
   if (numElements == 0)
      path = gSystem->pwd();
   else {
      if (filename(0, 1) == "/")
         path.Append("/");

      for (Int_t i = 0; i < numElements; i++) {
         path.Append(((TObjString *)pathElements->At(i))->GetString());
         path.Append("/");
      }
   }

   TString tempDataFile = ((TObjString *)pathElements->Last())->GetString();

   nextData = gSystem->Which(path, tempDataFile);
   if (!nextData.EqualTo("")) {
      std::cout << "== [VMEDecoder] Data file found: " << filename << std::endl;

      Bool_t isExist = 0;
      for (Int_t iIdx = 0; iIdx < fDataList.size(); iIdx++) {
         if (fDataList.at(0) == nextData) {
            std::cout << "== [VMEDecoder] The file already exists in the list!" << std::endl;
            isExist = 1;
         }
      }

      if (!isExist)
         fDataList.push_back(nextData);
   } else {
      std::cout << "== [VMEDecoder] Data file not found: " << filename << std::endl;

      return kFALSE;
   }

   delete pathElements;

   return kTRUE;
}

Bool_t VMEDecoder::SetData(Int_t index)
{
   if (index >= fDataList.size()) {
      std::cout << "== [VMEDecoder] End of list!" << std::endl;

      return kFALSE;
   }

   if (fData.is_open())
      fData.close();

   fEOF = kFALSE;

   TString filename = fDataList.at(index);

   fData.open(filename.Data(), std::ios::in | std::ios::ate | std::ios::binary);

   if (!(fData.is_open())) {
      std::cout << "== [VMEDecoder] VME file open error! Check it exists!" << std::endl;

      return kFALSE;
   } else {
      fFileSize = fData.tellg();

      std::cout << "== [VMEDecoder] " << filename << " is opened!" << std::endl;
      fData.seekg(0, fData.beg);

      fCurrentDataID = index;

      return kTRUE;
   }
}

Bool_t VMEDecoder::SetNextFile()
{
   return SetData(fCurrentDataID + 1);
}

Int_t VMEDecoder::GetNextEvent()
{

   UInt_t dataBuff = 0;
   UInt_t eventHeader = 0;
   UInt_t eventLength = 0;
   UInt_t scalerBuff = 0;
   UInt_t scalerBuffTerm = 0;
   UInt_t fadcBuff = 0;
   UInt_t fadcBuffTerm = 0;
   UInt_t SubEveNum = 0;

   UInt_t dummyBuff = 0;
   fScalerArray.clear();
   fIsScaler = kFALSE;
   memset(fRawAdc, 0, sizeof(Int_t) * 8 * 512);
   fCoinReg = 0;
   fTimeStamp = 0;

   // TH1I* test = new TH1I("test","test",512,0,511);

   while (!fData.eof()) {

      // std::cout<<" Data Position : "<<(ULong64_t)fData.tellg()<<std::endl;
      fData.read(reinterpret_cast<Char_t *>(&dataBuff), 2);

      if ((dataBuff & 0XFFFF) == 0Xe238) {
         if (fDebug)
            std::cout << " E238 Header found! " << std::endl;
         fData.seekg((ULong64_t)fData.tellg() - 4);
         fData.read(reinterpret_cast<Char_t *>(&eventHeader), 2);
         if (fDebug)
            std::cout << " Event Header  : " << std::hex << eventHeader << std::endl;

         if ((eventHeader & 0XFFFF) == 0X2025) {
            if (fIsScaler == kTRUE)
               std::cout << " = VMEDecoder : Warning Two Scaler Events following each other! " << std::endl;

            if (fDebug) {
               std::cout << " - Scaler Event Found ! " << std::endl;
               std::cout << "     - Stack ID                 : " << std::dec << ((eventHeader & 0xE000) >> 13)
                         << std::endl;
               std::cout << "     - Continuation Bit         : " << std::dec << ((eventHeader & 0x1000) >> 12)
                         << std::endl;
               std::cout << "     - Event Length             : " << std::dec << ((eventHeader & 0xFFF)) << std::endl;
            }
            fIsScaler = kTRUE;
            eventLength = eventHeader & 0xFFF;
            fData.read(reinterpret_cast<Char_t *>(&dataBuff), 2); // Read the header again
            for (Int_t i = 0; i < eventLength - 1; i++) {
               fData.read(reinterpret_cast<Char_t *>(&scalerBuff), 2);
               fScalerArray.push_back(scalerBuff);
               // std::cout<<" Scaler Buff  : "<<std::hex<<scalerBuff<<std::endl;
            }
            /*fData.read(reinterpret_cast<Char_t *>(&scalerBuffTerm),2);
                       std::cout<<" Scaler Buff Terminator : "<<std::hex<<scalerBuffTerm<<std::endl;
                  fData.read(reinterpret_cast<Char_t *>(&scalerBuffTerm),2);
                    std::cout<<" Scaler Buff Second Terminator : "<<std::hex<<scalerBuffTerm<<std::endl;*/
            // return 0;
         } else if ((eventHeader & 0XFFFF) == 0X17FB) {
            if (fDebug) {
               std::cout << " - fADC Event Found ! " << std::endl;
               std::cout << "     - Stack ID                        : " << std::dec << ((eventHeader & 0xE000) >> 13)
                         << std::endl;
               std::cout << "     - Continuation Bit                : " << std::dec << ((eventHeader & 0x1000) >> 12)
                         << std::endl;
               std::cout << "     - Event Length                    : " << std::dec << ((eventHeader & 0xFFF))
                         << std::endl;
            }
            eventLength = eventHeader & 0xFFF;
            fData.read(reinterpret_cast<Char_t *>(&fadcBuff), 2); // Read the header again
            SubEveNum = GetSubEventNum();
            if (fDebug)
               std::cout << "     - Event Number                    : " << std::dec << SubEveNum << std::endl;
            fTimeStamp = GetTimeStamp();
            if (fDebug)
               std::cout << "     - TimeStamp                       : " << std::dec << fTimeStamp << std::endl;
            fCoinReg = GetCoinReg();
            if (fDebug)
               std::cout << "     - Coincidence Register Pattern    : " << std::hex << fCoinReg << std::endl;
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            for (Int_t i = 0; i < 512; i++) {
               fData.read(reinterpret_cast<Char_t *>(&fadcBuff), 4);
               Int_t sample_fadc1 = (fadcBuff & 0x0FFF0000) >> 16;
               Int_t sample_fadc2 = (fadcBuff & 0xFFF);
               fRawAdc[i] = sample_fadc1;
               fRawAdc[i + 512] = sample_fadc2;
               // test->SetBinContent(i,sample);

               // std::cout<<" i : "<<std::dec<<i<<std::endl;
               //  std::cout<<" fadc Buff  : "<<std::hex<<fadcBuff<<std::endl;
            }
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            // std::cout<<dummyBuff<<std::endl;
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            // std::cout<<dummyBuff<<std::endl;
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            // std::cout<<dummyBuff<<std::endl;
            fData.read(reinterpret_cast<Char_t *>(&dummyBuff), 4);
            // std::cout<<dummyBuff<<std::endl;
            for (Int_t i = 0; i < 512; i++) {
               fData.read(reinterpret_cast<Char_t *>(&fadcBuff), 4);
               Int_t sample_fadc3 = (fadcBuff & 0x0FFF0000) >> 16;
               fRawAdc[i + 512 * 2] = sample_fadc3;
               // test->SetBinContent(i,sample);
               // std::cout<<" i : "<<std::dec<<i<<std::endl;
               // std::cout<<" fadc Buff  : "<<std::hex<<fadcBuff<<std::endl;
            }

            /*fData.read(reinterpret_cast<Char_t *>(&fadcBuffTerm),2);
                       std::cout<<" Pekaboo : "<<std::hex<<fadcBuffTerm<<std::endl;
            fData.read(reinterpret_cast<Char_t *>(&fadcBuffTerm),2);
                       std::cout<<" Pekaboo : "<<std::hex<<fadcBuffTerm<<std::endl;    */
            // test->Draw();
            return 1;
         }
      }
   }

   std::cout << " = VMEDecoder : End of File! " << std::endl;
   if (fData.eof())
      return 0;
}

ULong64_t VMEDecoder::GetInitBuffHeader(Int_t event)
{

   std::cout << " Event ID : " << event << std::endl;

   fData.seekg((ULong64_t)fData.tellg(), fData.beg); // TODO: This will work for 1 file only
   UInt_t vmeHeader = 0;
   UInt_t bufferHeader = 0;
   UInt_t optHeader = 0;
   UInt_t eventHeader = 0;
   fData.read(reinterpret_cast<Char_t *>(&bufferHeader), 2);
   fData.read(reinterpret_cast<Char_t *>(&optHeader), 2);
   fData.read(reinterpret_cast<Char_t *>(&eventHeader), 2);
   fData.read(reinterpret_cast<Char_t *>(&vmeHeader), 2);
   if ((vmeHeader & 0XFFFF) == 0Xe238)
      std::cout << " Data Header found after buffer header : " << std::hex << (vmeHeader & 0xFFFF) << std::endl;
   std::cout << " - Buffer Header    : " << std::hex << (bufferHeader) << std::endl;
   std::cout << "     - Last Buffer              : " << ((bufferHeader & 0x8000) >> 15) << std::endl;
   std::cout << "     - Scaler Buffer            : " << ((bufferHeader & 0x4000) >> 14) << std::endl;
   std::cout << "     - Multi Buffer             : " << ((bufferHeader & 0x1000) >> 12) << std::endl;
   std::cout << "     - No. of events in buffer  : " << std::dec << ((bufferHeader & 0xFFF)) << std::endl;
   std::cout << " - Opt Header    : " << std::hex << (optHeader) << std::endl;
   std::cout << "     - No. of words in buffer   : " << std::dec << ((optHeader & 0xFFF)) << std::endl;
   std::cout << " - Event Header  : " << std::hex << (eventHeader) << std::endl;
   std::cout << "     - Stack ID                 : " << std::dec << ((eventHeader & 0xE000) >> 13) << std::endl;
   std::cout << "     - Continuation Bit         : " << std::dec << ((eventHeader & 0x1000) >> 12) << std::endl;
   std::cout << "     - Event Length             : " << std::dec << ((eventHeader & 0xFFF)) << std::endl;
   fStackID = (eventHeader & 0xE000) >> 13;
   fContinuationBit = (eventHeader & 0x1000) >> 12;

   return (ULong64_t)fData.tellg();
}

std::vector<UInt_t> VMEDecoder::GetScalerBuff()
{

   fData.seekg((ULong64_t)fData.tellg());
   std::vector<UInt_t> ScalerArray;
   UInt_t scalerBuff = 0;
   UInt_t scalerBuffTerm = 0;
   for (Int_t i = 0; i < 36; i++) {
      fData.read(reinterpret_cast<Char_t *>(&scalerBuff), 2);
      // std::cout<<" Scaler Buff  : "<<std::hex<<scalerBuff<<std::endl;
      ScalerArray.push_back(scalerBuff);
   }
   fData.read(reinterpret_cast<Char_t *>(&scalerBuffTerm), 2);
   std::cout << " Scaler Buff Terminator : " << std::hex << scalerBuffTerm << std::endl;
   fData.read(reinterpret_cast<Char_t *>(&scalerBuffTerm), 2);
   std::cout << " Scaler Buff Second Terminator : " << std::hex << scalerBuffTerm << std::endl;
}

UInt_t VMEDecoder::GetSubEventNum()
{

   UInt_t SubEveNum;
   fData.read(reinterpret_cast<Char_t *>(&SubEveNum), 4);
   // std::cout<<" SubEvent Number : "<<std::hex<<SubEveNum<<std::endl;
   return SubEveNum;
}

UInt_t VMEDecoder::GetTimeStamp()
{

   UInt_t TimeStamp;
   fData.read(reinterpret_cast<Char_t *>(&TimeStamp), 4);
   // std::cout<<" SubEvent Number : "<<std::hex<<SubEveNum<<std::endl;
   return TimeStamp;
}

UInt_t VMEDecoder::GetCoinReg()
{
   UInt_t CoinRegInner;
   fData.read(reinterpret_cast<Char_t *>(&CoinRegInner), 4);
   // std::cout << " SubEvent Number : " << std::hex << SubEveNum << std::endl;
   return CoinRegInner;
}

Int_t *VMEDecoder::GetRawfADC(Int_t chIdx)
{
   return fRawAdc + chIdx * 512;
}
