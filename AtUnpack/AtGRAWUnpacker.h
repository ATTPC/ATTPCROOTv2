/* Unpacker for GRAW files using GETDecoder2 using unified unpacking task.
 * GETDEcoder2 originally written by Genie Jhang. Adapted by Yassid Ayyad
 *
 * Designed to support unpacking of GADGETII data, but should also work for
 * AT-TPC data. This replaced AtCore2.
 * Current version: Adam Anthony
 *
 * Input is a text file with a different GRAW file on each line.
 */

#ifndef _ATGRAWUNPACKER_H_
#define _ATGRAWUNPACKER_H_

#include <Rtypes.h>
#include <TString.h>
#include <mutex>
#include <vector>
#include <memory>
#include <string>

#include "AtUnpacker.h"

class GETLayeredFrame;
class GETBasicFrame;
class AtPedestal;
class AtRawEvent;
class GETDecoder2;
class TBuffer;
class TClass;
class TMemberInspector;

using GETDecoder2Ptr = std::unique_ptr<GETDecoder2>;
using AtPedestalPtr = std::unique_ptr<AtPedestal>;

class AtGRAWUnpacker : public AtUnpacker {
protected:
   // Number of unique graw files (cobo or asad) to unpack.
   // Each has its own GETDecoder2, and AtPedestal instance and we will spawn fNumFiles
   // threads to unpack them in parallel
   Int_t fNumFiles;

   std::vector<GETDecoder2Ptr> fDecoder;
   std::vector<AtPedestalPtr> fPedestal;
   std::vector<Int_t> fCurrentEventID;

   Double_t fFPNSigmaThreshold = 5;
   Bool_t fIsData = false;
   Bool_t fIsNegativePolarity = true;
   Bool_t fIsSeparatedData;

   // String to identify which file in fInputFileName map to which fDecoder
   std::string fFileIDString;
   std::mutex fRawEventMutex;

   Int_t fTargetFrameID; // fDataEventID

public:
   AtGRAWUnpacker(mapPtr map, Int_t numGrawFiles = 4);
   ~AtGRAWUnpacker() = default;

   // Getters
   Double_t GetFPNSigmaThreshold() const { return fFPNSigmaThreshold; }
   Bool_t GetIsPositivePolarity() const { return !fIsNegativePolarity; }
   Bool_t GetIsSeparatedData() const { return fIsSeparatedData; }

   // Setters
   void SetFPNSigmaThreshold(Double_t val) { fFPNSigmaThreshold = val; }
   void SetIsPositivePolarity(Bool_t val) { fIsNegativePolarity = !val; }
   void SetPseudoTopologyFrame(Int_t asadMask, Bool_t check);

   // AtUnpacker interface
   virtual void Init() override;
   virtual void FillRawEvent(AtRawEvent &event) override; // Pass by ref to ensure it's a valid object
   virtual bool IsLastEvent() override;
   virtual void SetInputFileName(std::string fileName) override;
   void SetInputFileName(std::string fileName, std::string fileIDString);

private:
   virtual Long64_t GetNumEvents() override { return -1; }
   Int_t GetFPNChannel(Int_t chIdx);

   void processInputFile();

   Bool_t AddData(TString filename, Int_t fileIdx);

   void ProcessFile(Int_t fileIdx);
   void ProcessBasicFile(Int_t fileIdx);
   void ProcessLayeredFrame(GETLayeredFrame *layeredFrame);
   void ProcessBasicFrame(GETBasicFrame *basicFrame);

   ClassDefOverride(AtGRAWUnpacker, 1)
};

#endif //#ifndef _ATGRAWUNPACKER_H_
