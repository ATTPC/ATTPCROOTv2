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

#include "AtUnpacker.h"

#include <Rtypes.h>
#include <TString.h>

#include <memory>
#include <mutex>
#include <string>
#include <utility> // for pair
#include <vector>

class GETLayeredFrame;
class GETBasicFrame;
class AtPedestal;
class AtRawEvent;
class GETDecoder2;
class TBuffer;
class TClass;
class TMemberInspector;
class AtPad;
struct AtPadReference;

class AtGRAWUnpacker : public AtUnpacker {
protected:
   using GETDecoder2Ptr = std::unique_ptr<GETDecoder2>;
   using AtPedestalPtr = std::unique_ptr<AtPedestal>;
   using CoboAndEvent = std::pair<int, int>;

   // Number of unique graw files (cobo or asad) to unpack.
   // Each has its own GETDecoder2, and AtPedestal instance and we will spawn fNumFiles
   // threads to unpack them in parallel
   Int_t fNumFiles;
   Int_t fNumEvents{-1};

   std::vector<GETDecoder2Ptr> fDecoder;
   std::vector<AtPedestalPtr> fPedestal;
   std::vector<Int_t> fCurrentEventID;

   Double_t fFPNSigmaThreshold = 5;
   Bool_t fIsData = false;
   Bool_t fIsNegativePolarity = true;
   Bool_t fIsSaveLastCell = false;
   Bool_t fIsSeparatedData;
   Bool_t fIsSubtractFPN = true;
   Bool_t fIsBaseLineSubtraction{};
   /** Checks for the number of events from only the first decoder. Set to true if all
    * files have the same number of events (MuTANT trigger or one cobo).
    */
   Bool_t fIsMutantOneRun{false};
   Bool_t fCheckNumEvents{false};

   // String to identify which file in fInputFileName map to which fDecoder
   std::string fFileIDString;
   std::mutex fRawEventMutex;

   Int_t fTargetFrameID{}; // fDataEventID

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
   void SetSaveLastCell(Bool_t val) { fIsSaveLastCell = val; }
   void SetSubtractFPN(Bool_t val) { fIsSubtractFPN = val; }
   void SetBaseLineSubtraction(Bool_t val) { fIsBaseLineSubtraction = val; }
   void SetMutantOneRun(Bool_t val) { fIsMutantOneRun = val; }
   void SetCheckNumEvents() { fCheckNumEvents = true; }
   // AtUnpacker interface
   virtual void Init() override;
   virtual void FillRawEvent(AtRawEvent &event) override; // Pass by ref to ensure it's a valid object
   virtual bool IsLastEvent() override;
   virtual void SetInputFileName(std::string fileName) override;
   void SetInputFileName(std::string fileName, std::string fileIDString);

   virtual Long64_t GetNumEvents() override;

private:
   void processInputFile();

   Bool_t AddData(TString filename, Int_t fileIdx);

   void ProcessFile(Int_t fileIdx);
   void ProcessBasicFile(Int_t fileIdx);
   void ProcessLayeredFrame(GETLayeredFrame *layeredFrame);
   void ProcessBasicFrame(GETBasicFrame *basicFrame);

   CoboAndEvent GetLastEvent(Int_t fileIdx);

   void doFPNSubtraction(GETBasicFrame &basicFrame, AtPedestal &pedestal, AtPad &pad, AtPadReference padRef);
   void doBaselineSubtraction(AtPad &pad);
   void saveFPN(GETBasicFrame &frame, AtPadReference PadRef, AtRawEvent *event);
   void savePad(GETBasicFrame &frame, AtPadReference PadRef, AtRawEvent *event, Int_t fileIdx);
   void fillPadAdc(GETBasicFrame &frame, AtPadReference PadRef, AtPad *pad);
   void saveLastCell(AtPad &pad, Double_t lastCell);
   void FindAndSetNumEvents();

   ClassDefOverride(AtGRAWUnpacker, 1)
};

#endif //#ifndef _ATGRAWUNPACKER_H_
