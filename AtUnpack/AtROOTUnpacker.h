/* Unpacker for SpecMAT style ROOT files using common
 * interface AtUnpack
 *
 *
 */

#ifndef _ATROOTUNPACKER_H_
#define _ATROOTUNPACKER_H_

#include "AtUnpacker.h"

#include <Rtypes.h>

#include <memory>
#include <stdexcept> // for runtime_error
#include <vector>

class AtPedestal;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

using vecBool = std::vector<bool>;
using vecFPN = std::vector<Int_t[4][4][4][512]>;
using pedestalPtr = std::unique_ptr<AtPedestal>;

class AtROOTUnpacker : public AtUnpacker {
protected:
   pedestalPtr fPedestal;
   Double_t fFPNSigmaThreshold = 5;
   Long64_t fNumEvents = 0;
   // Int_t fCurrentEventID[16];
   // Int_t fTargetFrameID;

   Int_t fNumCobo;
   vecBool fIsPadPlaneCobo;
   vecBool fIsNegativePolarity;
   vecFPN fFPNChannels; //! Don't write to disk (root can't handle it) [cobo][asad][aget][fpn][sample]

public:
   AtROOTUnpacker() : AtUnpacker(nullptr) {}
   AtROOTUnpacker(mapPtr map, Int_t numCobo = 4);
   ~AtROOTUnpacker() = default;

   void Init() override;
   void FillRawEvent(AtRawEvent &event) override;
   bool IsLastEvent() override;
   Long64_t GetNumEvents() override { return fNumEvents; }

   // Functions called by decoder init
   void SetIsPadPlaneCobo(vecBool vec);
   void SetIsNegativePolarity(vecBool vec);
   void SetFPNPedestalRMS(double sigma) { fFPNSigmaThreshold = sigma; }
   void SetSaveFPN(bool val = true) override
   {
      throw std::runtime_error("SaveFPN is not implemented in AtROOTUnpacker");
   }

   ClassDefOverride(AtROOTUnpacker, 1);

private:
   void GetFPNChannelsFromROOTFILE();
   void ProcessROOTFILE(AtRawEvent &eventToFill);
   Int_t GetFPNChannel(Int_t chIdx);
   void SetNumEvents();
};
#endif //#ifndef _ATROOTUNPACKER_H_
