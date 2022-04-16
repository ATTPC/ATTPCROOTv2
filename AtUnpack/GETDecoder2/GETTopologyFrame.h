#ifndef GETTOPOLOGYFRAME
#define GETTOPOLOGYFRAME

#include <Rtypes.h>

#include "GETHeaderBase.h"
#include <stdint.h>

#include <bitset>
#include <iosfwd>

class TBuffer;
class TClass;
class TMemberInspector;

using std::bitset;

#define GETTOPOLOGYFRAMESIZE (GETHEADERBASESIZE + 4)

class GETTopologyFrame : public GETHeaderBase {
public:
   GETTopologyFrame();

   UInt_t GetCoboIdx();
   bitset<4> GetAsadMask();
   UInt_t Get2pMode();
   UInt_t GetUNUSED();
   ULong64_t GetFrameSkip();
   ULong64_t GetHeaderSkip();

   void Clear(Option_t * = "");
   void Read(ifstream &Stream);

   void Print();

private:
   uint8_t fCoboIdx;
   uint8_t fAsadMask;
   uint8_t f2pMode;
   uint8_t fUNUSED;

   ClassDef(GETTopologyFrame, 1)
};

#endif
