#ifndef GETLAYEREDFRAME
#define GETLAYEREDFRAME

#include <Rtypes.h>
#include <RtypesCore.h>
#include <iosfwd>

#include "GETLayerHeader.h"

class GETBasicFrame;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

class GETLayeredFrame : public GETLayerHeader {
public:
   GETLayeredFrame();

   Int_t GetNumFrames();
   TClonesArray *GetFrames();
   GETBasicFrame *GetFrame(Int_t index);

   void Clear(Option_t * = "");
   void Read(ifstream &stream);

private:
   TClonesArray *fFrames;

   ClassDef(GETLayeredFrame, 1);
};

#endif
