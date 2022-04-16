#ifndef AtPROTOEVENT_H
#define AtPROTOEVENT_H

#include "AtProtoQuadrant.h"

#include <Rtypes.h>
#include <TNamed.h>

#include <cstddef>
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

class AtProtoEvent : public TNamed {
public:
   AtProtoEvent();
   ~AtProtoEvent();

   void SetEventID(Int_t evtid);
   void AddQuadrant(AtProtoQuadrant quadrant);
   void SetQuadrantArray(std::vector<AtProtoQuadrant> *quadrantArray);

   AtProtoQuadrant *GetQuadrant(Int_t quadrantNo);
   std::vector<AtProtoQuadrant> *GetQuadrantArray();
   std::size_t GetNumQuadrants();

   Int_t fEventID{};

   ClassDef(AtProtoEvent, 1);

private:
   std::vector<AtProtoQuadrant> fQuadrantArray;
};

#endif
