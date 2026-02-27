#ifndef AtENCOURSEEVENT_H
#define AtENCOURSEEVENT_H

#include "AtBaseEvent.h"

#include "AtPPACPair.h"

#include <FairLogger.h>

#include <Rtypes.h>

class AtENCourseEvent : public AtBaseEvent {
private:
   std::unique_ptr<AtPPACPair> fF2PPACs;
   std::unique_ptr<AtPPACPair> fF3PPACs;

public:
   AtENCourseEvent();
   AtENCourseEvent(const AtENCourseEvent &copy);
   AtENCourseEvent(const AtBaseEvent &copy) : AtBaseEvent(copy) { SetName("AtENCourseEvent"); }
   AtENCourseEvent &operator=(const AtENCourseEvent object);
   virtual ~AtENCourseEvent() = default;

   friend void swap(AtENCourseEvent &first, AtENCourseEvent &second)
   {
      using std::swap;
      swap(dynamic_cast<AtBaseEvent &>(first), dynamic_cast<AtBaseEvent &>(second));

      swap(first.fF2PPACs, second.fF2PPACs);
      swap(first.fF3PPACs, second.fF3PPACs);
   }

   void Clear(Option_t *opt = nullptr) override;

   void SetF2PPACs(std::unique_ptr<AtPPACPair> F2PPACs) { fF2PPACs = std::move(F2PPACs); }
   void SetF3PPACs(std::unique_ptr<AtPPACPair> F3PPACs) { fF3PPACs = std::move(F3PPACs); }

   const AtPPACPair &GetF2PPACs() const { return *fF2PPACs; }
   const AtPPACPair &GetF3PPACs() const { return *fF3PPACs; }

   ClassDefOverride(AtENCourseEvent, 1);
};

#endif
