#ifndef ATPROTOEVENT_H
#define ATPROTOEVENT_H

#include "TROOT.h"
#include "TObject.h"

#include <vector>
#include <map>

#include "ATProtoQuadrant.hh"
#include "ATHit.hh"

class ATProtoEvent : public TNamed {
  public:
    ATProtoEvent();
    ~ATProtoEvent();

    void SetEventID(Int_t evtid);
    void AddQuadrant(ATProtoQuadrant quadrant);
    void SetQuadrantArray(std::vector<ATProtoQuadrant> *quadrantArray);

    ATProtoQuadrant *GetQuadrant(Int_t quadrantNo);
    std::vector<ATProtoQuadrant> *GetQuadrantArray();
    Int_t GetNumQuadrants();


    Int_t fEventID;

    ClassDef(ATProtoEvent, 1);
   private:
     std::vector<ATProtoQuadrant> fQuadrantArray;


};

#endif
