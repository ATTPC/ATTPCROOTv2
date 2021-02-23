#include "AtProtoEvent.h"

ClassImp(AtProtoEvent);

AtProtoEvent::AtProtoEvent()
:TNamed("AtProtoEvent", "Proto Event container")
{

}

AtProtoEvent::~AtProtoEvent()
{

}

void AtProtoEvent::SetEventID(Int_t evtid)                                        { fEventID = evtid; }
void AtProtoEvent::AddQuadrant(AtProtoQuadrant quadrant)                          { fQuadrantArray.push_back(quadrant); }
void AtProtoEvent::SetQuadrantArray(std::vector<AtProtoQuadrant> *quadrantArray)  { fQuadrantArray = *quadrantArray; }

Int_t AtProtoEvent::GetNumQuadrants() { return fQuadrantArray.size(); }

AtProtoQuadrant *AtProtoEvent::GetQuadrant(Int_t quadrantNo)
{
  return (quadrantNo < GetNumQuadrants() ? &fQuadrantArray[quadrantNo] : NULL);
}

std::vector<AtProtoQuadrant> *AtProtoEvent::GetQuadrantArray()
{
  return &fQuadrantArray;
}
