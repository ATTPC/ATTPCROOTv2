#include "ATProtoEvent.hh"

ClassImp(ATProtoEvent);

ATProtoEvent::ATProtoEvent()
:TNamed("ATProtoEvent", "Proto Event container")
{

}

ATProtoEvent::~ATProtoEvent()
{

}

void ATProtoEvent::SetEventID(Int_t evtid)                                        { fEventID = evtid; }
void ATProtoEvent::AddQuadrant(ATProtoQuadrant quadrant)                          { fQuadrantArray.push_back(quadrant); }
void ATProtoEvent::SetQuadrantArray(std::vector<ATProtoQuadrant> *quadrantArray)  { fQuadrantArray = *quadrantArray; }

Int_t ATProtoEvent::GetNumQuadrants() { return fQuadrantArray.size(); }

ATProtoQuadrant *ATProtoEvent::GetQuadrant(Int_t quadrantNo)
{
  return (quadrantNo < GetNumQuadrants() ? &fQuadrantArray[quadrantNo] : NULL);
}

std::vector<ATProtoQuadrant> *ATProtoEvent::GetQuadrantArray()
{
  return &fQuadrantArray;
}
