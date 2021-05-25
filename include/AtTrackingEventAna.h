#ifndef AtTRACKINGEVENTANA_H
#define AtTRACKINGEVENTANA_H

#include "TROOT.h"
#include "TObject.h"
#include "AtTrack.h"
#include "TVector3.h"

#include <vector>
#include <map>

class AtTrackingEventAna : public TNamed {

public:
   AtTrackingEventAna();
   ~AtTrackingEventAna();

   void SetTrackArray(std::vector<AtTrack> *trackArray);
   void SetTrack(AtTrack *track);
   void SetVertex(Double_t vertex);
   void SetGeoVertex(TVector3 vertex);
   void SetVertexEnergy(Double_t vertexEner);

   Double_t GetVertex();
   Double_t GetVertexEnergy();
   TVector3 GetGeoVertex();
   std::vector<AtTrack> GetTrackArray();

private:
   std::vector<AtTrack> fTrackArray;
   Double_t fVertex;
   Double_t fVertexEnergy;
   TVector3 fGeoVertex;

   ClassDef(AtTrackingEventAna, 1);
};

#endif
