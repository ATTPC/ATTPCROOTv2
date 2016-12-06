#ifndef ATTRACKINGEVENTANA_H
#define ATTRACKINGEVENTANA_H

#include "TROOT.h"
#include "TObject.h"
#include "ATTrack.hh"
#include "TVector3.h"

#include <vector>
#include <map>

class ATTrackingEventAna : public TNamed {

  public:
    ATTrackingEventAna();
    ~ATTrackingEventAna();

    void SetTrackArray(std::vector<ATTrack> *trackArray);
    void SetTrack(ATTrack *track);
    void SetVertex(Double_t vertex);
    void SetGeoVertex(TVector3 vertex);
    void SetVertexEnergy(Double_t vertexEner);

    Double_t GetVertex();
    Double_t GetVertexEnergy();
    TVector3 GetGeoVertex();
    std::vector<ATTrack> GetTrackArray();


  private:

   std::vector<ATTrack> fTrackArray;
   Double_t fVertex;
   Double_t fVertexEnergy;
   TVector3 fGeoVertex;

   ClassDef(ATTrackingEventAna, 1);

 };

 #endif
