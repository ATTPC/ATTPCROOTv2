#ifndef ATTRACKINGEVENTANA_H
#define ATTRACKINGEVENTANA_H

#include "TROOT.h"
#include "TObject.h"
#include "ATTrack.hh"

#include <vector>
#include <map>

class ATTrackingEventAna : public TNamed {

  public:
    ATTrackingEventAna();
    ~ATTrackingEventAna();

    void SetTrackArray(std::vector<ATTrack> *trackArray);
    void SetTrack(ATTrack *track);
    void SetVertex(Double_t vertex);
    void SetVertexEnergy(Double_t vertexEner);

    Double_t GetVertex();
    Double_t GetVertexEnergy();
    std::vector<ATTrack> GetTrackArray();


  private:

   std::vector<ATTrack> fTrackArray;
   Double_t fVertex;
   Double_t fVertexEnergy;

   ClassDef(ATTrackingEventAna, 1);

 };

 #endif
