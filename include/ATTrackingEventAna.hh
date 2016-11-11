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

    Double_t GetVertex();

    std::vector<ATTrack> GetTrackArray();


  private:

   std::vector<ATTrack> fTrackArray;
   Double_t fVertex;

   ClassDef(ATTrackingEventAna, 1);

 };

 #endif
