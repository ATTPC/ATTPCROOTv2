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

     std::vector<ATTrack> GetTrackArray();


  private:

   std::vector<ATTrack> fTrackArray;

   ClassDef(ATTrackingEventAna, 1);

 };

 #endif
