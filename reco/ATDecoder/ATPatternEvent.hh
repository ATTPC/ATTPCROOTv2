#ifndef ATPATTERNEVENT_H
#define ATPATTERNEVENT_H

#include "TROOT.h"
#include "TObject.h"
#include "ATTrack.hh"
#include "TVector3.h"

#include <vector>
#include <map>

class ATPatternEvent : public TNamed {

  public:
    ATPatternEvent();
    ~ATPatternEvent();

    void SetTrackCand(std::vector<ATTrack> tracks);

    std::vector<ATTrack>& GetTrackCand();



  private:

   std::vector<ATTrack> fTrackCand; //Candidate tracks

   ClassDef(ATPatternEvent, 1);

};

 #endif
