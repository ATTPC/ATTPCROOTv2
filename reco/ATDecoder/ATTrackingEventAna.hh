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

  private:

   ClassDef(ATTrackingEventAna, 1);

 };

 #endif
