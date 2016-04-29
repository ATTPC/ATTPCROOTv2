#ifndef ATANALYSIS_H
#define ATANALYSIS_H

#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoEventAna.hh"
#include "ATProtoQuadrant.hh"
#include "ATHoughSpace.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpaceCircle.hh"
#include "ATHit.hh"

#include "TF1.h"
#include "TGraph.h"


// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

// STL
#include <vector>
#include <algorithm>

// ROOT classes
#include "TClonesArray.h"

class ATAnalysis : public TObject
{
     public:
        ATAnalysis();
        virtual ~ATAnalysis();

       virtual void Analyze(ATProtoEvent* protoevent,ATProtoEventAna* protoeventAna,ATHoughSpaceLine* houghspace,TF1 *(&HoughFit)[4],TGraph *(&HitPatternFilter)[4],TF1 *(&FitResult)[4])=0;

       ClassDef(ATAnalysis, 1);

};

#endif
