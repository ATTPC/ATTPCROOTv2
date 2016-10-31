#ifndef ATTRACKINGANALYSIS_H
#define ATTRACKINGANALYSIS_H

#include "ATAnalysis.hh"
#include "ATRansac.hh"

#include "ATMCQMinimization.hh"

class ATTrackingAnalysis : public ATAnalysis{

      public:
	       ATTrackingAnalysis();
        ~ATTrackingAnalysis();

        void Analyze(ATRANSACN::ATRansac *Ransac); // Analysis after RANSAC

      private:

          ClassDef(ATTrackingAnalysis, 1);


};

#endif
