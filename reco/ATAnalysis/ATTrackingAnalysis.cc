#include "ATTrackingAnalysis.hh"
#include "TMath.h"
#ifdef _OPENMP
#include <omp.h>
#endif
#include <memory>

ClassImp(ATTrackingAnalysis)

ATTrackingAnalysis::ATTrackingAnalysis()
{

}

ATTrackingAnalysis::~ATTrackingAnalysis()
{
}

void Analyze(ATRANSACN::ATRansac *Ransac)
{

  ATMCQMinimization *min = new ATMCQMinimization();
  min->ResetParameters();

}
