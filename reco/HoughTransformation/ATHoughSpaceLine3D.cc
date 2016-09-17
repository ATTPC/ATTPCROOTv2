#include "ATHoughSpaceLine3D.hh"
#include "TCanvas.h"
#include "Fit/Fitter.h"
#ifdef _OPENMP
#include <omp.h>
#endif

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATHoughSpaceLine3D)

ATHoughSpaceLine3D::ATHoughSpaceLine3D()
{



}

ATHoughSpaceLine3D::~ATHoughSpaceLine3D()
{

}

TH2F* ATHoughSpaceLine3D::GetHoughSpace(TString ProjPlane) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane,multiarray PadCoord) {}
void ATHoughSpaceLine3D::CalcMultiHoughSpace(ATEvent* event) {}
void ATHoughSpaceLine3D::CalcHoughSpace(ATEvent* event) {}
