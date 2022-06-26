#ifndef MANUAL_C
#define MANUAL_C
#include "mira.C"

/*For fitting pad traces near window or plane
 *Using equation [p1]/(exp((x-[p0])/[p3])+1)-[p2] for window
 *Using equation [p2]*exp(-3*(x-[p0])/[p1])*sin((x-[p0])/[p1])*((x-[p0])/[p1])^3 for plane
 *Load using root -l manual_fitting.C
 */

//functions declarations
void manual_fitting();
void loadData(ULong64_t eventNumber, int padNumber);
double WindowEquation(double *x, double *par);
double PlaneEquation(double *x, double *par);
void window_run_fit(int left_bound, int right_bound, int xloc, int jump_height, int init_height);
void plane_run_fit(int left_bound, int right_bound);

//variable declarations
TH1F *padHist;


void manual_fitting()
{
   //runs automatically to load in run. For right now, we always use this run
   loadRun("/mnt/analysis/e12014/TPC/unpackedReducedFiltered/run_0210.root");

}

void loadData(ULong64_t eventNumber, int padNumber)
{
   //uses functions from mira.C to load in histogram for given event and padnumber
   loadEvent(eventNumber);
   //TH1F *padHist = loadPad(padNumber);
   padHist = loadPad(padNumber);
   if (padHist != nullptr)
      padHist->Draw("");
}

double WindowEquation(double *x, double *par)
{
   //sets up equation used for the fitting
   double fitval = par[1] / (TMath::Exp((x[0] - par[0]) / par[3]) + 1) - par[2];
   return fitval;
}

double PlaneEquation(double *x, double *par)
{
   //sets up equation used for the fitting
   double fitval = par[2]*TMath::Exp(-3*(x[0]-par[0])/par[1])*TMath::Sin((x[0]-par[0])/par[1])*TMath::Power((x[0]-par[0])/par[1], 3)
   return fitval;
}


void window_run_fit(int left_bound, int right_bound, int xloc, int jump_height, int init_height)
{
   //5 integer inputs. First 2 are the time-bucket bounds. 3rd is x-location of jump, 4th is jump height, and 5th is height before the jump (all approximated)
   TF1 *func = new TF1("func", WindowEquation, left_bound, right_bound, 4);

   func->SetParameters(xloc, jump_height, init_height, 1);
   func->SetParNames("xloc", "jump_height", "init_height", "temp");

   padHist->Fit("func","R");
}

void plane_run_fit(int left_bound, int right_bound)
{
   TF1 *func = new TF1("func", PlaneEquation, left_bound, right_bound, 3);

   func->SetParameters(100, 2.25, 25000);
   func->SetParLimits(0,left_bound,1.e6);
   func->SetParLimits(2,0,1.e6);
   func->SetParNames("par0", "drift_time", "par2");

   padHist->Fit("func","R");
}


#endif //ifndef MANUAL_C
