#ifndef WINDOW_C
#define WINDOW_C
#include "mira.C"

/*For fitting pad traces near window
 *Using equation [p1]/(exp((x-[p0])/[p3])+1)-[p2]
 *Load using root -l window_fitting.C
 */
void window_fitting()
{
   //runs automatically to load in run. For right now, we always use this run
   loadRun("/mnt/analysis/e12014/TPC/unpackedReducedFiltered/run_0210.root");

}

void loadData(ULong64_t eventNumber, int padNumber)
{
   //uses functions from mira.C to load in histogram for given event and padnumber
   loadEvent(eventNumber);
   TH1F *padHist = loadPad(padNumber);
   if (padHist != nullptr)
      padHist->Draw("hist");
}

double fitEquation(double *x, double *par)
{
   //sets up equation used for the fitting
   double fitval = par[1] / (TMath::Exp((x[0] - par[0]) / par[3]) + 1) - par[2];
   return fitval;
}

void runFit(int left_bound, int right_bound, int xloc, int jump_height, int init_height)
{
   //5 integer inputs. First 2 are the time-bucket bounds. 3rd is x-location of jump, 4th is jump height, and 5th is height before the jump (all approximated)
   TF1 *func = new TF1("func", fitEquation, left_bound, right_bound, 4);

   func->SetParameters(xloc, jump_height, init_height, 1);
   func->SetParNames("xloc", "jump_height", "init_height", "temp");

   func->Fit("fitEquaion")
}

/* I think these need to be in a function to compile properly.
 * Outside a function, you can only new variables
 * (like TF1* func = ...) but you cannot use them and call functions on them
 * (there is no injection popint for the code, so like the func->Set... needs to be in window_fitting() or another
 * function. to run */


#endif //ifndef WINDOW_C
