#ifndef MANUAL_C
#define MANUAL_C
#include "mira.C"
#include <cmath>

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
//void window_run_fit(int left_bound, int right_bound, int xloc, int jump_height, int init_height); old equation with user defined initial values
double window_run_fit();
//void plane_run_fit(int left_bound, int right_bound);
double plane_run_fit();
void automater_plane();
void automater_window();


//variable declarations
TH1F *padHist;
int padNum;


void manual_fitting()
{
   //runs automatically to load in run. For right now, we always use this run
   loadRun("/mnt/analysis/e12014/TPC/unpackedReducedFiltered/run_0210.root");

}


void loadData(ULong64_t eventNumber, int padNumber)
{
   //uses functions from mira.C to load in histogram for given event and padnumber
   //has been fazed out for automatic pad choices.
   loadEvent(eventNumber);
   //TH1F *padHist = loadPad(padNumber);
   padHist = loadPad(padNumber);
   if (padHist != nullptr)
      padHist->Draw("");
   padNum = padNumber;
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
   //double fitval = (x[0]>par[0])*(par[2]*TMath::Exp(-3*(x[0]-par[0])/par[1])*TMath::Sin((x[0]-par[0])/par[1])*TMath::Power((x[0]-par[0])/par[1], 3)>0)*par[2]*TMath::Exp(-3*(x[0]-par[0])/par[1])*TMath::Sin((x[0]-par[0])/par[1])*TMath::Power((x[0]-par[0])/par[1], 3);
   double fitval = (x[0]>par[0])*par[2]*TMath::Exp(-3*(x[0]-par[0])/par[1])*TMath::Sin((x[0]-par[0])/par[1])*TMath::Power((x[0]-par[0])/par[1], 3);
   return fitval;
}


//void window_run_fit(int left_bound, int right_bound, int xloc, int jump_height, int init_height) user defined initial parameters
//5 integer inputs. First 2 are the time-bucket bounds. 3rd is x-location of jump, 4th is jump height, and 5th is height before the jump (all approximated)
double window_run_fit()
{
   double left_bound = 440;
   double right_bound = 475;
   double xloc = 457.5;
   
   auto trace = rawEventPtr->GetPad(padNum)->GetADC();

   double max = *std::max_element(begin(trace), end(trace));
   double min = *std::min_element(begin(trace), end(trace));

   double jump_height = max-min;
   double init_height = std::abs(min);
   double temp = 1;

   TF1 *func = new TF1("func", WindowEquation, left_bound, right_bound, 4);

   func->SetParameters(xloc, jump_height, init_height, 1);
   func->SetParNames("xloc", "jump_height", "init_height", "temp");

   padHist->Fit("func","R");
   return func->GetParameter(0);
}


//void plane_run_fit(int left_bound, int right_bound)
double plane_run_fit()
{
   double left_bound = 70;
   double right_bound = 100;
   TF1 *func = new TF1("func", PlaneEquation, left_bound, right_bound, 3);

   func->SetParameters(78, 2.25, 25000);
   //func->SetParLimits(0,left_bound,1.e6);
   func->SetParLimits(2,0,1.e6);
   func->SetParNames("par0", "drift_time", "par2");

   padHist->Fit("func","R");
   return func->GetParameter(0);
}


void automater_plane()
{
   int event_ctr = 0;
   TH1 *h1 = new TH1D("h1", "h1",100, 70.0, 85.0);

   while(loadEvent(event_ctr)){
      cout << endl << "event number: " << event_ctr << endl << endl;

      std::vector<AtPad *> pad_vec = GetLikelyPadPlane();

      for(auto & pad : pad_vec){
         padNum = pad -> GetPadNum();
         padHist = loadPad(padNum);
         //if (padHist != nullptr)
         //   padHist->Draw("");
         cout << "pad number: " << padNum;
         double temp = plane_run_fit();
	 h1->Fill(temp);
      }
      event_ctr++;
   }
   h1->Draw();
}


void automater_window()
{
   int event_ctr = 0;
   TH1 *h1 = new TH1D("h1", "h1", 100, 200.0, 600.0);

   while(loadEvent(event_ctr)){
      cout << endl << "event number: " << event_ctr << endl << endl;

      std::vector<AtPad *> pad_vec = GetLikelyWindow();

      for(auto & pad : pad_vec){
         //loops through and fits all pads in the vector
         padNum = pad -> GetPadNum();
         padHist = loadPad(padNum);
         //if (padHist != nullptr)
         //   padHist->Draw("");
         cout << "pad number: " << padNum;
         double temp = window_run_fit();
         h1->Fill(temp);
      }
      event_ctr++;
   }
   h1->Draw();
}


#endif //ifndef MANUAL_C
