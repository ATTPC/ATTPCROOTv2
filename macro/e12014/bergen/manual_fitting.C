#ifndef MANUAL_C
#define MANUAL_C
#include "mira.C"
#include <cmath>

/*For fitting pad traces near window or plane
 *Using equation [p1]/(exp((x-[p0])/[p3])+1)-[p2] for window
 *Using equation [p2]*exp(-[p3]*(x-[p0])/[p1])*sin((x-[p0])/[p1])*((x-[p0])/[p1])^3 for plane
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
void plane_run_fit(double *pa0, double *pa1, double *pa2, double *pa3, double *chis);
void automater_plane();
void automater_window();
void automater_both();


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
   double fitval = (x[0]>par[0])*par[2]*TMath::Exp(-par[3]*(x[0]-par[0])/par[1])*TMath::Sin((x[0]-par[0])/par[1])*TMath::Power((x[0]-par[0])/par[1], 3);
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
void plane_run_fit(double *pa0, double *pa1, double *pa2, double *pa3, double *chis)
{
   double left_bound = 70;
   double right_bound = 100;
   TF1 *func = new TF1("func", PlaneEquation, left_bound, right_bound, 4);

   func->SetParameters(78, 2.25, 25000, 3);
   //func->SetParLimits(0,left_bound,1.e6);
   func->SetParLimits(2,0,1.e6);
   func->SetParLimits(3,2,20);
   func->SetParNames("par0", "drift_time", "par2", "par3");

   padHist->Fit("func","R");
   *pa0 = func->GetParameter(0);
   *pa1 = func->GetParameter(1);
   *pa2 = func->GetParameter(2);
   *pa3 = func->GetParameter(3);
   *chis = func->GetChisquare();
}


void automater_plane()
{
   int event_ctr = 0;
   double p0, p1, p2, p3, chisq;
   //TH1 *h1 = new TH1D("h1", "p0_histogram",100, 70.0, 85.0);
   //TH1 *h1 = new TH1D("h1", "p3_histogram",100, 2.0, 20.0);
   TH1 *h1 = new TH1D("h1", "chisquare", 100, 0.0, 250.0);

   while(loadEvent(event_ctr)){
      cout << endl << "event number: " << event_ctr << endl << endl;

      std::vector<AtPad *> pad_vec = GetLikelyPadPlane();

      for(auto & pad : pad_vec){
         padNum = pad -> GetPadNum();
         padHist = loadPad(padNum);
         //if (padHist != nullptr)
         //   padHist->Draw("");
         cout << "pad number: " << padNum;
         plane_run_fit(&p0, &p1, &p2, &p3, &chisq);
	 h1->Fill(chisq);
      }
      event_ctr++;
   }
   h1->Draw();
}


void automater_window()
{
   int event_ctr = 0;
   TH1 *h1 = new TH1D("h1", "p0_hist", 100, 400.0, 500.0);

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


void automater_both()
{
   int event_ctr = 0;
   double p0, p1, p2, p3, chisq;
   double plane_ctr, plane_sum, plane_mean, window_ctr, window_sum, window_mean, diff;
   TH1 *h1 = new TH1D("h1", "difference_hist", 100, 315.0, 430.0);

   while(loadEvent(event_ctr)){
      cout << endl << "event number: " << event_ctr << endl << endl;

      std::vector<AtPad *> pad_vec_plane = GetLikelyPadPlane();
      std::vector<AtPad *> pad_vec_window = GetLikelyWindow();
      plane_ctr = plane_sum = plane_mean = window_ctr = window_sum = window_mean = 0;

      for(auto & pad : pad_vec_plane){
         padNum = pad -> GetPadNum();
         padHist = loadPad(padNum);
         cout << "pad number: " << padNum;
         plane_run_fit(&p0, &p1, &p2, &p3, &chisq);
	 plane_sum += p0;
	 plane_ctr++;
      }
      for(auto & pad : pad_vec_window){
         padNum = pad->GetPadNum();
	 padHist = loadPad(padNum);
         cout << "pad number: " << padNum;
         window_sum += window_run_fit();
	 window_ctr++;
      }
      plane_mean = plane_sum/plane_ctr;
      window_mean = window_sum/window_ctr;
      diff = window_mean - plane_mean;
      h1->Fill(diff);
      event_ctr++;
   }
   h1->Draw();

}



#endif //ifndef MANUAL_C
