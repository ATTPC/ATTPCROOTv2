#ifndef ACROSS_C
#define ACROSS_C
#include "mira.C"
#include <cmath>

/*For fitting pad traces near window or plane
 *Using equation [p1]/(exp((x-[p0])/[p3])+1)-[p2] for window
 *Using equation [p2]*exp(-[p3]*(x-[p0])/[p1])*sin((x-[p0])/[p1])*((x-[p0])/[p1])^3 for plane
 *Load using root -l manual_fitting.C
 */

//functions declarations
void across_runs();
double WindowEquation(double *x, double *par);
double PlaneEquation(double *x, double *par);
double window_run_fit(double *chis);
void plane_run_fit(double *pa0, double *pa1, double *pa2, double *pa3, double *chis, double *maxloc);
void process_run(int run_number);
void save_data();
void plotting_across_runs();


//variable declarations
TH1F *padHist;
int padNum;
double holder = 0.0;

void across_runs()
{
   cout << "script loaded" << endl;
   //loadRun("/mnt/analysis/e12014/TPC/unpackedReducedFiltered/run_0210.root");
}


double WindowEquation(double *x, double *par)
{
   double fitval = par[1] / (TMath::Exp((x[0] - par[0]) / par[3]) + 1) - par[2];
   return fitval;
}


double PlaneEquation(double *x, double *par)
{
   double fitval = (x[0]>par[0])*par[2]*TMath::Exp(-par[3]*(x[0]-par[0])/par[1])*TMath::Sin((x[0]-par[0])/par[1])*TMath::Power((x[0]-par[0])/par[1], 3);
   return fitval;
}


double window_run_fit(double *chis = &holder)
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
   *chis = func->GetChisquare();
   return func->GetParameter(0);
}


void plane_run_fit(double *pa0 = &holder, double *pa1 = &holder, double *pa2 = &holder, double *pa3 = &holder, double *chis = &holder, double *maxloc = &holder)
{
   double left_bound = 70;
   double right_bound = 100;
   TF1 *func = new TF1("func", PlaneEquation, left_bound, right_bound, 4);

   func->SetParameters(78, 2.25, 25000, 3);
   //func->SetParLimits(0,left_bound,1.e6);
   func->SetParLimits(2,0,1.e6);
   func->SetParLimits(3,0,20);
   func->SetParNames("par0", "drift_time", "par2", "par3");

   padHist->Fit("func","R");
   *pa0 = func->GetParameter(0);
   *pa1 = func->GetParameter(1);
   *pa2 = func->GetParameter(2);
   *pa3 = func->GetParameter(3);
   *chis = func->GetChisquare();
   *maxloc = func->GetMaximumX(70, 100);
   //cout << "max location: " << *maxloc << endl;
}


void process_run(int run_number)
{
   //begin section for linear
   /*int num_events = 0;
   double xarr [num_events];
   double yarr [num_events];*/
   //end section for linear

   int event_ctr = 0;
   double p0, p1, p2, p3, chisq, max_loc, wind_temp;
   double plane_ctr, plane_sum, plane_mean, window_ctr, window_sum, window_mean, diff;
   std::string title_str = "h" + to_string(run_number);
   const char *title = title_str.c_str();
   TH1 *h1 = new TH1D(title, "difference_hist_using_p0", 100, 315.0, 430.0); //for histogram

   while(loadEvent(event_ctr)){
      cout << endl << "event number: " << event_ctr << endl << endl;

      std::vector<AtPad *> pad_vec_plane = GetLikelyPadPlane();
      std::vector<AtPad *> pad_vec_window = GetLikelyWindow();
      plane_ctr = plane_sum = plane_mean = window_ctr = window_sum = window_mean = 0;

      for(auto & pad : pad_vec_plane){
         padNum = pad -> GetPadNum();
         padHist = loadPad(padNum);
         cout << "pad number: " << padNum;
         plane_run_fit(&p0, &p1, &p2, &p3, &chisq, &max_loc);
	 if(p0 >= 74 && p0 <= 77 && max_loc >= 72)
	 {
	    plane_sum += p0;
	    plane_ctr++;
	 }
      }
      for(auto & pad : pad_vec_window){
         padNum = pad->GetPadNum();
	 padHist = loadPad(padNum);
         cout << "pad number: " << padNum;
	 wind_temp = window_run_fit();
	 if(wind_temp >= 446)
	 {
            window_sum += wind_temp;
	    window_ctr++;
	 }
      }
      plane_mean = plane_sum/plane_ctr;
      window_mean = window_sum/window_ctr;
      diff = window_mean - plane_mean;
      h1->Fill(diff); //for histogram
      //xarr[event_ctr] = event_ctr; //for linear
      //yarr[event_ctr] = diff; //for linear
      event_ctr++;
   }
   
   //h1->Draw(); //for histogram
   h1->Write();
   //double max_tbhist = h1->GetMaximumBin(); //for histogram
   //h1->Fit("gaus","","",381,386); //373 to 380 //for histogram

   //TGraph *g = new TGraph(num_events, xarr, yarr); //for linear
   //g->Draw("ap"); //for linear
   //g->GetYaxis()->SetRangeUser(300,450); //for linear
}

void save_data()
{
   TFile f("runhistos.root", "new");
   //int run_nums [53] = {200, 201, 202, 203, 204, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 219, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 250, 251, 252, 253, 254, 255, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271};

   int run_nums [3] = {200, 201, 202};
   
   for (int cur_run : run_nums){
      loadRun("/mnt/analysis/e12014/TPC/unpackedReducedFiltered/run_0" + to_string(cur_run) + ".root");
      process_run(cur_run);
   }
   f.Close();
}


void plotting_across_runs()
{
   TFile f("runhistos.root");
   string title_str;
   const char *title;

   int run_nums [3] = {200, 201, 202};
   double run_nums_dbl [3] = {200, 201, 202};
   double yarr [3];
   int ctr = 0;

   for (int cur_run : run_nums){
      title_str = "h" + to_string(cur_run);
      title = title_str.c_str();
      TH1D *h1 = (TH1D*)f.Get(title);
      double binmax = h1->GetMaximumBin();
      double max_val = h1->GetXaxis()->GetBinCenter(binmax);
      yarr[ctr] = max_val;
      ctr++;
   }
   TGraph *g = new TGraph(3, run_nums_dbl, yarr);
   g->Draw("ap*");
}


#endif //ifndef ACROSS_C
