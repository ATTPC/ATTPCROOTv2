#include <TMath.h>
#include <TGraph2D.h>
#include <TRandom2.h>
#include <TStyle.h>
#include <TCanvas.h>
#include <TF2.h>
#include <TH1.h>
#include <Math/Functor.h>
#include <TPolyLine3D.h>
#include <Math/Vector3D.h>
#include <Fit/Fitter.h>
#include "AtTrackTransformer.h"
#include <cassert>
 
using namespace ROOT::Math;

Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

void line(double t, const double *p, double &x, double &y, double &z) {
   // a parametric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = p[0] + p[1]*t;
   y = p[2] + p[3]*t;
   z = t;
}

 
bool first = true;
struct SumDistance2 {
   // the TGraph is a data member of the object
   TGraph2D *fGraph;
 
   SumDistance2(TGraph2D *g) : fGraph(g) {}
 
   // calculate distance line-point
   double distance2(double x,double y,double z, const double *p) {
      // distance line point is D= | (xp-x0) cross  ux |
      // where ux is direction of line and x0 is a point in the line (like t = 0)
      XYZVector xp(x,y,z);
      XYZVector x0(p[0], p[2], 0. );
      XYZVector x1(p[0] + p[1], p[2] + p[3], 1. );
      XYZVector u = (x1-x0).Unit();
      double d2 = ((xp-x0).Cross(u)).Mag2();
      return d2;
   }
 
   // implementation of the function to be minimized
   double operator() (const double *par) {
      assert(fGraph != nullptr);
      double * x = fGraph->GetX();
      double * y = fGraph->GetY();
      double * z = fGraph->GetZ();
      int npoints = fGraph->GetN();
      double sum = 0;
      for (int i  = 0; i < npoints; ++i) {
         double d = distance2(x[i],y[i],z[i],par);
         sum += d;
      }
      if (first) {
         std::cout << "Total Initial distance square = " << sum << std::endl;
      }
      first = false;
      return sum;
   }
 
};

std::tuple<double, double>
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject)
{

   // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
   double Et1 = K_proj + m1;
   double Et2 = m2;
   double Et3 = K_eject + m3;
   double Et4 = Et1 + Et2 - Et3;
   double m4_ex, Ex, theta_cm;
   double s, t, u; //---Mandelstam variables

   s = pow(m1, 2) + pow(m2, 2) + 2 * m2 * Et1;
   u = pow(m2, 2) + pow(m3, 2) - 2 * m2 * Et3;

   m4_ex = sqrt((cos(thetalab) * omega(s, pow(m1, 2), pow(m2, 2)) * omega(u, pow(m2, 2), pow(m3, 2)) -
                 (s - pow(m1, 2) - pow(m2, 2)) * (pow(m2, 2) + pow(m3, 2) - u)) /
                   (2 * pow(m2, 2)) +
                s + u - pow(m2, 2));
   Ex = m4_ex - m4;

   t = pow(m2, 2) + pow(m4_ex, 2) - 2 * m2 * Et4;

   // for inverse kinematics Note: this angle corresponds to the recoil
   theta_cm = TMath::Pi() - acos((pow(s, 2) + s * (2 * t - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m3, 2) - pow(m4_ex, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m3, 2), pow(m4_ex, 2))));

   theta_cm = theta_cm * TMath::RadToDeg();
   return std::make_tuple(Ex, theta_cm);
}
double_t npointsinside = 0;
double_t convert = 0.0065688;
void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E);

void C14_pp_ana()
{
   std::vector<double> bro_values;
   std::vector<double> eneralpha_values;
   std::vector<double> vertexener_values;
   std::vector<double> eneralpha_values_tb;

   std::ifstream file("/home/david/PhD/PhD-14-02/attpcroot/ATTPCROOTv2/be10_range.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    std::string line;
    std::vector<std::string> ener;
    std::vector<std::string> range;
    std::vector<std::string> stopping_power;
    vector<double> Ebeam_tb_all;
    vector<double> Ebeam_a_all;
    vector<double> Ebeam_cm_values;
    vector<double> angles_cm_values;
    // Skip the header row
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        // Read each cell in the row
        while (std::getline(ss, cell, '\t')) {
            row.push_back(cell);
        }

        // Ensure the row has at least 4 columns
        if (row.size() >= 4) {
            ener.push_back(row[0]);
            double stopping_pwr = (std::stod(row[1]) + std::stod(row[2]))*convert;
            stopping_power.push_back(std::to_string(stopping_pwr));
            range.push_back(row[3]);
        }
    }
    

    file.close();  

    int n = range.size();
    std::vector<double> x(n), y(n), y2(n);
    for (int i = 0; i < n; ++i) {
        x[i] = std::stod(range[i]);
        y[i] = std::stod(ener[i]);
        y2[i] = std::stod(stopping_power[i]);
    }
   
    // Create TGraph
    TGraph *interpsrim = new TGraph(n, x.data(), y.data()); 
    TGraph *stopping_power_vs_energy = new TGraph(n, y.data(), y2.data());
    TGraph *interpsrimenergy = new TGraph(n, y.data(), x.data());
    double_t ener_0 = 7.35; //MeV
    double_t energy_i = ener_0;
    std::vector<double> energy_vs_distance;
    energy_vs_distance.push_back(energy_i);

    while(energy_i > 0.01){
      double_t stp = stopping_power_vs_energy->Eval(energy_i);
      energy_i = energy_i - stp;
      energy_vs_distance.push_back(energy_i);
    }

   std::vector<double> distances(energy_vs_distance.size());
    for (size_t i = 0; i < distances.size(); ++i) {
        distances[i] = static_cast<double>(i);
    }

   TGraph *energy_vs_distance_gr = new TGraph(distances.size(), distances.data(), energy_vs_distance.data());

   double_t tb_entrance = 290.0;
   double_t max_range = 1000 - 240.; //beam max range in mm with this conditions (pressure, energy...)
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 500, 0, 3);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 500, 0, 3);
   TH2F *bro_vs_angle = new TH2F("bro_vs_angle", "bro_vs_angle", 1000,0,180,1000,0,20);
   TH2F *bro_vs_alphaenergy = new TH2F("bro_vs_alphaenergy", "bro_vs_alphaenergy", 1000,0,20,1000,0,20);
   TH2F *radius_vs_alphaenergy = new TH2F("radius_vs_alphaenergy", "radius_vs_alphaenergy", 1000,0,10,1000,0,100);
   TH2F *bro_vs_radius = new TH2F("bro_vs_radius", "bro_vs_radius", 1000,0,100,100,0,2);
   //TH2F *angle_vs_alphaenergy = new TH2F("angle_vs_alphaenergy", "angle_vs_alphaenergy", 100,0,20,1000,0,180);
   TH2F *eneralpha_vs_vertexener = new TH2F("eneralpha_vs_vertexener", "eneralpha_vs_vertexener", 1000, -10, 10.0, 1000, 0, 10.0);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 1000, 0, 90., 1000, 0, 20.0);
   TH2F *angle_vs_energy_lr_gscut = new TH2F("angle_vs_energy_lr_gscut", "angle_vs_energy_lr_gscut", 1000, 0, 90., 1500, 0, 30.0);
   TH2F *angle_vs_energy_lr_1stcut = new TH2F("angle_vs_energy_lr_1stcut", "angle_vs_energy_lr_1stcut", 1000, 0, 90., 1500, 0, 30.0);
   TH2F *angle_vs_energy_t = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);
   TH2F *firstpeak = new TH2F("firstpeak", "firstpeak", 1000, 0, 90., 100, 0, 3.0);
   TH2F *secondpeak = new TH2F("secondpeak", "secondpeak", 1000, 0, 90., 100, 0, 3.0);
   TH2F *angle_vs_angle = new TH2F("angle_vs_angle", "angle_vs_angle", 720, 0, 179, 720, 0, 179);
   TH2F *ecm_anglecm = new TH2F("ecm_anglecm", "ecm_anglecm", 1000, 0, 180.0, 1000, 0, 15.0);
   TH2F *ecm_anglecm_l1 = new TH2F("ecm_anglecm_l1", "ecm_anglecm_l1", 1000, 0, 180.0, 1000, 0, 2.0);
   TH2F *ecm_anglecm_l2 = new TH2F("ecm_anglecm_l2", "ecm_anglecm_l2", 1000, 0, 180.0, 1000, 0, 2.0);
   TH2F *ecm_anglecm_vertex = new TH2F("ecm_anglecm_vertex", "ecm_anglecm_vertex", 1000, 0, 180.0, 1000, -5.0, 2.0);
   TH2F *ecm_anglecm_vertex_l1 = new TH2F("ecm_anglecm_vertex_l1", "ecm_anglecm_vertex_l1", 1000, 0, 180.0, 1000, -5.0, 2.0);
   TH2F *ecm_anglecm_vertex_l2 = new TH2F("ecm_anglecm_vertex_l2", "ecm_anglecm_vertex_l2", 1000, 0, 180.0, 1000, -5.0, 2.0);
   TH1F *vertex_distribution = new TH1F("vertex_distribution", "Vertex", 200, -1000, 2000);
   TH1F *vertex_energy = new TH1F("vertex_energy", "Vertex Energy", 400, -20., 20.);
   TH1F *vertex_energy_tb = new TH1F("vertex_energy_tb", "Vertex Energy using TB", 600, -20, 40);
   TH1F *timebucket = new TH1F("timebucket", "Time Bucket", 1000, 0, 1000);
   TH1F *charge_line1 = new TH1F("charge_line1", "charge_line1", 1000000, 1000000, 50000000);
   TH1F *charge_line2 = new TH1F("charge_line2", "charge_line2", 1000000, 1000000, 50000000);
   TH1F *ex_ener_120 = new TH1F("ex_ener_120", "ex_ener_120", 100, 0, 15.0);
   TH1F *ex_ener_110 = new TH1F("ex_ener_110", "ex_ener_110", 100, 0, 15.0);
   TH1F *ex_ener_100 = new TH1F("ex_ener_100", "ex_ener_100", 100, 0, 15.0);
   TH1F *ex_ener_90 = new TH1F("ex_ener_90", "ex_ener_90", 100, 0, 15.0);
   TH1F *ex_ener_80 = new TH1F("ex_ener_80", "ex_ener_80", 100, 0, 15.0);
   TH1F *ex_ener_70 = new TH1F("ex_ener_70", "ex_ener_70", 100, 0, 15.0);
   TH1F *ex_ener_60 = new TH1F("ex_ener_60", "ex_ener_60", 100, 0, 15.0);
   TH1F *radius = new TH1F("radius", "radius", 100, 0, 100);
   TH2F *vertex_vs_beamenergy = new TH2F("vertex_vs_beamenergy", "Vertex and beam energy",1000, 0., 1000, 600, -20., 40.);
   TH2F *vertex_vs_vertex = new TH2F("vertex_vs_vertex", "Vertex TB vs Vertex alpha",600, -20., 40., 600, -20., 40.);


   TH1F *HQval = new TH1F("HQval", "HQval", 600, -5, 55);
   TH2F *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);

   TCutG *cutg_line1 = new TCutG("CUTG_firstline",13);
   cutg_line1->SetVarX("angle_vs_energy_lr");
   cutg_line1->SetVarY("");
   cutg_line1->SetTitle("Graph");
   cutg_line1->SetFillStyle(1000);
   cutg_line1->SetPoint(0,37.6097,5.03447);
   cutg_line1->SetPoint(1,40.256,4.92628);
   cutg_line1->SetPoint(2,43.3541,4.49351);
   cutg_line1->SetPoint(3,47.0977,4.08479);
   cutg_line1->SetPoint(4,49.4213,3.76022);
   cutg_line1->SetPoint(5,49.3567,3.30342);
   cutg_line1->SetPoint(6,48.4531,3.14714);
   cutg_line1->SetPoint(7,44.9032,3.26735);
   cutg_line1->SetPoint(8,42.386,3.72416);
   cutg_line1->SetPoint(9,40.1269,4.20501);
   cutg_line1->SetPoint(10,37.287,4.55362);
   cutg_line1->SetPoint(11,37.9324,5.02244);
   cutg_line1->SetPoint(12,37.6097,5.03447);

   TCutG *cutg_line2 = new TCutG("CUTG_secondline",12);
   cutg_line2->SetVarX("angle_vs_energy_lr");
   cutg_line2->SetVarY("");
   cutg_line2->SetTitle("Graph");
   cutg_line2->SetFillStyle(1000);
   cutg_line2->SetPoint(0,31.6071,5.25085);
   cutg_line2->SetPoint(1,37.4161,4.20501);
   cutg_line2->SetPoint(2,41.676,3.48373);
   cutg_line2->SetPoint(3,44.4514,2.91874);
   cutg_line2->SetPoint(4,42.6441,2.38981);
   cutg_line2->SetPoint(5,40.5787,2.2696);
   cutg_line2->SetPoint(6,37.287,2.91874);
   cutg_line2->SetPoint(7,33.2207,3.90448);
   cutg_line2->SetPoint(8,29.4772,4.83011);
   cutg_line2->SetPoint(9,30.5099,5.34702);
   cutg_line2->SetPoint(10,31.6071,5.25085);
   cutg_line2->SetPoint(11,31.6071,5.25085);

   TCutG *cutg_lowener = new TCutG("CUTG",14);
   cutg_lowener->SetVarX("angle_vs_angle");
   cutg_lowener->SetVarY("");
   cutg_lowener->SetTitle("Graph");
   cutg_lowener->SetFillStyle(1000);
   cutg_lowener->SetPoint(0,39.625,23.4881);
   cutg_lowener->SetPoint(1,44.3098,24.2644);
   cutg_lowener->SetPoint(2,49.0539,23.0445);
   cutg_lowener->SetPoint(3,56.5258,20.383);
   cutg_lowener->SetPoint(4,65.5989,17.0561);
   cutg_lowener->SetPoint(5,71.7069,13.3964);
   cutg_lowener->SetPoint(6,70.7581,11.4003);
   cutg_lowener->SetPoint(7,66.5477,10.2913);
   cutg_lowener->SetPoint(8,59.0165,13.2855);
   cutg_lowener->SetPoint(9,51.4259,17.6105);
   cutg_lowener->SetPoint(10,45.3179,19.3849);
   cutg_lowener->SetPoint(11,39.6843,23.1554);
   cutg_lowener->SetPoint(12,39.8622,24.0426);
   cutg_lowener->SetPoint(13,39.625,23.4881);

   TCutG *cutg_highener_gs = new TCutG("CUTG_HE_GS",13);
   cutg_highener_gs->SetVarX("angle_vs_angle");
   cutg_highener_gs->SetVarY("");
   cutg_highener_gs->SetTitle("Graph");
   cutg_highener_gs->SetFillStyle(1000);
   cutg_highener_gs->SetPoint(0,36.7706,27.9542);
   cutg_highener_gs->SetPoint(1,45.5199,25.3528);
   cutg_highener_gs->SetPoint(2,54.8526,21.7216);
   cutg_highener_gs->SetPoint(3,65.1185,16.6272);
   cutg_highener_gs->SetPoint(4,75.501,12.3458);
   cutg_highener_gs->SetPoint(5,78.7675,10.2322);
   cutg_highener_gs->SetPoint(6,75.7344,8.55208);
   cutg_highener_gs->SetPoint(7,65.2351,13.5381);
   cutg_highener_gs->SetPoint(8,54.2693,18.4699);
   cutg_highener_gs->SetPoint(9,42.7202,22.3178);
   cutg_highener_gs->SetPoint(10,37.0039,24.4856);
   cutg_highener_gs->SetPoint(11,37.3539,27.8458);
   cutg_highener_gs->SetPoint(12,36.7706,27.9542);

   TCutG *cutg_highener_1st = new TCutG("CUTG_HE_1ST",9);
   cutg_highener_1st->SetVarX("angle_vs_angle");
   cutg_highener_1st->SetVarY("");
   cutg_highener_1st->SetTitle("Graph");
   cutg_highener_1st->SetFillStyle(1000);
   cutg_highener_1st->SetPoint(0,35.8373,20.8545);
   cutg_highener_1st->SetPoint(1,34.6707,18.2531);
   cutg_highener_1st->SetPoint(2,41.7869,14.9472);
   cutg_highener_1st->SetPoint(3,48.903,10.8825);
   cutg_highener_1st->SetPoint(4,50.4196,9.69019);
   cutg_highener_1st->SetPoint(5,54.7359,10.3405);
   cutg_highener_1st->SetPoint(6,47.0365,15.7601);
   cutg_highener_1st->SetPoint(7,35.954,20.8545);
   cutg_highener_1st->SetPoint(8,35.8373,20.8545);
   



   // NB: Not used
   // Q-value calculation
   Double_t m_p = 1.007825 * 931.49401;
   Double_t m_d = 2.0135532 * 931.49401;
   Double_t m_t = 3.016049281 * 931.49401;
   Double_t m_He3 = 3.016029 * 931.49401;
   Double_t m_Be10 = 10.013533818 * 931.49401;
   Double_t m_Be11 = 11.021657749 * 931.49401;
   Double_t m_Li9 = 9.026790 * 931.49401;
   Double_t m_beam = m_Be10;
   Float_t aMass = 4.00260325415;
   Float_t O16Mass = 15.99491461956;
   Float_t Be10Mass = 10.013533818;
   Double_t m_C14 = 14.003242 * 931.49401;
   Double_t m_C13 = 13.00335484 * 931.49401;
   Double_t m_C12 = 12.00 * 931.49401;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 18.1;
   Double_t m_b;
   Double_t m_B;
   ULong64_t countertracks = 0;
   m_b = m_a;
   m_B = m_Be10;
   AtTools::AtTrackTransformer transformer;
   //TString FileName = "run_0062.root";
   // std::cout << " Opening File : " << FileName.Data() << std::endl;
   // TFile *file = new TFile(FileName.Data(), "READ");

   TString dir = "/home/david/PhD/PhD-14-02/attpcroot/ATTPCROOTv2/macro/Unpack_HDF5/a1954/";

   //"/home/david/PhD/PhD-14-02/attpcroot/ATTPCROOTv2/macro/Simulation/ATTPC/10Be_aa/data/"

   //"/media/david/EXTERNAL_USB/e22502/low_energy/"

   std::vector<TString> files{"run_0062.root","run_0063.root","run_0064.root","run_0065.root","run_0067.root"};

   //"output_lowener_digi_0.root"

   /*"run_0062.root","run_0064.root","run_0065.root","run_0066.root",
   "run_0067.root","run_0070.root","run_0071.root","run_0072.root","run_0073.root","run_0074.root",
   "run_0075.root","run_0076.root","run_0077.root","run_0078.root","run_0079.root","run_0080.root",
   "run_0081.root","run_0082.root","run_0083.root","run_0084.root","run_0085.root","run_0086.root",
   "run_0087.root","run_0089.root","run_0090.root","run_0091.root","run_0092.root",
   "run_0093.root","run_0094.root","run_0095.root","run_0096.root","run_0097.root","run_0098.root",
   "run_0099.root","run_0100_old.root","run_0101_old.root","run_0102_old.root","run_0103_old.root","run_0104_old.root",
   "run_0105_old.root"*/
/*"run_0116.root", "run_0117.root", "run_0118.root", "run_0119.root", "run_0120.root",
                              "run_0134.root", "run_0135.root", "run_0136.root", "run_0137.root", "run_0138.root", 
                              "run_0139.root", "run_0140.root", "run_0141.root", "run_0143.root", "run_0144.root", 
                              "run_0145.root", "run_0146.root", "run_0147.root", "run_0148.root", "run_0149.root", 
                              "run_0150.root", "run_0153.root", "run_0156.root", "run_0157.root", "run_0158.root",
                              "run_0159.root", "run_0160.root", "run_0161.root", "run_0162.root", "run_0163.root",
                              "run_0164.root", "run_0165.root", "run_0166.root", "run_0167.root", "run_0168.root",
                              "run_0169.root", "run_0170.root", "run_0171.root", "run_0172.root", "run_0173.root",
                              "run_0174.root", "run_0175.root"
*/

   for (auto iFile : files) {

      TString filePath = dir + iFile;
      TFile *file = new TFile(filePath.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
      TTreeReaderValue<TClonesArray> events(Reader1, "AtEventH");
    

      for (Int_t i = 0; i < nEvents; i++) {

         // eventArray->Clear();
         //if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         Reader1.Next();


         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);
         AtEvent *event = (AtEvent *)events->At(0);
         

         if (patternEvent) {
            
            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            std::vector<AtTrack> newTracks;  
            countertracks += patternTrackCand.size();
            // std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

            // Find track with largest angle
            auto itMax =
               std::max_element(patternTrackCand.begin(), patternTrackCand.end(),
                                [](const auto &a, const auto &b) { return b.GetGeoTheta() > a.GetGeoTheta(); });
            Int_t maxAIndex = std::distance(patternTrackCand.begin(), itMax);

            std::sort(patternTrackCand.begin(), patternTrackCand.end(),
              [](const AtTrack &a, const AtTrack &b) {
                  return a.GetHitArray().size() > b.GetHitArray().size();
              });

              /*std::sort(patternTrackCand.begin(), patternTrackCand.end(),
              [](const AtTrack &a, const AtTrack &b) {
                  return a.GetHitArray().at(0)->GetTimeStamp() > b.GetHitArray().at(0)->GetTimeStamp();
              });*/
               //std::cout << "PatterTrackCand Size = "<< patternTrackCand.size() << std::endl;
            
              
            // for (auto track : patternTrackCand) {
            

                     if (patternTrackCand.size() > 1){
                        
                     auto track1 = patternTrackCand.at(0);
                     auto track2 = patternTrackCand.at(1);
                     
                     bool skiptrack1 = false;
                     bool skiptrack2 = false;
                     
                     
                     

                     Double_t theta1 = track1.GetGeoTheta();
                     if(theta1 * TMath::RadToDeg() > 90.0) 
                        theta1 = TMath::Pi() - theta1;
                     Double_t theta2 = track2.GetGeoTheta();  
                     if(theta2 * TMath::RadToDeg() > 90.0) 
                        theta2 = TMath::Pi() - theta2;     
                     //std::cout << "Theta 1: " << theta1 << " Theta 2: " << theta2 << std::endl;
                     auto hitArray1 = track1.GetHitArrayObject();
                     //std::cout << "Number of Hits in Track 1: " << hitArray1.size() << std::endl;
                     auto hitArray2 = track2.GetHitArrayObject(); 
                     //std::cout << "Number of Hits in Track 2: " << hitArray2.size() << std::endl;
                     
                     for (const auto &hit : hitArray1) {
                      if (hit.GetTimeStamp() < 60) {
                      skiptrack1 = true;
                      break;
                      }
                     }

                     for (const auto &hit : hitArray2) {
                      if (hit.GetTimeStamp() < 60) {
                      skiptrack2 = true;
                      break;
                      }
                     }
                        
                    /* if (skiptrack1 && patternTrackCand.size() > 2) {
                        track1 = patternTrackCand.at(2);
                        hitArray1 = track1.GetHitArrayObject();
                        theta1 = track1.GetGeoTheta();
                        if(theta1 * TMath::RadToDeg() > 90.0) 
                           theta1 = TMath::Pi() - theta1;
                     }

                     if (skiptrack2 && patternTrackCand.size() > 2) {
                        track2 = patternTrackCand.at(2);
                        hitArray2 = track2.GetHitArrayObject();
                        theta2 = track1.GetGeoTheta();
                        if(theta2 * TMath::RadToDeg() > 90.0) 
                           theta2 = TMath::Pi() - theta2;
                     }
*/
                    // if (skiptrack1 || skiptrack2)
                      //  continue;
                     
                     for (const auto& hit : hitArray1) {
                        timebucket->Fill(hit.GetTimeStamp());
                     }

                     for (const auto& hit : hitArray2) {
                        timebucket->Fill(hit.GetTimeStamp());
                     }
                     
                     //auto firstPoint1 = hitArray1(0);
                     //auto firstPoint2 = hitArray2(0);
                     Double_t radius1 = track1.GetGeoRadius();
                        Double_t radius2 = track2.GetGeoRadius(); 
                        radius->Fill(radius1);
                        radius->Fill(radius2);
                        Double_t B_f = 2.00;

                        double brotrack1 = 2.00 * radius1 / TMath::Sin(theta1) / 1000.0;
                        double brotrack2 = 2.00 * radius2 / TMath::Sin(theta2) / 1000.0;
                       
                        bro_vs_angle->Fill(theta1 * TMath::RadToDeg(), brotrack1);
                        bro_vs_angle->Fill(theta2 * TMath::RadToDeg(), brotrack2);
                        double enertrack1 = 0;
                        double enertrack2 = 0;
                        Double_t Am = 4.0;

                        GetEnergy(Am, 2.0, brotrack1, enertrack1);
                        GetEnergy(Am, 2.0, brotrack2, enertrack2);
                        Double_t p_ej1 = brotrack1 * 2.0 * 2.99792458 / 10 * 1000;
                        Double_t p_ej2 = brotrack2 * 2.0 * 2.99792458 / 10 * 1000;
                        Double_t E_ej1 = TMath::Sqrt(p_ej1 * p_ej1 + m_a * m_a) - m_a;
                        Double_t E_ej2 = TMath::Sqrt(p_ej2 * p_ej2 + m_a * m_a) - m_a;
                           //std::cout << "Check " << std::endl;
                         if(theta1 >= theta2){
                           
                        bro_vs_alphaenergy->Fill(E_ej1, brotrack1);
                        radius_vs_alphaenergy->Fill(E_ej1, radius1);
                        bro_vs_radius->Fill(radius1, brotrack1);
                        if(i == 9){
                       // std::cout << "Event: " << i << " Ener: " << E_ej1 << " Bro: " << brotrack1 << " Radius: " << radius1 << "Angle: " << (theta1 * TMath::RadToDeg()) << std::endl;
                        }
                        //angle_vs_alphaenergy->Fill(E_ej1, theta1 * TMath::RadToDeg());
                         }
                        if(theta1 < theta2){
                        if(i == 9) {                       
                        //std::cout << "Event: " << i << " Ener: " << E_ej2 << " Bro: " << brotrack2 << " Radius: " << radius2 << "Angle: " << (theta2 * TMath::RadToDeg()) << std::endl;
                        }
                        bro_vs_alphaenergy->Fill(E_ej2, brotrack2);
                        radius_vs_alphaenergy->Fill(E_ej2, radius2);
                        bro_vs_radius->Fill(radius2, brotrack2);
                        }
                  
                     if(theta1 >= theta2) 
                     angle_vs_angle->Fill(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg());
                     
                     
                     if(theta1 < theta2)
                     angle_vs_angle->Fill(theta2 * TMath::RadToDeg(), theta1 * TMath::RadToDeg());
                     //std::cout << "First Z track 1: " << firstPoint1.GetPosition().Z() << std::endl;
                     //std::cout << "First Z track 2: " << firstPoint2.GetPosition().Z() << std::endl;

                     auto hitClusterArray1 = track1.GetHitClusterArray();
                     //std::cout << "Number of Hits in Track 1: " << hitClusterArray1->size() << std::endl;
                     
                     //std::cout << "Number of Hits in Track 1: " << hitClusterArray1->size() << std::endl;
                  
                     auto firstCluster1 = hitClusterArray1->back();
                     auto zpos1 = firstCluster1.GetPosition().Z();
                     //std::cout << "Checking track 1" << std::endl;
                     auto hitClusterArray2 = track2.GetHitClusterArray();
                     //std::cout << "Number of Hits in Track 2: " << hitClusterArray2->size() << std::endl;
                     auto firstCluster2 = hitClusterArray2->back();
                     
                     auto zpos2 = firstCluster2.GetPosition().Z();               
                     
                     //std::cout << "First hit in track 1: " << hitArray1[0].GetPosition().Z() << std::endl;
                     //std::cout << "First hit in track 2: " << hitArray2[0].GetPosition().Z() << std::endl;
                     
                     /*if(hitClusterArray1->size() > 20 && hitClusterArray2->size() > 20){
                     if(theta1 >= theta2) 
                     angle_vs_angle->Fill(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg());
                     
                     
                     if(theta1 < theta2)
                     angle_vs_angle->Fill(theta2 * TMath::RadToDeg(), theta1 * TMath::RadToDeg());
                     }*/
                     
                     if(cutg_lowener->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg()))
                        npointsinside++;

                     
                     vector<double> p1_line1;
                     vector<double> p2_line1;
                     vector<double> p1_line2;
                     vector<double> p2_line2;
                     
                     if (hitClusterArray1->size() > 3 && hitClusterArray2->size() > 3) {
                        //std::cout << "Track 1: " << hitClusterArray1->size() << " Track 2: " << hitClusterArray2->size() << std::endl;
                        
                        auto onep = hitClusterArray1->at(hitClusterArray1->size() - 1);
                        auto twop = hitClusterArray1->at(hitClusterArray1->size() - 2);
                        
                        auto onep2 = hitClusterArray2->at(hitClusterArray2->size() - 1);
                        auto twop2 = hitClusterArray2->at(hitClusterArray2->size() - 2);
                        
                        p1_line1 = {onep.GetPosition().X(), onep.GetPosition().Y(), onep.GetPosition().Z()};
                        p2_line1 = {twop.GetPosition().X(), twop.GetPosition().Y(), twop.GetPosition().Z()};

                        p1_line2 = {onep2.GetPosition().X(), onep2.GetPosition().Y(), onep2.GetPosition().Z()};
                        p2_line2 = {twop2.GetPosition().X(), twop2.GetPosition().Y(), twop2.GetPosition().Z()};
                  
                        vector<double> dirvec1 = {p2_line1[0] - p1_line1[0], p2_line1[1] - p1_line1[1], p2_line1[2] - p1_line1[2]};
                        vector<double> dirvec2 = {p2_line2[0] - p1_line2[0], p2_line2[1] - p1_line2[1], p2_line2[2] - p1_line2[2]};

                        double s = ((dirvec1[0]*dirvec1[0] + dirvec1[1]*dirvec1[1] + dirvec1[2]*dirvec1[2])*(p1_line1[0]*dirvec2[0] - p1_line2[0]*dirvec2[0] + p1_line1[1]*dirvec2[1]-p1_line2[1]*dirvec2[1] + p1_line1[2]*dirvec2[2] - p1_line2[2]*dirvec2[2]) + (dirvec1[0]*dirvec2[0] + dirvec1[1]*dirvec2[1] + dirvec1[2]*dirvec2[2])*(p1_line2[0]*dirvec1[0] - p1_line1[0]*dirvec1[0] + p1_line2[1]*dirvec1[1] - p1_line1[1]*dirvec1[1] + p1_line2[2]*dirvec1[2] - p1_line1[2]*dirvec1[2]))/((dirvec2[0]*dirvec2[0] + dirvec2[1]*dirvec2[1] + dirvec2[2]*dirvec2[2])*(dirvec1[0]*dirvec1[0] + dirvec1[1]*dirvec1[1] + dirvec1[2]*dirvec1[2]) - (dirvec1[0]*dirvec2[0] + dirvec1[1]*dirvec2[1] + dirvec1[2]*dirvec2[2])*(dirvec1[0]*dirvec2[0] + dirvec1[1]*dirvec2[1] + dirvec1[2]*dirvec2[2]));
                        double t = (s*(dirvec2[0]*dirvec2[0] + dirvec2[1]*dirvec2[1] + dirvec2[2]*dirvec2[2]) + p1_line2[0]*dirvec2[0] - p1_line1[0]*dirvec2[0] + p1_line2[1]*dirvec2[1] - p1_line1[1]*dirvec2[1] + p1_line2[2]*dirvec2[2] - p1_line1[2]*dirvec2[2])/(dirvec1[0]*dirvec2[0] + dirvec1[1]*dirvec2[1] + dirvec1[2]*dirvec2[2]);

                        double z1 = p1_line1[2] + t*dirvec1[2];
                        double z2 = p1_line2[2] + s*dirvec2[2];

                        double finalz = (z1 + z2)/2.0;   

                        /*
                        double a = dirvec1[0], b = -dirvec2[0], c = p1_line2[0] - p1_line1[0];
                        double d = dirvec1[1], e = -dirvec2[1], f = p1_line2[1] - p1_line1[1];
                        double g = dirvec1[2], h = -dirvec2[2], I = p1_line2[2] - p1_line1[2];

    
                        double denominator = a * (e * I - f * h) - b * (d * I - f * g) + c * (d * h - e * g);


                        double t = (c * (e * I - f * h) - b * (f * I - c * h) + a * (f * h - e * I)) / denominator;
                        double s = (a * (f * I - c * h) - c * (d * I - f * g) + b * (d * h - e * g)) / denominator;

    
                        vector<double> vertex = {p1_line1[0] + t * dirvec1[0], p1_line1[1] + t * dirvec1[1], p1_line1[2] + t * dirvec1[2]};*/
                        //if(vertex[2] > 1800.0 && vertex[2] < 1850.0)
                        //std::cout << "Check this event: " << i << std::endl;
                        
                        Double_t radius1 = track1.GetGeoRadius();
                        Double_t radius2 = track2.GetGeoRadius(); 
                        radius->Fill(radius1);
                        radius->Fill(radius2);
                        Double_t B_f = 2.00;

                        double brotrack1 = 2.00 * radius1 / TMath::Sin(theta1) / 1000.0;
                        double brotrack2 = 2.00 * radius2 / TMath::Sin(theta2) / 1000.0;
                       
                        bro_vs_angle->Fill(theta1 * TMath::RadToDeg(), brotrack1);
                        bro_vs_angle->Fill(theta2 * TMath::RadToDeg(), brotrack2);
                        double enertrack1 = 0;
                        double enertrack2 = 0;
                        Double_t Am = 4.0;

                        GetEnergy(Am, 2.0, brotrack1, enertrack1);
                        GetEnergy(Am, 2.0, brotrack2, enertrack2);
                        Double_t p_ej1 = brotrack1 * 2.0 * 2.99792458 / 10 * 1000;
                        Double_t p_ej2 = brotrack2 * 2.0 * 2.99792458 / 10 * 1000;
                        Double_t E_ej1 = TMath::Sqrt(p_ej1 * p_ej1 + m_a * m_a) - m_a;
                        Double_t E_ej2 = TMath::Sqrt(p_ej2 * p_ej2 + m_a * m_a) - m_a;
                           //std::cout << "Check " << std::endl;
                         if(theta2 >= theta1){
                           
                        bro_vs_alphaenergy->Fill(E_ej1, brotrack1);
                        radius_vs_alphaenergy->Fill(E_ej1, radius1);
                        bro_vs_radius->Fill(radius1, brotrack1);
                        if(i == 9){
                        std::cout << "Event: " << i << " Ener: " << E_ej1 << " Bro: " << brotrack1 << " Radius: " << radius1 << "Angle: " << (theta1 * TMath::RadToDeg()) << std::endl;
                        }
                        //angle_vs_alphaenergy->Fill(E_ej1, theta1 * TMath::RadToDeg());
                         }
                        if(theta2 < theta1){
                        if(i == 9) {                       
                        std::cout << "Event: " << i << " Ener: " << E_ej2 << " Bro: " << brotrack2 << " Radius: " << radius2 << "Angle: " << (theta2 * TMath::RadToDeg()) << std::endl;
                        }
                        bro_vs_alphaenergy->Fill(E_ej2, brotrack2);
                        radius_vs_alphaenergy->Fill(E_ej2, radius2);
                        bro_vs_radius->Fill(radius2, brotrack2);
                        //angle_vs_alphaenergy->Fill(E_ej2, theta2 * TMath::RadToDeg());
                        }
                        angle_vs_energy_lr->Fill(theta1 * TMath::RadToDeg(), enertrack1 * Am);
                        angle_vs_energy_lr->Fill(theta2 * TMath::RadToDeg(), enertrack2 * Am);
                        //angle_vs_energy_lr->Fill(theta1 * TMath::RadToDeg(), E_ej1);
                        //angle_vs_energy_lr->Fill(theta2 * TMath::RadToDeg(), E_ej2);

                        //if(cutg_line1->IsInside(theta1 * TMath::RadToDeg(), enertrack1 * Am) || cutg_line1->IsInside(theta2 * TMath::RadToDeg(), enertrack2 * Am)){
                        //if(cutg_lowener->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg()))
                              vertex_distribution->Fill(finalz);

                        //std::cout<< "Event in line 1 cut: " << i << ", file: " << iFile << std::endl;
                        //charge_line1->Fill(event->GetEventCharge());
                        //std::cout << "Event charge: " << event->GetEventCharge() << std::endl;
                        //vertex_distribution_1->Fill(finalz);      
                        //}
                        //if(cutg_line2->IsInside(theta1 * TMath::RadToDeg(), enertrack1 * Am) || cutg_line2->IsInside(theta2 * TMath::RadToDeg(), enertrack2 * Am)){
                        //std::cout<< "Event in line 2 cut: " << i << ", file: " << iFile << std::endl;
                        //vertex_distribution_2->Fill(finalz);
                        //charge_line2->Fill(event->GetEventCharge());
                        
                        //}
                        //double_t covered_range = max_range - vertex[2];     
                        double_t covered_range = 1000 - finalz;

                        //std::cout << "Vertex: " << vertex[2] << " Covered range: " << covered_range << std::endl;
                        
                        if(cutg_lowener->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg())){           
                        Double_t rad1 = track1.GetGeoRadius();
                        Double_t rad2 = track2.GetGeoRadius();

                        Double_t B_f = 2.0;

                        double bro1 = B_f * rad1 / TMath::Sin(theta1) / 1000.0;
                        double bro2 = B_f * rad2 / TMath::Sin(theta2) / 1000.0;
                        double ener1 = 0;
                        double ener2 = 0;
                        Double_t Am = 4.0;

                        GetEnergy(Am, 2.0, bro1, ener1);
                        angle_vs_energy_lr_gscut->Fill(theta1 * TMath::RadToDeg(), ener1 * Am);
                        GetEnergy(Am, 2.0, bro2, ener2);
                        angle_vs_energy_lr_gscut->Fill(theta2 * TMath::RadToDeg(), ener2 * Am);


                        if(covered_range > 0){
                        double_t Ebeam_tb = energy_vs_distance_gr->Eval(covered_range);
                        Ebeam_tb_all.push_back(Ebeam_tb);
                      
                        vertex_energy_tb->Fill(Ebeam_tb);
                        vertexener_values.push_back(Ebeam_tb);

                     
                     
                        if(theta1 < theta2){
                        eneralpha_vs_vertexener->Fill(Ebeam_tb, ener2 * Am);
                        eneralpha_values_tb.push_back(ener2);

                        ecm_anglecm_vertex->Fill((2*(TMath::Pi()/2 - theta2)) * TMath::RadToDeg(), Ebeam_tb);

                        if(cutg_line1->IsInside(theta2 * TMath::RadToDeg(), enertrack2 * Am))
                        ecm_anglecm_vertex_l1->Fill((2*(TMath::Pi()/2 - theta2)) * TMath::RadToDeg(), Ebeam_tb);

                        if(cutg_line2->IsInside(theta2 * TMath::RadToDeg(), enertrack2 * Am))
                        ecm_anglecm_vertex_l2->Fill((2*(TMath::Pi()/2 - theta2)) * TMath::RadToDeg(), Ebeam_tb);
                        }

                        if(theta1 >= theta2){
                        eneralpha_vs_vertexener->Fill(Ebeam_tb, ener1 * Am);
                        eneralpha_values_tb.push_back(ener1);

                        ecm_anglecm_vertex->Fill((2*(TMath::Pi()/2 - theta1)) * TMath::RadToDeg(), Ebeam_tb);

                        if(cutg_line1->IsInside(theta1 * TMath::RadToDeg(), enertrack1 * Am))
                        ecm_anglecm_vertex_l1->Fill((2*(TMath::Pi()/2 - theta1)) * TMath::RadToDeg(), Ebeam_tb);

                        if(cutg_line2->IsInside(theta1 * TMath::RadToDeg(), enertrack1 * Am))
                        ecm_anglecm_vertex_l2->Fill((2*(TMath::Pi()/2 - theta1)) * TMath::RadToDeg(), Ebeam_tb);
                        }   
                        }
                        }

                        if(cutg_lowener->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg())){           
                        Double_t rad1 = track1.GetGeoRadius();
                        Double_t rad2 = track2.GetGeoRadius();

                        Double_t B_f = 2.0;

                        double bro1 = B_f * rad1 / TMath::Sin(theta1) / 1000.0;
                        double bro2 = B_f * rad2 / TMath::Sin(theta2) / 1000.0;
                        double ener1 = 0;
                        double ener2 = 0;
                        Double_t Am = 4.0;

                        GetEnergy(Am, 2.0, bro1, ener1);
                        angle_vs_energy_lr_1stcut->Fill(theta1 * TMath::RadToDeg(), ener1 * Am);
                        GetEnergy(Am, 2.0, bro2, ener2);
                        angle_vs_energy_lr_1stcut->Fill(theta2 * TMath::RadToDeg(), ener2 * Am);
                        }
                        
                     }

                     
                     auto alphatrack = track1;
                     auto anglealpha = theta1;
                     auto radalpha = track1.GetGeoRadius();
                     double Ebeam;
                     if(hitArray1.size() > 20 && hitArray2.size() > 20){
                        
                     if(theta1 < theta2){

                     alphatrack = track2;
                     anglealpha = theta2;
                     //auto anglealpha = 10. * TMath::Pi() / 180.;
                     //auto anglealpha = 20. * TMath::DegToRad();
                     //auto anglealpha = 50. * TMath::Pi() / 180.;
                     //auto anglealpha = 70. * TMath::Pi() / 180.;
                     radalpha = track2.GetGeoRadius();
                     }


                     

                     if (anglealpha * TMath::RadToDeg() > 90.0)
                        anglealpha = TMath::Pi() - anglealpha;

                     double broalpha = 2.0 * radalpha / TMath::Sin(anglealpha) / 1000.0;
                     double eneralpha = 0; //14.35, 13.06, 6.1, 1.73

                     GetEnergy(4.0, 2.0, broalpha, eneralpha);
                     double a = (m_a*(eneralpha*4.0 + m_a) - m_a*m_a)/(1 - TMath::Cos(2*(TMath::Pi()/2 - anglealpha)));
                                     
                        Ebeam = ((TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a))*(TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a)) - (m_a + m_Be10)*(m_a + m_Be10))/(2*m_a);
                       // std::cout << "Ebeam: " << Ebeam << std::endl;

                        
                        ecm_anglecm->Fill((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg(), Ebeam * 2.0/7.0);

                        if((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()>115. && (2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()<125.)
                           ex_ener_120->Fill(Ebeam * 2.0/7.0);

                        if((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()>105. && (2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()<115.)
                           ex_ener_110->Fill(Ebeam * 2.0/7.0);

                        if((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()>95. && (2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()<105.)
                           ex_ener_100->Fill(Ebeam * 2.0/7.0);

                        if((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()>85. && (2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()<95.)
                           ex_ener_90->Fill(Ebeam * 2.0/7.0);

                         if((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()>75. && (2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()<85.)
                           ex_ener_80->Fill(Ebeam * 2.0/7.0);

                        if((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()>65. && (2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()<75.)
                           ex_ener_70->Fill(Ebeam * 2.0/7.0);

                        if((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()>55. && (2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg()<65.)
                           ex_ener_60->Fill(Ebeam * 2.0/7.0);   

                       if(cutg_lowener->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg())){
                       vertex_energy->Fill(Ebeam);
                       Ebeam_a_all.push_back(Ebeam);
                       //ecm_anglecm->Fill((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg(), Ebeam * 2.0/7.0);                      
                       if(cutg_line1->IsInside(anglealpha * TMath::RadToDeg(), eneralpha * 4.0))
                        ecm_anglecm_l1->Fill((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg(), Ebeam * 2.0/7.0);

                         if(cutg_line2->IsInside(anglealpha * TMath::RadToDeg(), eneralpha * 4.0))
                        ecm_anglecm_l2->Fill((2*(TMath::Pi()/2 - anglealpha)) * TMath::RadToDeg(), Ebeam * 2.0/7.0);
                        if(Ebeam > 1.6 && Ebeam < 2.0){
                           firstpeak->Fill(anglealpha * TMath::RadToDeg(), eneralpha*0.25);
                        }

                        if(Ebeam > 2.1 && Ebeam < 2.5){
                           secondpeak->Fill(anglealpha * TMath::RadToDeg(), eneralpha*0.25);
                        }

                        bro_values.push_back(broalpha);
                        eneralpha_values.push_back(anglealpha);
    
                       }
                      if(cutg_lowener->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg())){
                     for(int i=0; i<Ebeam_tb_all.size(); i++){
                     vertex_vs_vertex->Fill(Ebeam_tb_all[i],Ebeam_a_all[i]);
                     }
                      }

                     }
                     
                  }
                  
            for (auto index = 0; index < patternTrackCand.size(); ++index) {
               bool skiptrack = false;
               if (index != maxAIndex)
                  continue;

                  
               
               auto track = patternTrackCand.at(index);
               auto hitArray = track.GetHitArrayObject();
               
               for (const auto &hit : hitArray) {
                      if (hit.GetTimeStamp() < 60) {
                      skiptrack = true;
                      break;
                      }
               }

               if (skiptrack)
                  continue;
      
               Double_t theta = track.GetGeoTheta();
               Double_t rad = track.GetGeoRadius();

               Double_t B_f = 2.0;

               double bro = B_f * rad / TMath::Sin(theta) / 1000.0;
               double ener = 0;
               Double_t Am = 4.0;

               GetEnergy(Am, 2.0, bro, ener);
               //angle_vs_energy_lr->Fill(theta * TMath::RadToDeg(), ener * Am);
               //std::cout << "Filled!"<<std::endl;
               // if (track.GetHitArray()->size() > 80)
               // angle_vs_energy->Fill(theta * TMath::RadToDeg(), ener * Am);

               // std::cout << " Brho : " << bro << " - Angle : " << theta * TMath::RadToDeg() << " - Radius : " << rad
               //          << " - Energy :" << ener * Am << "\n";

               /*std::vector<AtHit> *hitArray = track.GetHitArray();
                     for (auto hit : *hitArray) {
                        TVector3 pos = hit.GetPosition();
                        int TB = hit.GetTimeStamp();
                        std::cout << " Pos : " << pos.X() << "   " << pos.Y() << "       " << pos.Z() << " " << TB <<
                  "\n";
                   }*/

               // PID
               Double_t len = 0;
               Double_t eloss = 0;
               Double_t dedx = 0;
               
               // Energy loss from ADC
               auto hitClusterArray = track.GetHitClusterArray();
               //std::cout << "Number of Hits in Track: " << hitClusterArray->size() << std::endl;
               auto firstCluster = hitClusterArray->back();
               //std::cout << "Check" << std::endl;
               auto zpos = firstCluster.GetPosition().Z();
               std::size_t cnt = 0;

               auto it = hitClusterArray->rbegin();
               
               while (it != hitClusterArray->rend()) {

                  if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.5)
                     break;
                  auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
                  eloss += (*it).GetCharge();
                  len += std::sqrt(dir.Mag2());
                  dedx += (*it).GetCharge();
                  // std::cout<<(*it).GetCharge()<<"\n";
                  it++;
                  ++cnt;
               }
               eloss /= cnt;
               dedx /= len;

               /*std::cout << " Brho : " << bro << " - Angle : " << theta * TMath::RadToDeg() << " - Radius : " << rad
                         << " - Energy :" << ener * Am << " - dE     :" << eloss << "\n";*/

               // Selection of events
               /*if (zpos < 500.0 || zpos > 950)
                  continue;

               if (theta * TMath::RadToDeg() < 13.0)
                  continue;*/

               if (cutg_lowener->IsInside(eloss, bro)) { // Selection of protons

                 
                  auto [ex_energy_exp, theta_cm] = kine_2b(m_Be10, m_a, m_a, m_Be10, Ebeam_buff, theta, ener);

                  HQval->Fill(ex_energy_exp);

                  // Excitation energy vs Beam energy
                  for (auto iEb = 0; iEb < 300; ++iEb) {
                     auto [Qdep, theta_cm_qdep] = kine_2b(m_Be10, m_a, m_a, m_Be10, iEb, theta, ener);
                     QvsEb->Fill(Qdep, iEb);
                  }

                  // Rough vertex
                  //QvsZpos->Fill(ex_energy_exp, zpos / 10.0);

               } // protons

               bro_vs_eloss->Fill(eloss, bro);
               bro_vs_dedx->Fill(dedx, bro);

               
            }
         }

      } // nEvents

      file->Close();

   } // Files

   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];
   std::cout << countertracks << std::endl;
   TString fileKine = "10Be_a_gs_lowener.txt";
   std::ifstream *kineStr = new std::ifstream(fileKine.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         // numKin++;

         // MomLabRec[numKin] =( pow(EnerLabRec[numKin] + M_Ener,2) - TMath::Power(M_Ener, 2))/1000.0;
         // std::cout<<" Momentum : " <<MomLabRec[numKin]<<"\n";
         // Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec = new TGraph(numKin, ThetaLabRec, EnerLabRec);
   TGraph *Kine_AngRec_AngSca = new TGraph(numKin, ThetaLabRec, ThetaLabSca);

   TString fileKine2 = "10Be_a_gs_lowener.txt";
   std::ifstream *kineStr2 = new std::ifstream(fileKine2.Data());
   numKin = 0;

   if (!kineStr2->fail()) {
      while (!kineStr2->eof()) {
         *kineStr2 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr2->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_1m1 = new TGraph(numKin, ThetaLabRec, EnerLabRec);
   TGraph *Kine_1m1_ang_ang = new TGraph(numKin, ThetaLabRec, ThetaLabSca);
   
   

   /*TCanvas *c_kn_gs = new TCanvas();
   angle_vs_energy_lr_gscut->Draw("ZCOL");
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");

   TCanvas *c_kn_1st = new TCanvas();
   angle_vs_energy_lr_1stcut->Draw("ZCOL");
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");*/

   TCanvas *c8 = new TCanvas();
   QvsEb->Draw("zcol");

   TCanvas *cQZ = new TCanvas();
   QvsZpos->Draw();

   TCanvas *angle_angle = new TCanvas();
   angle_vs_angle->Draw("colz");
   cutg_lowener->Draw("SAME");
   Kine_AngRec_AngSca->SetLineColor(kRed);
   Kine_AngRec_AngSca->Draw("SAME");
   Kine_1m1_ang_ang->SetLineColor(kBlack);
   Kine_1m1_ang_ang->Draw("SAME");

   TCanvas *c_vertex = new TCanvas();
   vertex_distribution->Draw();

    TCanvas *c_vertex_ebeam = new TCanvas();
   vertex_energy->Draw();
   vertex_energy_tb->Draw("SAME");

  TGraph *graph = new TGraph(eneralpha_values.size(), eneralpha_values.data(), bro_values.data());
    graph->SetTitle("Brho vs Energy;Energy (MeV);Brho (T*m)");
    graph->SetMarkerStyle(20);
    graph->SetMarkerColor(kBlue);

    TGraph *graph2 = new TGraph(eneralpha_values_tb.size(), vertexener_values.data(), eneralpha_values_tb.data());
    graph2->SetTitle("Alpha vs Vertex Energies;Energy (MeV);Energy (MeV)");
    graph2->SetMarkerStyle(20);
    graph2->SetMarkerColor(kBlue);

    // Draw the graph
    TCanvas *c1 = new TCanvas("c1", "alpha vs vertex", 800, 600);
    graph2->Draw("AP");
   

   TCanvas *c_vertex_ebeam_tb = new TCanvas();
   vertex_energy_tb->Draw();
  
   

   TCanvas *c_vertex_vs_vertex = new TCanvas();
   vertex_vs_vertex->Draw(); 

   TCanvas *c_eneralpha_vertex = new TCanvas();
   eneralpha_vs_vertexener->Draw();    

   TCanvas *c_vertex_vs_beamenergy = new TCanvas();
   vertex_vs_beamenergy->Draw();

   TCanvas *c_kn_el_lr = new TCanvas();
   angle_vs_energy_lr->Draw("ZCOL");
   Kine_AngRec_EnerRec->Draw("SAME");
   //Kine_1m1->Draw("SAME");

  // std::cout<<"Points inside the banana: "<<npointsinside<<std::endl;
   TCanvas *c_kn_el = new TCanvas();
   c_kn_el->Divide(3, 1);
   c_kn_el->cd(1);
   angle_vs_energy->Draw("colz");
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");
   c_kn_el->cd(2);
   bro_vs_dedx->Draw("colz");
   c_kn_el->cd(3);
   angle_vs_energy_t->Draw("colz");

   TCanvas *c_charge = new TCanvas();
   charge_line1->Draw();
   charge_line2->SetLineColor(kRed);
   charge_line2->Draw("SAME");

   TCanvas *c_fpeak = new TCanvas();
   firstpeak->Draw("colz");

   TCanvas *c_speak = new TCanvas();
   secondpeak->Draw("colz");

   TCanvas *c_ecm_angle = new TCanvas();
   ecm_anglecm->Draw("colz");

   TCanvas *c_ecm_angle_vertex = new TCanvas();
   ecm_anglecm_vertex->Draw("colz");

   TCanvas *c_ecm_angle_vertex_l1 = new TCanvas();
   ecm_anglecm_vertex_l1->Draw("colz");

   TCanvas *c_ecm_angle_vertex_l2 = new TCanvas();
   ecm_anglecm_vertex_l2->Draw("colz");

   TCanvas *c_ecm_angle_l1 = new TCanvas();
   ecm_anglecm_l1->Draw("colz");

   TCanvas *c_ecm_angle_l2 = new TCanvas();
   ecm_anglecm_l2->Draw("colz");

   TCanvas *c_ex_ener_120 = new TCanvas();
   ex_ener_120->Draw();

   TCanvas *c_ex_ener_110 = new TCanvas();
   ex_ener_110->Draw();

   TCanvas *c_ex_ener_100 = new TCanvas();
   ex_ener_100->Draw();

   TCanvas *c_ex_ener_90 = new TCanvas();
   ex_ener_90->Draw();

   TCanvas *c_ex_ener_80 = new TCanvas();
   ex_ener_80->Draw();

   TCanvas *c_ex_ener_70 = new TCanvas();
   ex_ener_70->Draw();

   TCanvas *c_ex_ener_60 = new TCanvas();
   ex_ener_60->Draw();

   TCanvas *c_timebucket = new TCanvas();
   timebucket->Draw();

   TCanvas *c_radius = new TCanvas();
   radius->Draw();

   TCanvas *c_bro_vs_angle = new TCanvas();
   bro_vs_angle->Draw("colz");

   TCanvas *c_bro_vs_alphaenergy = new TCanvas();
   bro_vs_alphaenergy->Draw("colz");
   
   TCanvas *c_radius_vs_alphaenergy = new TCanvas();
   radius_vs_alphaenergy->Draw("colz");

   TCanvas *c_bro_vs_radius = new TCanvas(); 
   bro_vs_radius->Draw("colz");

   //TCanvas *c_angle_vs_alphaenergy = new TCanvas();
   //angle_vs_alphaenergy->Draw("colz");

   TCanvas *c_PID_eloss = new TCanvas();
   TCanvas *c_PID_dedx = new TCanvas();
   c_PID_eloss->cd();
   bro_vs_eloss->Draw("colz");
   c_PID_dedx->cd();
   bro_vs_dedx->Draw("colz");

   angle_vs_energy->GetXaxis()->SetTitle("#theta (deg)");
   angle_vs_energy->GetYaxis()->SetTitle("E (MeV)");
   angle_vs_energy->SetTitle("gate on p");

   angle_vs_energy_t->GetXaxis()->SetTitle("#theta (deg)");
   angle_vs_energy_t->GetYaxis()->SetTitle("E (MeV)");
   angle_vs_energy_t->SetTitle("gate on d");

   bro_vs_dedx->GetXaxis()->SetTitle("Brho ");
   bro_vs_dedx->GetYaxis()->SetTitle("dE/dx (au)");
   bro_vs_dedx->SetTitle("PID (p/d)");

   angle_vs_energy->SetStats(0);
   angle_vs_energy_t->SetStats(0);
   bro_vs_dedx->SetStats(0);

  // TCanvas *c_ExEner = new TCanvas();
  // HQval->Draw();
   std::ofstream outFile("ex_ener_90_data.txt");
   std::ofstream outFile2("ex_ener_80_data.txt");
   std::ofstream outFile3("ex_ener_70_data.txt");
   std::ofstream outFile4("ex_ener_60_data.txt");

   for (int bin = 1; bin <= ex_ener_90->GetNbinsX(); ++bin) {
        double binContent = ex_ener_90->GetBinContent(bin);
        double binCenter = ex_ener_90->GetBinCenter(bin);
        double binError = ex_ener_90->GetBinError(bin);
        outFile << binCenter << " " << 90 << " " << binContent << " " << binError << std::endl;
   }

   for (int bin = 1; bin <= ex_ener_80->GetNbinsX(); ++bin) {
        double binContent = ex_ener_80->GetBinContent(bin);
        double binCenter = ex_ener_80->GetBinCenter(bin);
        double binError = ex_ener_80->GetBinError(bin);
        outFile2 << binCenter << " " << 80 << " " << binContent << " " << binError << std::endl;
   }

   for (int bin = 1; bin <= ex_ener_70->GetNbinsX(); ++bin) {
        double binContent = ex_ener_70->GetBinContent(bin);
        double binCenter = ex_ener_70->GetBinCenter(bin);
        double binError = ex_ener_70->GetBinError(bin);
        outFile3 << binCenter << " " << 70 << " " << binContent << " " << binError << std::endl;
   }

   for (int bin = 1; bin <= ex_ener_60->GetNbinsX(); ++bin) {
        double binContent = ex_ener_60->GetBinContent(bin);
        double binCenter = ex_ener_60->GetBinCenter(bin);
        double binError = ex_ener_60->GetBinError(bin);
        outFile4 << binCenter << " " << 60 << " " << binContent << " " << binError << std::endl;
   }

   outFile.close();
   outFile2.close();
   outFile3.close();
   outFile4.close();

   TFile *outputFile = new TFile("low_energy_data_sim.root", "RECREATE");
    bro_vs_alphaenergy->Write();
    bro_vs_angle->Write();
    bro_vs_radius->Write();
    radius_vs_alphaenergy->Write();
    //angle_vs_alphaenergy->Write();
    radius->Write();
    timebucket->Write();
    angle_vs_angle->Write();
    angle_vs_energy_lr->Write();
    vertex_distribution->Write();
    vertex_energy->Write();
    vertex_energy_tb->Write();
    vertex_vs_vertex->Write();
    eneralpha_vs_vertexener->Write();
    vertex_vs_beamenergy->Write();
    ex_ener_100->Write();
    ex_ener_90->Write();
    ex_ener_80->Write();
    ex_ener_70->Write();
    ex_ener_60->Write();
    ex_ener_120->Write();   
    ex_ener_110->Write();
    angle_vs_energy_lr_gscut->Write();
    angle_vs_energy_lr_1stcut->Write();
    QvsEb->Write();
    ecm_anglecm->Write();

   outputFile->Close();


}

void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E)
{

   // Energy per nucleon
   Float_t AM = 931.5;
   Float_t X = BRO / 0.1439 * IZ / M;
   X = pow(X, 2);
   X = 2. * AM * X;
   X = X + pow(AM, 2);
   E = TMath::Sqrt(X) - AM;
}
