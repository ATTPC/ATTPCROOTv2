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

void C14_pp_ana_IC_test()
{
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
    double_t ener_0 = 18.1; //MeV
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

   energy_vs_distance_gr->Draw("AP");
   double_t tb_entrance = 290.0;
   double_t max_range = 1000 - 651.01; //beam max range in mm with this conditions (pressure, energy...)
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 500, 0, 3);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 500, 0, 3);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 1000, 0, 90., 1000, 0, 20.0);
   TH2F *angle_vs_energy_t = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);
   TH2F *angle_vs_angle = new TH2F("angle_vs_angle", "angle_vs_angle", 720, 0, 179, 720, 0, 179);
   TH1F *vertex_distribution = new TH1F("vertex_distribution", "Vertex", 200, -1000, 2000);
   TH1F *vertex_energy = new TH1F("vertex_energy", "Vertex Energy", 400, -20., 20.);
   TH1F *vertex_energy_tb = new TH1F("vertex_energy_tb", "Vertex Energy using TB", 600, -20, 40);
   TH2F *vertex_vs_beamenergy = new TH2F("vertex_vs_beamenergy", "Vertex and beam energy",1000, 0., 1000, 600, -20., 40.);
   TH2F *vertex_vs_vertex = new TH2F("vertex_vs_vertex", "Vertex TB vs Vertex alpha",600, -20., 40., 600, -20., 40.);


   TH1F *HQval = new TH1F("HQval", "HQval", 600, -5, 55);
   TH2F *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);

   TCutG *cutg = new TCutG("CUTG",14);
   cutg->SetVarX("angle_vs_angle");
   cutg->SetVarY("");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetPoint(0,39.625,23.4881);
   cutg->SetPoint(1,44.3098,24.2644);
   cutg->SetPoint(2,49.0539,23.0445);
   cutg->SetPoint(3,56.5258,20.383);
   cutg->SetPoint(4,65.5989,17.0561);
   cutg->SetPoint(5,71.7069,13.3964);
   cutg->SetPoint(6,70.7581,11.4003);
   cutg->SetPoint(7,66.5477,10.2913);
   cutg->SetPoint(8,59.0165,13.2855);
   cutg->SetPoint(9,51.4259,17.6105);
   cutg->SetPoint(10,45.3179,19.3849);
   cutg->SetPoint(11,39.6843,23.1554);
   cutg->SetPoint(12,39.8622,24.0426);
   cutg->SetPoint(13,39.625,23.4881);


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

   m_b = m_a;
   m_B = m_Be10;
   AtTools::AtTrackTransformer transformer;
   //TString FileName = "run_0062.root";
   // std::cout << " Opening File : " << FileName.Data() << std::endl;
   // TFile *file = new TFile(FileName.Data(), "READ");

   TString dir = "/media/david/EXTERNAL_USB/e22502/low_energy/";

   std::vector<TString> files{"run_0100.root"};
   for (auto iFile : files) {

// GET Data
      TString filePath = dir + iFile;
      TFile *file = new TFile(filePath.Data(), "READ");
      TTree *tree = (TTree *)file->Get("cbmsim;85");
      Int_t nEvents = tree->GetEntries();
      //std::cout << " Processing file : " << iFile.first.Data() << "\n";
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader ReaderTracking("cbmsim;85", file);
      //TTreeReaderValue<TClonesArray> trackingArray(ReaderTracking, "AtTrackingEvent");
      TTreeReaderValue<TClonesArray> eventArray(ReaderTracking, "AtEventH");
      TTreeReaderValue<TClonesArray> eventArray1(ReaderTracking, "AtPatternEvent");


      // FRIB data
      TTree *treeFRIB = (TTree *)file->Get("cbmsim");
      Int_t nEventsFRIB = treeFRIB->GetEntries();
      std::cout << " Number of FRIB DAQ events : " << nEventsFRIB << std::endl;

      TTreeReader Reader2("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray2(Reader2, "AtEventH");
      TTreeReaderValue<TClonesArray> eventArray3(Reader2, "AtPatternEvent");
      
   

      if (nEvents != nEventsFRIB + 1) {
         std::cerr << " Error, incompatible number of events! Exiting... "
                   << "\n";
         // std::exit(0);
      }

      ULong64_t fribTSRef = 0;
      ULong64_t getTSRef_0 = 0;
      ULong64_t getTSRef_1 = 0;

      ULong64_t fribDTS = 0;
      ULong64_t getDTS_0 = 0;
      ULong64_t getDTS_1 = 0;
      for (Int_t i = 0; i < 30000; i++) {

         // eventArray->Clear();
         if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         //Reader1.Next();
         ReaderTracking.Next();
         Reader2.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray1->At(0);
          AtEvent *event = (AtEvent *)eventArray->At(0);

         AtPatternEvent *patternEventFRIB = (AtPatternEvent *)eventArray3->At(0);
         AtEvent *eventFRIB = (AtEvent *)eventArray2->At(0);
         auto getTS_0 = event->GetTimestamp(0);
         auto getTS_1 = event->GetTimestamp(1);

         ULong64_t ts = eventFRIB->GetTimestamp(1);

         if (i == 0) {
               fribDTS = 0;
               getDTS_0 = 0;
               getDTS_1 = 0;
            } else {

               fribDTS = ts - fribTSRef;
               std::cout << " FRIB DTS : " << fribDTS << "\n";
               //std::cout << " FRIB TS : " << *ts << "\n";
               //std::cout << "Event name : " << *fribEvName << "\n";
               getDTS_0 = getTS_0 - getTSRef_0;
               getDTS_1 = getTS_1 - getTSRef_1;
               std::cout << " GET DTS 0 : " << getDTS_0 << "\n";
               std::cout << " GET DTS 1 : " << getDTS_1 << "\n";
              // std::cout << "Event name : " << event->GetEventName() << "\n";

              
            }

            getTSRef_0 = getTS_0;
            getTSRef_1 = getTS_1;
            fribTSRef = ts;


         if (patternEvent) {
            Bool_t goodBeam = false;
            //for (auto enerIC : *energyIC) {
              // if (enerIC > 600) {
               //   goodBeam = true;
                  //henergyIC->Fill(ener);
              // }
           // }
            //if (!goodBeam)
              // continue;

            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            std::vector<AtTrack> newTracks;  
            // std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

            // Find track with largets angle
            auto itMax =
               std::max_element(patternTrackCand.begin(), patternTrackCand.end(),
                                [](const auto &a, const auto &b) { return b.GetGeoTheta() > a.GetGeoTheta(); });
            Int_t maxAIndex = std::distance(patternTrackCand.begin(), itMax);

            std::sort(patternTrackCand.begin(), patternTrackCand.end(),
              [](const AtTrack &a, const AtTrack &b) {
                  return a.GetHitArray().size() > b.GetHitArray().size();
              });
            // for (auto track : patternTrackCand) {
                     if (patternTrackCand.size() > 1){
                     auto track1 = patternTrackCand.at(0);
                     auto track2 = patternTrackCand.at(1);

                     Double_t theta1 = track1.GetGeoTheta();
                     if(theta1 * TMath::RadToDeg() > 90.0) 
                        theta1 = TMath::Pi() - theta1;
                     Double_t theta2 = track2.GetGeoTheta();  
                     if(theta2 * TMath::RadToDeg() > 90.0) 
                        theta2 = TMath::Pi() - theta2;     
                     //std::cout << "Theta 1: " << theta1 << " Theta 2: " << theta2 << std::endl;
                     auto hitArray1 = track1.GetHitArrayObject();
                     auto hitArray2 = track2.GetHitArrayObject(); 

                     //auto firstPoint1 = hitArray1(0);
                     //auto firstPoint2 = hitArray2(0);
                  
                  

                     //std::cout << "First Z track 1: " << firstPoint1.GetPosition().Z() << std::endl;
                     //std::cout << "First Z track 2: " << firstPoint2.GetPosition().Z() << std::endl;

                     auto hitClusterArray1 = track1.GetHitClusterArray();
                     auto firstCluster1 = hitClusterArray1->back();
                     auto zpos1 = firstCluster1.GetPosition().Z();

                     auto hitClusterArray2 = track2.GetHitClusterArray();
                     auto firstCluster2 = hitClusterArray2->back();
                     auto zpos2 = firstCluster2.GetPosition().Z();               

                     //std::cout << "First hit in track 1: " << hitArray1[0].GetPosition().Z() << std::endl;
                     //std::cout << "First hit in track 2: " << hitArray2[0].GetPosition().Z() << std::endl;
                   

                     if(theta1 >= theta2) 
                     angle_vs_angle->Fill(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg());

                     if(theta1 < theta2)
                     angle_vs_angle->Fill(theta2 * TMath::RadToDeg(), theta1 * TMath::RadToDeg());

                     if(cutg->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg()))
                        npointsinside++;


                     vector<double> p1_line1;
                     vector<double> p2_line1;
                     vector<double> p1_line2;
                     vector<double> p2_line2;
                     if (hitClusterArray1->size() > 1 && hitClusterArray2->size() > 1) {
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
                        if(cutg->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg()))
                              //vertex_distribution->Fill(vertex[2]);

                        vertex_distribution->Fill(finalz);      

                        //double_t covered_range = max_range - vertex[2];     
                        double_t covered_range = 1000 - finalz;

                        //std::cout << "Vertex: " << vertex[2] << " Covered range: " << covered_range << std::endl;
                        
                       /* if(cutg->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg())){           
                        Double_t rad1 = track1.GetGeoRadius();
                        Double_t rad2 = track2.GetGeoRadius();

                        Double_t B_f = 2.0;

                        double bro1 = B_f * rad1 / TMath::Sin(theta1) / 1000.0;
                        double bro2 = B_f * rad2 / TMath::Sin(theta2) / 1000.0;
                        double ener1 = 0;
                        double ener2 = 0;
                        Double_t Am = 4.0;

                        GetEnergy(Am, 2.0, bro1, ener1);
                        angle_vs_energy_lr->Fill(theta1 * TMath::RadToDeg(), ener1 * Am);
                        GetEnergy(Am, 2.0, bro2, ener2);
                        angle_vs_energy_lr->Fill(theta2 * TMath::RadToDeg(), ener2 * Am);
                        }*/
                        if(covered_range > 0){
                        double_t Ebeam_tb = energy_vs_distance_gr->Eval(covered_range);
                        Ebeam_tb_all.push_back(Ebeam_tb);

                        //std::cout << "Ebeam: " << Ebeam_tb << std::endl;
                        if(cutg->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg())){
                        vertex_energy_tb->Fill(Ebeam_tb);
                        //vertex_vs_beamenergy->Fill(vertex[2],Ebeam_tb);
                        vertex_vs_beamenergy->Fill(finalz,Ebeam_tb);
                        
                        }
                        }
                     }

                     auto alphatrack = track1;
                     auto anglealpha = theta1;
                     auto radalpha = track1.GetGeoRadius();
                     double Ebeam;

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
                        anglealpha = TMath::Pi()  - anglealpha;

                     double broalpha = 2.0 * radalpha / TMath::Sin(anglealpha) / 1000.0;
                     double eneralpha = 0; //14.35, 13.06, 6.1, 1.73

                     GetEnergy(4.0, 2.0, broalpha, eneralpha);
                     double a = (m_a*(eneralpha + m_a) - m_a*m_a)/(1 - TMath::Cos(2*(TMath::Pi()/2 - anglealpha)));
                                     
                        Ebeam = ((TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a))*(TMath::Sqrt(a + m_Be10*m_Be10) + TMath::Sqrt(a + m_a*m_a)) - (m_a + m_Be10)*(m_a + m_Be10))/(2*m_a);
                       // std::cout << "Ebeam: " << Ebeam << std::endl;
                       if(cutg->IsInside(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg())){
                       vertex_energy->Fill(Ebeam);
                       Ebeam_a_all.push_back(Ebeam);
                       }
                  }
            for (auto index = 0; index < patternTrackCand.size(); ++index) {

               //if (index != maxAIndex)
                 // continue;
                  




               auto track = patternTrackCand.at(index);

               Double_t theta = track.GetGeoTheta();
               Double_t rad = track.GetGeoRadius();

               Double_t B_f = 2.0;

               double bro = B_f * rad / TMath::Sin(theta) / 1000.0;
               double ener = 0;
               Double_t Am = 4.0;

               GetEnergy(Am, 2.0, bro, ener);
               angle_vs_energy_lr->Fill(theta * TMath::RadToDeg(), ener * Am);
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
               auto firstCluster = hitClusterArray->back();
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
               if (zpos < 500.0 || zpos > 950)
                  continue;

               if (theta * TMath::RadToDeg() < 13.0)
                  continue;

               if (cutg->IsInside(eloss, bro)) { // Selection of protons

                 
                  auto [ex_energy_exp, theta_cm] = kine_2b(m_Be10, m_a, m_a, m_Be10, Ebeam_buff, theta, ener);

                  HQval->Fill(ex_energy_exp);

                  // Excitation energy vs Beam energy
                  for (auto iEb = 0; iEb < 300; ++iEb) {
                     auto [Qdep, theta_cm_qdep] = kine_2b(m_Be10, m_a, m_a, m_Be10, iEb, theta, ener);
                     QvsEb->Fill(Qdep, iEb);
                  }

                  // Rough vertex
                  QvsZpos->Fill(ex_energy_exp, zpos / 10.0);

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

   TString fileKine = "10Be_a_gs.txt";
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

   TString fileKine2 = "10Be_a_1s.txt";
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
   
   TCanvas *c_kn_el_lr = new TCanvas();
   angle_vs_energy_lr->Draw("ZCOL");
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");

   TCanvas *c8 = new TCanvas();
   QvsEb->Draw("zcol");

   TCanvas *cQZ = new TCanvas();
   QvsZpos->Draw();

   TCanvas *angle_angle = new TCanvas();
   angle_vs_angle->Draw("colz");
   Kine_AngRec_AngSca->SetLineColor(kRed);
   Kine_AngRec_AngSca->Draw("SAME");
   Kine_1m1_ang_ang->SetLineColor(kBlack);
   Kine_1m1_ang_ang->Draw("SAME");

   TCanvas *c_vertex = new TCanvas();
   vertex_distribution->Draw();

    TCanvas *c_vertex_ebeam = new TCanvas();
   vertex_energy->Draw();
   vertex_energy_tb->Draw("SAME");

   TCanvas *c_vertex_ebeam_tb = new TCanvas();
   vertex_energy_tb->Draw();

   for(int i=0; i<Ebeam_tb_all.size(); i++){
      vertex_vs_vertex->Fill(Ebeam_tb_all[i],Ebeam_a_all[i]);
   }

   TCanvas *c_vertex_vs_vertex = new TCanvas();
   vertex_vs_vertex->Draw();   

   TCanvas *c_vertex_vs_beamenergy = new TCanvas();
   vertex_vs_beamenergy->Draw();

   std::cout<<"Points inside the banana: "<<npointsinside<<std::endl;
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

   TCanvas *c_PID_eloss = new TCanvas();
   TCanvas *c_PID_dedx = new TCanvas();
   c_PID_eloss->cd();
   bro_vs_eloss->Draw("colz");
   cutg->Draw("l");
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

   TCanvas *c_ExEner = new TCanvas();
   HQval->Draw();
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
