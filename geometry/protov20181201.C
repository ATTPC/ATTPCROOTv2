#include <TArc.h>
#include <TCanvas.h>
#include <TChain.h>
#include <TClonesArray.h>
#include <TCutG.h>
#include <TF1.h>
#include <TF2.h>
#include <TF3.h>
#include <TFile.h>
#include <TFitter.h>
#include <TGraph.h>
#include <TGraph2D.h>
#include <TH1.h>
#include <TH1F.h>
#include <TH2.h>
#include <TH2F.h>
#include <TH2Poly.h>
#include <TH3.h>
#include <TH3F.h>
#include <TLine.h>
#include <TMarker3DBox.h>
#include <TMath.h>
#include <TMinuit.h>
#include <TPolyLine3D.h>
#include <TPolyMarker.h>
#include <TROOT.h>
#include <TRandom2.h>
#include <TRint.h>
#include <TString.h>
#include <TStyle.h>
#include <TTree.h>
#include <TVirtualFitter.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

void protov20181201()
{
   TGraph *g[2016];
   TH2Poly *h2pol = new TH2Poly();
   h2pol->SetName("ATTPC_Proto");
   h2pol->SetTitle("ATTPC_Proto");

   double pad[2016][3][2]; // pads [pindex] [point in the triangle] [x:0 or y:1]
   const double a = 6.700; // side of square in mm (includes the dead layer in between pads of 0.1 mm)

   int xN = 0;
   int pindex = 0; // initialize the pad index

   // A) Define the upper left corner of pads
   // -> First define the squared area (first 13 rows * 13 columns from the center)
   for (int y = 0; y < 13; y++) {
      for (int x = 0; x < 13; x++) {
         // First triangle in the square
         pad[pindex][0][0] = a * x;
         pad[pindex][0][1] = a * y;
         pad[pindex][1][0] = a * (x + 1);
         pad[pindex][1][1] = a * y;
         pad[pindex][2][0] = a * x;
         pad[pindex][2][1] = a * (y + 1);

         // Second triangle in the square
         pad[pindex + 1][0][0] = a * (x + 1);
         pad[pindex + 1][0][1] = a * y;
         pad[pindex + 1][1][0] = a * x;
         pad[pindex + 1][1][1] = a * (y + 1);
         pad[pindex + 1][2][0] = a * (x + 1);
         pad[pindex + 1][2][1] = a * (y + 1);

         pindex += 2;
      }

      // cout << "pindex = " << pindex << endl;
   }

   // -> Then loop for the edges (5 upper rows & columns)
   for (int yy = 0; yy < 5; yy++) {

      // define number of squares for the upper rows/columns
      if (yy == 0)
         xN = 11;
      else if (yy == 1)
         xN = 10;
      else if (yy == 2)
         xN = 9;
      else if (yy == 3)
         xN = 6;
      else if (yy == 4)
         xN = 3;

      for (int xx = 0; xx < xN; xx++) {
         // First triangle (upper rows)
         pad[pindex][0][0] = a * xx;
         pad[pindex][0][1] = a * (yy + 13);
         pad[pindex][1][0] = a * (xx + 1);
         pad[pindex][1][1] = a * (yy + 13);
         pad[pindex][2][0] = a * xx;
         pad[pindex][2][1] = a * ((yy + 13) + 1);

         // Second triangle (upper rows)
         pad[pindex + 1][0][0] = a * (xx + 1);
         pad[pindex + 1][0][1] = a * (yy + 13);
         pad[pindex + 1][1][0] = a * xx;
         pad[pindex + 1][1][1] = a * ((yy + 13) + 1);
         pad[pindex + 1][2][0] = a * (xx + 1);
         pad[pindex + 1][2][1] = a * ((yy + 13) + 1);

         // Triangles in the upper columns by symmetry
         for (int ii = 0; ii < 3; ii++) {
            pad[pindex + 2][ii][0] = pad[pindex][ii][1];
            pad[pindex + 2][ii][1] = pad[pindex][ii][0];

            pad[pindex + 3][ii][0] = pad[pindex + 1][ii][1];
            pad[pindex + 3][ii][1] = pad[pindex + 1][ii][0];
         }

         pindex += 4;
      }

      // Single triangle in the row end
      pad[pindex][0][0] = a * xN;
      pad[pindex][0][1] = a * (yy + 13);
      pad[pindex][1][0] = a * (xN + 1);
      pad[pindex][1][1] = a * (yy + 13);
      pad[pindex][2][0] = a * xN;
      pad[pindex][2][1] = a * ((yy + 13) + 1);

      for (int k = 0; k < 3; k++) {
         pad[pindex + 1][k][0] = pad[pindex][k][1];
         pad[pindex + 1][k][1] = pad[pindex][k][0];
      }
      pindex += 2;
   }

   // -> Use symmetry considerations for the upper right columns
   for (int j = 0; j < 504; j++) {
      for (int k = 0; k < 3; k++) {
         // upper left corner
         pad[pindex][k][0] = -pad[j][k][0];
         pad[pindex][k][1] = pad[j][k][1];

         // lower right corner
         pad[pindex + 1][k][0] = pad[j][k][0];
         pad[pindex + 1][k][1] = -pad[j][k][1];

         // lower left corner
         pad[pindex + 2][k][0] = -pad[j][k][0];
         pad[pindex + 2][k][1] = -pad[j][k][1];
      }
      pindex += 3;
   }

   // B) now that we mapped a fourth of the detector, just use symmetry to map the other 3 corners

   // C) Now we can fill the TH2Poly
   for (int N = 0; N < 2016; N++) {
      g[N] = new TGraph(3);
      g[N]->SetPoint(0, pad[N][0][0], pad[N][0][1]);
      g[N]->SetPoint(1, pad[N][1][0], pad[N][1][1]);
      g[N]->SetPoint(2, pad[N][2][0], pad[N][2][1]);
      h2pol->AddBin(g[N]);
   }

   // Adding vias
   Float_t x, y;
   Float_t xA[2015], yA[2015];
   TTree *viatree = new TTree("viastree", "Position of the vias");
   viatree->ReadFile("../resources/ViaLoc.csv", "x/F:y/F", ',');
   viatree->SetBranchAddress("x", &x);
   viatree->SetBranchAddress("y", &y);

   std::ofstream map;
   map.open("proto20181201.map");

   for (auto i = 0; i < viatree->GetEntries(); ++i) {
      viatree->GetEntry(i);
      // std::cout<<x<<" "<<y<<"\n";
      int bin = h2pol->Fill(x, y, 1.0);
      std::string bin_name = "pad_" + std::to_string(i);
      map << i << "   " << bin_name << "    " << x << " " << y << " " << bin << "\n";
      xA[i] = x;
      yA[i] = y;
   }

   map.close();

   TGraph *sec_center;
   sec_center = new TGraph(2015, xA, yA);
   sec_center->SetMarkerSize(0.4);
   sec_center->SetMarkerStyle(20);
   sec_center->SetMarkerColor(kRed);

   // TH2Poly file
   TFile *file = new TFile("proto20181201_geo_hires.root", "RECREATE");
   h2pol->Reset(0);
   h2pol->Write();
   file->Close();

   TCanvas *c1 = new TCanvas("c1", "Pad Plane", 600, 600);
   c1->cd();
   h2pol->Draw("colz L");
   sec_center->Draw("P");
}
