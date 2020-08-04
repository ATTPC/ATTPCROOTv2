#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"
#include "TMath.h"
#include "TRandom3.h"
#include "TVector3.h"
#include <TGraph2D.h>
#include <TStyle.h>
#include <TCanvas.h>
#include <TF2.h>
#include <TH1.h>
#include <TH2.h>
#include <TH3.h>
#include <Math/Functor.h>
#include <TPolyLine3D.h>
#include <Math/Vector3D.h>
#include <Fit/Fitter.h>
#include "TDOMParser.h"
#include "TXMLNode.h"


#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>



#define pi 3.1415926535
#define re2  7.84e-26 //cm2
#define me  0.511 //MeV
#define mp  52161 //MeV
#define nelc  4.8e19 //electrons/cm3
#define lumi  4.9e26

using namespace std;










void test_holePadPlane()
{

  TStopwatch timer;
  timer.Start();

  AtTpcMap* fAtMapPtr = new AtTpcMap();
  fAtMapPtr->GenerateATTPC();
  Bool_t MapIn = fAtMapPtr->ParseXMLMap("../../../scripts/Lookup20150611.xml");
  TH2Poly *fPadPlane = fAtMapPtr->GetATTPCPlane();
  
	//fPadPlane = fMap->GetATTPCPlane();
 //TH2Poly *fPadPlane2 = fAtMapPtr->GetATTPCPlane();

            //gStyle->SetOptStat(0);
            //gStyle->SetPalette(103);



TRandom3* gRandom = new TRandom3();
TH1D *plot1d = new TH1D("plot1d","plot1d",500,0,180);

        double g;
        double tnew;
        double P1;
        double K4;
        double P3;
        double P4;
        double theta;
        double phi;
        double x0,y0,z0;
        double xf,yf, zf;
        double L = 1000.;
        double holeradius = 0;
        double r;
        double dr = 0.1;
        double R =0;
        double eloss = 0;

  Float_t x=0;
  Float_t y=0;
  Float_t z=0;



        //---------------leemos stopping power table
	
	vector<int> X(126), Y(126), W(126);
	string filename= "coordmap_inhi.txt";  
  	ifstream  entrada;
	int col1, col2, col3;
	entrada. open(filename.c_str());
      	if(entrada.fail() ){
                       cerr << "error abriendo "<< filename << endl;
 			exit(1);
                      }  

         for(int k=0;k<126;k++){
        	entrada >> col1;  //defino el numero de columnas   				
		X[k]=col1; 
		}
	entrada.close();
	

	

            for(Int_t i=0;i<126;i++)
            {

		std::vector<Float_t> PadCenterCoord;
		PadCenterCoord = fAtMapPtr->CalcPadCenter(X[i]);	
                  cout<<X[i]<<"  "<<PadCenterCoord[0]<<endl;
              	Int_t bin=  fPadPlane->Fill(PadCenterCoord[0],PadCenterCoord[1],1);

            }


	     
            //fPadPlane2->Draw("scat");
            fPadPlane->Draw("colz  L0");
	    //fPadPlane->Draw("colz");
            //fPadPlane -> SetMinimum(1.0);
             TEllipse *el1 = new TEllipse(0.0,0.0,20,20);
	     el1->Draw("same");
	
             //plot1d->Draw();

            std::cout << std::endl << std::endl;
            std::cout << "Macro finished succesfully."  << std::endl << std::endl;
            // -----   Finish   -------------------------------------------------------
            timer.Stop();
            Double_t rtime = timer.RealTime();
            Double_t ctime = timer.CpuTime();
            cout << endl << endl;
            cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
            cout << endl;

}
