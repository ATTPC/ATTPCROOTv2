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


#include "spline.h"  //funcion externa que me hace interpolacion cubica entre dos puntos


#define pi 3.1415926535
#define re2  7.84e-26 //cm2
#define me  0.511 //MeV
#define mp  52161 //MeV
#define nelc  4.8e19 //electrons/cm3
#define lumi  4.9e26

using namespace std;





double sq(double val ){
	
	return val*val;
}




double GetRange(double Edelta,  std::vector<Double_t> X, std::vector<Double_t> Y){
	
		 tk::spline s;
   		 s.set_points(X,Y);
	
	return s(Edelta);
}


double GetEloss(double Edelta,  std::vector<Double_t> X, std::vector<Double_t> W){
	
		 tk::spline s1;
   		 s1.set_points(X,W);
	
	return s1(Edelta);
}




void delta_elec_padplane()
{

  TStopwatch timer;
  timer.Start();

  AtTpcMap* fAtMapPtr = new AtTpcMap();
  fAtMapPtr->GenerateATTPC();
  TH2Poly *fPadPlane = fAtMapPtr->GetATTPCPlane();
 //TH2Poly *fPadPlane2 = fAtMapPtr->GetATTPCPlane();

            //gStyle->SetOptStat(0);
            gStyle->SetPalette(103);


double ztar = 1.;
double zp = 28.;
double kin = 5600; // in MeV
double gamma = (1.+kin/mp);
double beta = sqrt(1.- 1./sq(gamma) );
double tmax = 2*me*(sq(gamma)-1)/(1+ 2*gamma*(me/mp) + sq(me/mp)); // in MeV
double tcut = 1e-3;//   in MeV

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

	vector<double> X(94), Y(94), W(94);
	string filename= "electronrange.dat";  
  	ifstream  entrada;
	double col1, col2, col3;
	entrada. open(filename.c_str());
      	if(entrada.fail() ){
                       cerr << "error abriendo "<< filename << endl;
 			exit(1);
                      }  

         for(int k=0;k<94;k++){
        	entrada >> col1 >> col2 >> col3;  //defino el numero de columnas   				
		X[k]=col1; Y[k]=col3;  W[k]=col2; 
		}
	entrada.close();




            for(Int_t i=0;i<100000;i++)
            {

                  
                  //Int_t bin=  fPadPlane->Fill(x,y,z);
                 g = gRandom->Uniform(0, 1);
                 tnew = 1./(1./tcut   -g*(1./tcut  - 1./tmax) );
                 P1 = sqrt(sq(kin) + 2*kin*mp);
                 K4 = kin - tnew;
                 P3 = sqrt(sq(tnew) + 2*tnew*me);
                 P4 = sqrt(sq(K4) + 2*K4*mp);
                 theta = acos(  (  sq(P1) + sq(P3) - sq(P4) )/(2*P1*P3)  );
                 phi = gRandom->Uniform(0, TMath::TwoPi() );

                do{
                 x0 = gRandom -> Gaus(0,10.); //in mm
                 y0 = gRandom -> Gaus(0,10.); //in mm
                 z0 = gRandom->Uniform(0, 1000 ); //in mm
                } while(sqrt(sq(x0) + sq(y0)) > 20. );

                R = GetRange(tnew,X,Y);
                eloss = GetEloss(tnew,X,W);
                zf = R/(tan(theta)) + z0;
                if(zf>L) zf = L;

                r = (zf-z0)*tan(theta);
                //xf = r *cos(phi) + x0;
                //yf = r *sin(phi)  + y0;

                x = x0;
                y = y0; 
                //cout<<tnew<<"  "<<R<<"  "<<theta*180/3.1415<<"  "<<eloss<<endl;

                for(int j=1;j<=r/dr;j++){
                        x += dr *cos(phi);
                        y += dr *sin(phi);

                        if(sqrt(x*x  +y*y)>= holeradius )
                         Int_t bin=  fPadPlane->Fill(x,y,1);
                        //Int_t bin=  fPadPlane->Fill(x,y,eloss*dr*1e3);
                        

                }

                //plot1d->Fill(theta*180/3.11415);

            }

            //fPadPlane2->Draw("scat");
            fPadPlane->Draw("colz  L0");
            fPadPlane -> SetMinimum(1.0);
            

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
