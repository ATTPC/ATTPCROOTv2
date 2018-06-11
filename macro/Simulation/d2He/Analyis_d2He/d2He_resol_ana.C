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
#include <Math/Functor.h>
#include <TPolyLine3D.h>
#include <Math/Vector3D.h>
#include <Fit/Fitter.h>
#include "TDOMParser.h"
#include "TXMLNode.h"

//#include "ATd2HeAnalysis.hh"

#include <iostream>
#include <fstream>
#include <vector>
#define PI 3.1415926435



//--------------------function for line3Dfit
using namespace ROOT::Math;
// define the parametric line equation
void line(double t, const double *p, double &x, double &y, double &z) {
   // a parametric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = p[0] + p[1]*t;
   y = p[2] + p[3]*t;
   z = t;
}
bool first = true;
// function Object to be minimized
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
      assert(fGraph != 0);
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

//----------------------------


double omega(double x, double y, double z){
	return sqrt(x*x + y*y + z*z -2*x*y -2*y*z -2*x*z);

}

double sq(double val ){
	
	return val*val;
}

double signo(double valor ){
	if(valor<0) return -1.0;
        else return 1.0;
	
}


double *kine_2b(double m1, double m2, double m3, double m4, double K_proj, double thetalab, double K_eject){

        //in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
        double Et1 = K_proj + m1;
        double Et2 = m2;
        double Et3 = K_eject + m3;
        double Et4  = Et1 + Et2 - Et3;
        double m4_ex, Ex, theta_cm;
        double s,t,u; //---Mandelstam variables
        double p1, p3;
        double J_LtoCM; //jacobian Lab to CM

        s = pow(m1,2) + pow(m2,2) +2*m2*Et1;
        u = pow(m2,2) + pow(m3,2) - 2*m2*Et3;

        m4_ex = sqrt(  (cos(thetalab) * omega(s,pow(m1,2),pow(m2,2)) * omega(u,pow(m2,2),pow(m3,2)) - (s - pow(m1,2) - pow(m2,2))*(pow(m2,2) + pow(m3,2) - u) )/(2*pow(m2,2)) + s + u - pow(m2,2)  );
        Ex = m4_ex - m4;
        
        t =   pow(m2,2) + pow(m4_ex,2) - 2*m2*Et4; 

        //for normal kinematics
        //theta_cm = acos( ( pow(s,2) +s*(2*t - pow(m1,2) - pow(m2,2) - pow(m3,2) - pow(m4_ex,2)) + (pow(m1,2) - pow(m2,2))*(pow(m3,2) - pow(m4_ex,2)) )/( omega(s,pow(m1,2),pow(m2,2))*omega(s,pow(m3,2),pow(m4_ex,2))) ) ;
        
        //for inverse kinematics Note: this angle corresponds to the recoil
        theta_cm = TMath::Pi() - acos( ( pow(s,2) +s*(2*t - pow(m1,2) - pow(m2,2) - pow(m3,2) - pow(m4_ex,2)) + (pow(m1,2) - pow(m2,2))*(pow(m3,2) - pow(m4_ex,2)) )/( omega(s,pow(m1,2),pow(m2,2))*omega(s,pow(m3,2),pow(m4_ex,2))) ) ;

        p1= sqrt(pow(Et1,2)-pow(m1,2));
        p3 = sqrt(pow(Et3,2)-pow(m3,2));

	J_LtoCM = fabs( ((omega(s,pow(m1,2),pow(m2,2))*omega(s,pow(m3,2),pow(m4,2)))/(4*s*p1*p3))*(1.+Et1/m2 - cos(thetalab)*(Et3*p1)/(m2*p3)) );


        static double output[3];
	output[0]= theta_cm;
        output[1]= Ex;
        output[2]= J_LtoCM;
  	return output;
        
}


void d2He_resol_ana()
{


	//-----------------tiempo-------------------------------------------
	Float_t Ttotal,inicio, final;
	inicio=clock();
	//------------------------------------------------------------------

    //TString mcFileNameHead = "../data/attpcsim_d2He";
    TString mcFileNameHead = "/mnt/simulations/attpcroot/data/attpcsim_d2He";
    TString mcFileNameTail = ".root";
    TString mcFileName     = mcFileNameHead + mcFileNameTail;
    std:cout << " Analysis of simulation file  " << mcFileName << endl;

    AtTpcPoint* point = new AtTpcPoint();
    AtTpcPoint* point_forw = new AtTpcPoint();
    AtTpcPoint* point_back = new AtTpcPoint();
    TClonesArray *pointArray=0;
    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");

    
    ATd2HeAnalysis*  d2he_obj = new ATd2HeAnalysis();

    tree = (TTree*) file -> Get("cbmsim");
    //TBranch *branch = tree->GetBranch("AtTpcPoint");
    tree -> SetBranchAddress("AtTpcPoint", &pointArray);
    Int_t nEvents = tree -> GetEntriesFast();


        // ***************Create ATTPC Pad Plane***************************
    TString scriptfile = "Lookup20150611.xml";
    TString dir = getenv("VMCWORKDIR");
    TString scriptdir = dir + "/scripts/"+ scriptfile;

     AtTpcMap *map = new AtTpcMap();
     TH2Poly *fPadPlane;                  //!< pad plane
     map->GenerateATTPC();
     Bool_t MapIn = map->ParseXMLMap(scriptdir);
     fPadPlane = map->GetATTPCPlane();

	//-------definimos unos histogramas
	TFile* outfile;
        outfile   = TFile::Open("hist_d2He_ana.root","recreate");
	outfile->mkdir("Parameters");
	outfile->mkdir("He2_reconstr");
        outfile->mkdir("He2_reconstr_resol");
	TH2D *tracks_z_r = new TH2D("tracks_z_r","ZvsR",500,-100,1000,500,0,300);
	TH2D *tracks_x_y = new TH2D("tracks_x_y","XvsY",500,-300,300,500,-300,300);
	TH2D *angle_r = new TH2D("theta_vs_R","theta_vs_R",500,0,300,500,0,180);
	TH2D *angle_eloss = new TH2D("theta_vs_Eloss","theta_vs_Eloss",500,0,180,500,0,200);
	TH2D *range_eloss_p1 = new TH2D("range_vs_Eloss_p1","Range_vs_Eloss_p1",100,0,500,100,0,5);
	TH2D *range_eloss_p2 = new TH2D("range_vs_Eloss_p2","Range_vs_Eloss_p2",100,0,500,100,0,5);
	TH2D *avth_eloss_p1 = new TH2D("Avth_vs_Eloss_p1","Avth_vs_Eloss_p1",100,0,180,100,0,5);
	TH2D *avth_eloss_p2 = new TH2D("Avth_vs_Eloss_p2","Avth_vs_Eloss_p2",100,0,180,100,0,5);
	TH2D *avth1_avth2 = new TH2D("avth1_avth2","avth1_avth2",100,0,180,100,0,180);
	TH2D *Eloss1_Eloss2 = new TH2D("Eloss1_Eloss2","Eloss1_Eloss2",100,0,5,100,0,5);
	TH2D *avphi1_avphi2 = new TH2D("avphi1_avphi2","avphi1_avphi2",100,0,360,100, 0,360);
	TH2D *diffth_diffphi = new TH2D("diffth_diffphi","diffth_diffphi",100,-180,180,100,-360,360);	
	TH2D *eloss_track = new TH2D("eloss_track","eloss_track",500,0,1000,500,0,50);
	TH1D *Eloss_p = new TH1D("Eloss_p","Eloss_p",500,0,4);
	
	TH1D *theta_r_he2 = new TH1D("theta_r_he2","theta 2He",500,0,180);
	TH1D *kin_r_he2 = new TH1D("kin_r_he2","Energy 2He",500,0,5);
	TH1D *phi_r_he2 = new TH1D("phi_r_he2","phi 2He",500,-180,180);
	TH2D *theta_kin_2he = new TH2D("theta_kin_2he","Kin vs Theta 2He",500,0,180,500,0,5);
	TH1D *thetacm_he2 = new TH1D("thetacm_he2","thetacm_he2",500,0,20);
	TH1D *Ex_res = new TH1D("Ex_res","Ex_res",500,-10,20);
	TH2D *thetacm_Ex_2he = new TH2D("thetacm_Ex_2he","thetacm_Ex_2he",500,0,20,500,-10,20);
	TH1D *ex_he2 = new TH1D("ex_he2","ex_he2",500,0,20);

        TH1D *theta_r_he2_res = new TH1D("theta_r_he2_res","theta 2He",500,0,180);
	TH1D *kin_r_he2_res = new TH1D("kin_r_he2_res","Energy 2He",500,0,5);
	TH1D *phi_r_he2_res = new TH1D("phi_r_he2_res","phi 2He",500,-180,180);
	TH2D *theta_kin_2he_res = new TH2D("theta_kin_2he_res","Kin vs Theta 2He",500,0,180,500,0,5);
	TH1D *thetacm_he2_res = new TH1D("thetacm_he2_res","thetacm_he2",500,0,20);
	TH1D *Ex_res_res = new TH1D("Ex_res_res","Ex_res",500,-10,20);
	TH2D *thetacm_Ex_2he_res = new TH2D("thetacm_Ex_2he_res","thetacm_Ex_2he",500,0,20,500,-10,20);
	TH2D *vertex_reco = new TH2D("vertex_reco","vertex_reco",500,-10,1000,500,-10,1000);
	TH1D *ex_he2_res = new TH1D("ex_he2_res","ex_he2",500,0,20);
	//----------------------------
	TRandom3* gRandom = new TRandom3();

	//cout<<nEvents<<endl;
        

    

    for(Int_t iEvent=0; iEvent<1000; iEvent++)
    //for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {
        
        
        
        Double_t energyLoss_p1=0.0;        
        Double_t energyLoss_p2=0.0;
        Double_t range_p1=0.0;
	Double_t range_p2=0.0;
	Double_t avetheta_p1=0.0;
	Double_t avetheta_p2=0.0;
	Int_t np1 = 0;
	Int_t np2 = 0;
	Double_t zpos = 0.0;
	Double_t xpos = 0.0;
	Double_t ypos = 0.0;
	Double_t rpos = 0.0;
	Double_t rpos_old = 0.0;
	Double_t zpos_old = 0.0;
	Double_t xpos_old = 0.0;
	Double_t ypos_old = 0.0;
	
        Double_t thetalab=0.0;
	Double_t philab=0.0;
	Double_t avephi_p1=0.0;
	Double_t avephi_p2=0.0;
	Double_t diff_theta=0.0;
	Double_t diff_phi=0.0;
	
	Double_t mom1_norm = 0.0;
	Double_t mom2_norm = 0.0;
	Double_t proton_mass = 1.0078250322*931.494;
	Double_t proj_mass = 12.00000000*931.494;
	Double_t target_mass = 2.01410177812*931.494;
	Double_t recoil_mass = 12.014352104*931.494;
	Double_t he2_mass = 2.0*proton_mass;
	Double_t kin_He2 =0;
	Double_t theta_He2 = 0;
	Double_t phi_He2 = 0;
	Double_t theta_cm = 0;
	Double_t Ex4 = 0;
	Double_t he2_mass_ex = 0;
	Double_t E_tot_2he = 0;
	Double_t av_dE_p1 = 0;
	Double_t av_dE_p2 = 0;

	

	TVector3 mom_proton1; 
	TVector3 mom_proton2;
	TVector3 mom_He2; 

        TVector3 mom_proton1_res; 
	TVector3 mom_proton2_res;
	TVector3 mom_He2_res; 

        Int_t n2=0;
	Int_t nrad=0;


        
	Double_t driftVelocity = 1.; //cm/um
        Double_t zMesh = 1000.0; //mm
        Double_t driftLength =  0; //mm
        Double_t D_coef_trans = 0.051;  //reduced D coef. = sqrt(2*D/v) [sqrt(mm)]
        Double_t D_coef_long = 0.085; //reduced D coef. = sqrt(2*D/v) [sqrt(mm)]       
        Double_t samplingtime = 0.080; //us
        Double_t sigstrtrans = 0;
        Double_t sigstrlong = 0;
        Double_t trans = 0;
        Double_t angulo = 0;
        Double_t propX = 0;
        Double_t propY = 0;
        Double_t propZ = 0;                
        Double_t driftTime = 0; //us
        Double_t pBin = 0;
        Int_t padNumber = 0;
        Int_t counter1 = 0;
        Int_t counter2 = 0;
        Int_t points1 = 0;
        Int_t points2 = 0;

        Double_t lastZ1 = 0;
        Double_t lastZ2 = 0;
        Double_t lastX1 = 0;
        Double_t lastX2 = 0;
	Double_t real_vertex = 0;         


        TString VolName;
        tree->GetEvent(iEvent);
        // tree -> GetEntry(iEvent);
        Int_t Npoints = pointArray -> GetEntries();
        //std::cout<<" Event Number : "<<iEvent<<std::endl;
	
        std::vector<Float_t> PadCenterCoord;

	
        
        TGraph2D * gr1 = new TGraph2D();
        TGraph2D * gr2 = new TGraph2D();
        TGraph * grphi1 = new TGraph();
        TGraph * grphi2 = new TGraph();
        

        for(Int_t i=0; i<Npoints; i++) {

            point = (AtTpcPoint*) pointArray -> At(i);
            VolName=point->GetVolName();
            //std::cout<<" Volume Name : "<<VolName<<std::endl;
            Int_t trackID = point -> GetTrackID();

	    

		if(iEvent%2!=0 && trackID!=1 && VolName=="drift_volume"){  //solo los protones
			xpos = point->GetXIn()*10; //in mm
			ypos = point->GetYIn()*10; //in mm
			rpos = sqrt(xpos*xpos + ypos*ypos);
			zpos = point->GetZIn()*10; //in mm
			tTime= point->GetTime()/1000; //us
			
			
			tracks_z_r->Fill(zpos, rpos, point -> GetEnergyLoss()*1e6);
			tracks_x_y->Fill(xpos, ypos, point -> GetEnergyLoss()*1e6);

			

			thetalab = fabs(atan2(rpos - rpos_old, zpos - zpos_old))*180/PI;
			philab = atan2(ypos - ypos_old, xpos - xpos_old);
			if(philab<0) philab+=2*PI;
			philab = philab*180/PI;
			angle_r->Fill(rpos, thetalab, point -> GetEnergyLoss()*1e6);
			angle_eloss->Fill(thetalab, point -> GetEnergyLoss()*1e6);
			rpos_old = rpos;
			zpos_old = zpos;
			xpos_old = xpos;
			ypos_old = ypos;

			if(i==0) real_vertex = zpos;

			if(trackID==2){
				np1++;
				energyLoss_p1 +=  point -> GetEnergyLoss()*1e3;// in MeV
				range_p1 = point -> GetLength()*10; //mm
				
				if(np1==2){
					avetheta_p1 = point->GetAIni();
					avephi_p1 = philab;
					}
				
				
				eloss_track->Fill(range_p1,point -> GetEnergyLoss()*1e6);
				//cout<<philab<<endl;
				//cout <<"  "<<range_p1<<"  "<<point -> GetEnergyLoss()*1e6<<endl;

				av_dE_p1 += point -> GetEnergyLoss()*1e6;

				
				//cout<<np1-1<<" "<<thetalab<<" "<<philab<<" "<<point->GetAIni()<<endl;

			}
			
			if(trackID==3){
				np2++;
				energyLoss_p2 +=  point -> GetEnergyLoss()*1e3;// in MeV
				range_p2 = point -> GetLength()*10; //mm
				
				if(np2==2){
					avetheta_p2 = point->GetAIni();
					avephi_p2 = philab;
					}
				//cout<<point ->GetEIni() <<"  "<<energyLoss_p2<<endl;
				av_dE_p2 += point -> GetEnergyLoss()*1e6;
                                //cout<<np2-1<<" "<<thetalab<<" "<<philab<<" "<<point->GetAIni()<<endl;
			}


                        driftLength       = fabs(zpos-zMesh); //mm
                        sigstrtrans       = D_coef_trans* sqrt(driftLength);//transverse diffusion coefficient
                        sigstrlong        = D_coef_long* sqrt(driftLength);//longitudal diffusion coefficient
                        Double_t meanX1 = 0;
                        Double_t meanY1 = 0;
                        Double_t meanZ1 = 0;
                        Double_t meanX2 = 0;
                        Double_t meanY2 = 0;
                        Double_t meanZ2 = 0;
                        counter1 = 0;
                        counter2 = 0;
                       
                       if(rpos>20){
                       for(int elec =0; elec<100; elec++){
                                trans	       = gRandom -> Gaus(0,sigstrtrans); //in mm
                                angulo         = gRandom->Uniform(0, TMath::TwoPi());
                                propX           = xpos + trans*TMath::Cos(angulo);
                                propY           = ypos + trans*TMath::Sin(angulo);
                                driftLength     = driftLength + (gRandom -> Gaus(0,sigstrlong)); //mm
                                driftTime       = samplingtime*(floor((((driftLength/10)/driftVelocity) +(tTime))/samplingtime) + 0.5); //us
                                propZ           = zMesh - driftVelocity*driftTime*10; //mm  
                                pBin            = fPadPlane->Fill(propX,propY,10);
                                padNumber       = pBin-1;
                                
                                if((padNumber<10240 && padNumber>0)  ){
                                        PadCenterCoord = map->CalcPadCenter(padNumber);   

                                        if(trackID==2 ){
                                                meanX1 += PadCenterCoord[0];
                                                meanY1 += PadCenterCoord[1];
                                                meanZ1 += propZ;
                                                lastZ1  = propZ;
                                                lastX1  = PadCenterCoord[0];                                                   
                                                counter1++;
                                                }
                                        if(trackID==3 ){
                                                meanX2 += PadCenterCoord[0];
                                                meanY2 += PadCenterCoord[1];
                                                meanZ2 += propZ;
                                                lastZ2  = propZ;                                                               
                                                lastX2  = PadCenterCoord[0];                                                              
                                                counter2++;
                                                }                                        
                                             
                                }

                                
                             }
                            }//if r>20

                        if(trackID==2  && counter1>0){
                                 
                                 gr1->SetPoint(points1,meanX1/counter1,meanZ1/counter1,meanY1/counter1);
                                 grphi1->SetPoint(points1,meanX1/counter1,meanY1/counter1);
                                 
                                 //cout<<points1-1<<"  "<<meanY1/counter1<<"  "<<meanZ1/counter1<<endl;
                                  points1++;
                                       }

                        if(trackID==3 && counter2>0){
                                 gr2->SetPoint(points2,meanX2/counter2,meanZ2/counter2,meanY2/counter2);
                                 grphi2->SetPoint(points2,meanX2/counter2,meanY2/counter2);
                                 
                                 points2++;
                                       }

		   }// only protons

		 
                 
        	}//N number of points (track)



                av_dE_p1 = av_dE_p1/np1;
		av_dE_p2 = av_dE_p2/np2;

                 
                //-----------Fitting tracks
                //----------------------------------------------------------------------------
                if(iEvent%2!=0 && points1>0 && points2>0){
                   //track 1
		   ROOT::Fit::Fitter  fitter;
   		// make the functor objet
   		   SumDistance2 sdist(gr1);
		   ROOT::Math::Functor fcn(sdist,4);
   		   // set the function and the initial parameter values
   		   double pStart[4] = {0.2,0.5,zpos,0.5};
   		   fitter.SetFCN(fcn,pStart);
		   
		   fitter.Config().ParSettings(0).SetLimits(-4.0,4.0);
		   fitter.Config().ParSettings(1).SetLimits(-20,20);
		   fitter.Config().ParSettings(2).SetLimits(-10,1000);
		   fitter.Config().ParSettings(3).SetLimits(-4,4);
		   
   		   // set step sizes different than default ones (0.3 times parameter values)
   		   for (int i = 0; i < 4; ++i) fitter.Config().ParSettings(i).SetStepSize(0.001);
   		   bool ok = fitter.FitFCN();
		   
   		   if (!ok) {
      		   Error("line3Dfit","Line3D Fit failed");
      		   std::cout <<"Proton 1"<<"   Event: "<<iEvent << std::endl;
   		   }
   		   const ROOT::Fit::FitResult & result = fitter.Result();
   		   //std::cout << "Total final distance square " << result.MinFcnValue()<<"  "<<iEvent << std::endl;
   		   //result.Print(std::cout);
		   // get fit parameters
   		   const double * parFit = result.GetParams();  

                    
                   
                   Double_t theta_p1_fit = fabs(atan(sqrt(parFit[1]*parFit[1] +1)/parFit[3] )*TMath::RadToDeg());
                   if(parFit[2]>lastZ1) theta_p1_fit = 180.0- theta_p1_fit;
                   

                   TFitResultPtr  res1 = grphi1->Fit("pol1","S"); 
                   Double_t phi_p1_fit = atan2(signo(lastX1)*signo(res1->Value(1))*fabs(res1->Value(1)),signo(lastX1))*TMath::RadToDeg();
                   if(phi_p1_fit<0) phi_p1_fit+=360;
                   

                   //cout<<res1->Value(0)<<" "<<res1->Value(1)<<" "<<phi_p1_fit<<" "<<theta_p1_fit<<endl;


                   //track 2
                   ROOT::Fit::Fitter  fitter2;
   		// make the functor objet
   		   SumDistance2 sdist2(gr2);
		   ROOT::Math::Functor fcn2(sdist2,4);
   		   // set the function and the initial parameter values
   		   //double pStart[4] = {0.2,0.5,zpos,0.5};
   		   fitter2.SetFCN(fcn2,pStart);
		   
		   fitter2.Config().ParSettings(0).SetLimits(-4.0,4.0);
		   fitter2.Config().ParSettings(1).SetLimits(-20,20);
		   fitter2.Config().ParSettings(2).SetLimits(-10,1000);
		   fitter2.Config().ParSettings(3).SetLimits(-4,4);
		   
   		   // set step sizes different than default ones (0.3 times parameter values)
   		   for (int i = 0; i < 4; ++i) fitter2.Config().ParSettings(i).SetStepSize(0.001);
   		   bool ok2 = fitter2.FitFCN();
		   
   		   if (!ok2) {
      		   Error("line3Dfit","Line3D Fit failed");
      		   std::cout <<"Proton 2"<<"   Event: "<<iEvent << std::endl;
   		   }
   		   const ROOT::Fit::FitResult & result2 = fitter2.Result();
   		   //std::cout << "Total final distance square " << result2.MinFcnValue()<<"  "<<iEvent << std::endl;
   		   //result2.Print(std::cout);
		   // get fit parameters
   		   const double * parFit2 = result2.GetParams();  

                   Double_t theta_p2_fit = fabs(atan(sqrt(parFit2[1]*parFit2[1] +1)/parFit2[3] )*TMath::RadToDeg());
                   if(parFit2[2]>lastZ2) theta_p2_fit = 180.0- theta_p2_fit;
                   

                   TFitResultPtr  res2 = grphi2->Fit("pol1","S"); 
                   Double_t phi_p2_fit = atan2(signo(lastX2)*signo(res2->Value(1))*fabs(res2->Value(1)),signo(lastX2))*TMath::RadToDeg();
                   if(phi_p2_fit<0) phi_p2_fit+=360;
                   

                   //cout<<parFit[2]<<" "<<parFit2[2]<<" "<<phi_p2_fit<<" "<<theta_p2_fit<<endl;

                bool goodfit = false;
                if(ok==true && ok2==true &&  (fabs(parFit2[2] -parFit[2])< 10)) goodfit = true;
                 

                if(goodfit==true && (av_dE_p1 >5 && av_dE_p2 > 5) ){
                        
                //cout<<"Fitted angles*******"<<endl;
                //cout<<theta_p1_fit<<"  "<<phi_p1_fit<<"  "<<theta_p2_fit<<"  "<<phi_p2_fit<<endl;
                    // reconstruction of 2He
		Double_t mom1_norm_res = TMath::Sqrt(energyLoss_p1*energyLoss_p1 + 2.0*energyLoss_p1*proton_mass);
		mom_proton1_res.SetX(mom1_norm_res*TMath::Sin(theta_p1_fit*PI/180)*TMath::Cos(phi_p1_fit*PI/180));
		mom_proton1_res.SetY(mom1_norm_res*TMath::Sin(theta_p1_fit*PI/180)*TMath::Sin(phi_p1_fit*PI/180));
		mom_proton1_res.SetZ(mom1_norm_res*TMath::Cos(theta_p1_fit*PI/180));

		Double_t mom2_norm_res = TMath::Sqrt(energyLoss_p2*energyLoss_p2 + 2.0*energyLoss_p2*proton_mass);
		mom_proton2_res.SetX(mom2_norm_res*TMath::Sin(theta_p2_fit*PI/180)*TMath::Cos(phi_p2_fit*PI/180));
		mom_proton2_res.SetY(mom2_norm_res*TMath::Sin(theta_p2_fit*PI/180)*TMath::Sin(phi_p2_fit*PI/180));
		mom_proton2_res.SetZ(mom2_norm_res*TMath::Cos(theta_p2_fit*PI/180));

		

		//if(abs(avephi_p1-avephi_p2)>100){
		//if(energyLoss_p1>0.1 && energyLoss_p2>0.1){
		mom_He2_res = mom_proton1_res + mom_proton2_res;

	        E_tot_2he = (proton_mass + energyLoss_p1) + (proton_mass + energyLoss_p2);
		he2_mass_ex = TMath::Sqrt(E_tot_2he*E_tot_2he - mom_He2_res.Mag2());
		ex_he2_res->Fill(he2_mass_ex - he2_mass);

		kin_He2 = TMath::Sqrt(mom_He2_res.Mag2() + he2_mass_ex*he2_mass_ex) -he2_mass_ex;
		//kin_He2 = energyLoss_p1 + energyLoss_p2;
		theta_He2 = mom_He2_res.Theta()*180/PI;
		phi_He2 = mom_He2_res.Phi()*180/PI;
		theta_r_he2_res->Fill(theta_He2);
		phi_r_he2_res->Fill(phi_He2);
		kin_r_he2_res->Fill(kin_He2);
		theta_kin_2he_res->Fill(theta_He2, kin_He2);

		

                d2he_obj->kine_2b(proj_mass, target_mass,  he2_mass_ex,  recoil_mass, 1200, theta_He2*TMath::DegToRad(), kin_He2);

       double *kinematics_res = kine_2b(proj_mass, target_mass,  he2_mass_ex,  recoil_mass, 1200, theta_He2*TMath::DegToRad(), kin_He2);

		  

		theta_cm = *(kinematics_res+0)*TMath::RadToDeg();
        	Ex4 = *(kinematics_res+1);


                cout<< theta_cm<<"  "<< Ex4<<endl;              
                cout<< d2he_obj->GetThetaCM()<<"  "<< d2he_obj->GetMissingMass()<<endl;              		
                cout<<endl;

		thetacm_he2->Fill(theta_cm);
		Ex_res_res->Fill(Ex4);
		thetacm_Ex_2he_res->Fill(theta_cm,Ex4);

		vertex_reco->Fill(real_vertex,0.5*(parFit2[2] + parFit[2]) );

                } //succesful fit for the two tracks

                        /*
                        if(iEvent==1){

                
                        
                        gr1->Draw("p0");
			  int n = 1000;
			  double t0 = 0;
   			  double dt = 10;
   			  TPolyLine3D *l = new TPolyLine3D(n);
   			  for (int i = 0; i <n;++i) {
      			  double t = t0+ dt*i/n;
      			  double x,y,z;
      			  line(t,parFit,x,y,z);
      			  l->SetPoint(i,x,y,z);
   				}
   			  l->SetLineColor(kRed);
   			  l->Draw("same");
			  // l->Draw();
			  cout<<"hallo----------------------------------------------------"<<endl;
                        break;

                        }
                        */


                } //fitting tracks

                //----------------------------------------------------------------------------
               
               

		
		if(av_dE_p1 >5 && av_dE_p2 > 5 && iEvent%2!=0){

		//if(energyLoss_p1==0 || energyLoss_p2==0) continue;
		//avetheta_p1 = avetheta_p1/np1;
		//avetheta_p2 = avetheta_p2/np2;
		//avephi_p1 = avephi_p1/np1;
		//avephi_p2 = avephi_p2/np2;
		
		
		diff_theta = (avetheta_p1 - avetheta_p2);
		diff_phi = (avephi_p1 - avephi_p2);

		range_eloss_p1->Fill(range_p1,energyLoss_p1);
		range_eloss_p2->Fill(range_p2,energyLoss_p2);
		avth_eloss_p1->Fill(avetheta_p1,energyLoss_p1);
		avth_eloss_p2->Fill(avetheta_p2,energyLoss_p2);
		if(fabs(avephi_p1-avephi_p2)>100) avth1_avth2->Fill(avetheta_p1,avetheta_p2);
		Eloss1_Eloss2->Fill(energyLoss_p1,energyLoss_p2);
		avphi1_avphi2->Fill(avephi_p1,avephi_p2);
		diffth_diffphi->Fill(diff_theta,diff_phi);
		Eloss_p->Fill(energyLoss_p1);
		Eloss_p->Fill(energyLoss_p2);

                //cout<<"Real angles*******"<<endl;
                //cout<<avetheta_p1<<"  "<<avephi_p1<<"  "<<avetheta_p2<<"  "<<avephi_p2<<endl;

		// reconstruction of 2He
		mom1_norm = TMath::Sqrt(energyLoss_p1*energyLoss_p1 + 2.0*energyLoss_p1*proton_mass);
		mom_proton1.SetX(mom1_norm*TMath::Sin(avetheta_p1*PI/180)*TMath::Cos(avephi_p1*PI/180));
		mom_proton1.SetY(mom1_norm*TMath::Sin(avetheta_p1*PI/180)*TMath::Sin(avephi_p1*PI/180));
		mom_proton1.SetZ(mom1_norm*TMath::Cos(avetheta_p1*PI/180));

		mom2_norm = TMath::Sqrt(energyLoss_p2*energyLoss_p2 + 2.0*energyLoss_p2*proton_mass);
		mom_proton2.SetX(mom2_norm*TMath::Sin(avetheta_p2*PI/180)*TMath::Cos(avephi_p2*PI/180));
		mom_proton2.SetY(mom2_norm*TMath::Sin(avetheta_p2*PI/180)*TMath::Sin(avephi_p2*PI/180));
		mom_proton2.SetZ(mom2_norm*TMath::Cos(avetheta_p2*PI/180));

		

		//if(fabs(avephi_p1-avephi_p2)>100){
		//if(energyLoss_p1>0.1 && energyLoss_p2>0.1){
		mom_He2 = mom_proton1 + mom_proton2;

		E_tot_2he = (proton_mass + energyLoss_p1) + (proton_mass + energyLoss_p2);
		he2_mass_ex = TMath::Sqrt(E_tot_2he*E_tot_2he - mom_He2.Mag2());
		ex_he2->Fill(he2_mass_ex - he2_mass);

		kin_He2 = TMath::Sqrt(mom_He2.Mag2() + he2_mass_ex*he2_mass_ex) -he2_mass_ex;
		//kin_He2 = energyLoss_p1 + energyLoss_p2;
		theta_He2 = mom_He2.Theta()*180/PI;
		phi_He2 = mom_He2.Phi()*180/PI;
		theta_r_he2->Fill(theta_He2);
		phi_r_he2->Fill(phi_He2);
		kin_r_he2->Fill(kin_He2);
		theta_kin_2he->Fill(theta_He2, kin_He2);


                //cout<<theta_He2<<" "<<mom1_norm<<" "<<mom2_norm<<" "<<avetheta_p1<<" "<<avetheta_p2<<" "<<avephi_p1<<" "<<avephi_p2<<endl;		

       double *kinematics = kine_2b(proj_mass, target_mass,  he2_mass_ex,  recoil_mass, 1200, theta_He2*TMath::DegToRad(), kin_He2);

		
        
		theta_cm = *(kinematics+0)*TMath::RadToDeg();
        	Ex4 = *(kinematics+1);
		
		thetacm_he2_res->Fill(theta_cm);
		Ex_res->Fill(Ex4);
		thetacm_Ex_2he->Fill(theta_cm,Ex4);

		//cout<<mom_He2.Mag2()<<" "<< he2_mass_ex - he2_mass <<"  "<<Ex4<<endl;

	        }//if cut high energy particles
                
                delete gr1;
                delete gr2;
                delete grphi1;
                delete grphi2;
                
                
              
	
	} //number of events

	

        
        

	outfile->cd("Parameters");
	tracks_z_r->Write();
	tracks_x_y->Write();
	angle_r->Write();
	angle_eloss->Write();
	range_eloss_p1->Write();
	range_eloss_p2->Write();
	avth_eloss_p1->Write();
	avth_eloss_p2->Write();
	avth1_avth2->Write();
	Eloss1_Eloss2->Write();
	avphi1_avphi2->Write();
	diffth_diffphi->Write();	
	eloss_track->Write();
	Eloss_p->Write();

	outfile->cd("He2_reconstr");
	theta_r_he2->Write();
	phi_r_he2->Write();
	kin_r_he2->Write();
	theta_kin_2he->Write();
	thetacm_he2->Write();
	Ex_res->Write();
	ex_he2->Write();
	thetacm_Ex_2he->Write();

        outfile->cd("He2_reconstr_resol");
	theta_r_he2_res->Write();
	phi_r_he2_res->Write();
	kin_r_he2_res->Write();
	theta_kin_2he_res->Write();
	thetacm_he2_res->Write();
	Ex_res_res->Write();
	ex_he2_res->Write();
	thetacm_Ex_2he_res->Write();
	vertex_reco->Write();

	outfile->Close();


	cout<<"*************************************"<<endl;
	cout<<endl;	
    	final=clock();
	Ttotal=(final-inicio)/(double) CLOCKS_PER_SEC;
	cout<<"tiempo de ejecucion: "<<Ttotal<<" segundos"<<endl;

}

