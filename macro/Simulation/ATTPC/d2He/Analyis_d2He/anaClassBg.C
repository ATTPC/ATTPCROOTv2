#define anaClassBg_cxx
#include "anaClassBg.h"
#include <TH2.h>
#include <TStyle.h>
#include <TCanvas.h>



#include<iostream>
#include<fstream>
#include<cmath>
#include<string>
#include <stdlib.h> 
#include <vector>
#include <ctime>
#include "spline.h"  //funcion externa que me hace interpolacion cubica entre dos puntos


using namespace std;



double sq(double val ){
	
	return val*val;
}


double omega(double x, double y, double z){
	return sqrt(x*x + y*y + z*z -2*x*y -2*y*z -2*x*z);

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


double GetEloss(double Eres, double lres, double trackmiss, std::vector<Double_t> X, std::vector<Double_t> Y){
	
	double dE = 0.0;
	if(lres>350){ //the particle punch through the detector
		dE = (Eres/lres)*trackmiss;
		}
	else{

		 tk::spline s;
   		 s.set_points(X,Y);

		 dE = s((Eres/lres)*trackmiss + Eres)*trackmiss;
		 //if(dE>Eres) dE = (Eres/lres)*trackmiss;

		}

	return dE;
}


int  *Selecttracks(std::vector<Double_t>* V){

        static int tracksP[2];
        double min = 1000;
        double dist = 0;
        
        for(int i=0; i<V->size(); i++){

                for(int j=i+1; j<V->size(); j++){

                        dist = fabs(V->at(i) - V->at(j));
                        if(dist< min  && V->at(i)!=-10000){ min = dist; tracksP[0] = i; tracksP[1] = j;}
                        }

        }

        return tracksP;
        }



void anaClassBg::Loop()
{
//   In a ROOT session, you can do:
//      root> .L anaClassBg.C
//      root> anaClassBg t
//      root> t.GetEntry(12); // Fill t data members with entry number 12
//      root> t.Show();       // Show values of entry 12
//      root> t.Show(16);     // Read and show values of entry 16
//      root> t.Loop();       // Loop on all entries
//

//     This is the loop skeleton where:
//    jentry is the global entry number in the chain
//    ientry is the entry number in the current Tree
//  Note that the argument to GetEntry must be:
//    jentry for TChain::GetEntry
//    ientry for TTree::GetEntry and TBranch::GetEntry
//
//       To read only selected branches, Insert statements like:
// METHOD1:
//    fChain->SetBranchStatus("*",0);  // disable all branches
//    fChain->SetBranchStatus("branchname",1);  // activate branchname
// METHOD2: replace line
//    fChain->GetEntry(jentry);       //read all branches
//by  b_branchname->GetEntry(ientry); //read only this branch


  	//-----------------tiempo-------------------------------------------
	Float_t Ttotal,inicio, final;
	inicio=clock();
	//------------------------------------------------------------------



   fChain->SetBranchStatus("Elosshibg", 1);
   fChain->SetBranchStatus("Rangelowbg", 1);
   fChain->SetBranchStatus("Rangehibg", 1);
   fChain->SetBranchStatus("Thetabg", 1);
   fChain->SetBranchStatus("Phibg", 1);
   fChain->SetBranchStatus("Av_elossbg", 1);
   fChain->SetBranchStatus("Goodfitbg", 1);
   fChain->SetBranchStatus("Vertexbg", 1);
   fChain->SetBranchStatus("ContaBg", 1);

   if (fChain == 0) return;

   Long64_t nentries = fChain->GetEntriesFast();
   Long64_t nbytes = 0, nb = 0;




//---------------leemos stopping power table

	vector<double> X(78), Y(78);
	string filename= "StoPow_proton_D2.dat";  
  	ifstream  entrada;
	double col1, col2;
	entrada. open(filename.c_str());
      	if(entrada.fail() ){
                       cerr << "error abriendo "<< filename << endl;
 			exit(1);
                      }  

         for(int k=0;k<78;k++){
        	entrada >> col1 >> col2 ;  //defino el numero de columnas   				
		X[k]=col1; Y[k]=col2;
		}
	entrada.close();


//-------definimos unos histogramas
	TFile* outfile;
        //std::stringstream string_hole;
        //string_hole<<"_"<<elhueco;
        //TString  outFileNameHead = "hist_d2He_12CBgonly" + geofile + string_hole.str() + ".root";
        TString  outFileNameHead = "bgtracks4.root";
	outfile   = TFile::Open(outFileNameHead.Data(),"recreate");


        TH1D *theta_r_he2_res = new TH1D("theta_r_he2_res","theta 2He",500,0,180);
	TH1D *kin_r_he2_res = new TH1D("kin_r_he2_res","Energy 2He",500,0,5);
	TH1D *phi_r_he2_res = new TH1D("phi_r_he2_res","phi 2He",500,-180,180);
	TH2D *theta_kin_2he_res = new TH2D("theta_kin_2he_res","Kin vs Theta 2He",500,0,180,500,0,5);
	TH1D *thetacm_he2_res = new TH1D("thetacm_he2_res","thetacm_he2",500,0,20);
	TH1D *Ex_res_res = new TH1D("Ex_res_res","Ex_res",500,-10,30);
	TH2D *thetacm_Ex_2he_res = new TH2D("thetacm_Ex_2he_res","thetacm_Ex_2he",500,0,20,500,-10,30);
	TH2D *vertex_reco = new TH2D("vertex_reco","vertex_reco",500,-10,1000,500,-10,1000);
	TH2D *theta_res = new TH2D("theta_res","theta_res",500,0,90,500,0,90);
	TH1D *ex_he2_res = new TH1D("ex_he2_res","ex_he2",500,0,20);
	TH2D *ener_reco = new TH2D("ener_reco","ener_reco",300,0,3,300,0,3);
	TH2D *ener_reco_ratio = new TH2D("ener_reco_ratio","ener_reco_ratio",300,0,3,200,-2,2);


//cambiar estas lineas para diferente beam
	Double_t mass_proj_uma =  12.000000000;
	Double_t mass_reco_uma = 12.014352104;
	Double_t Ekin_proj = 1200.0;

	Double_t mom1_norm = 0.0;
	Double_t mom2_norm = 0.0;
	Double_t proton_mass = 1.0078250322*931.494;
	Double_t proj_mass = mass_proj_uma*931.494;
	Double_t target_mass = 2.01410177812*931.494;
	Double_t recoil_mass = mass_reco_uma*931.494;
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


        srand( time( NULL ) );



for(int eventos = 0; eventos<100000; eventos++){

     Int_t ntracks = 4;
     
     vector<int> S;
     bool goodevt ;
     
        if( (eventos %1000) == 0) cout<<" Event number...."<<eventos<<endl;
//----------------escojemos algunos tracks
        for(int j=0; j<ntracks; j++){
                int  number; 
                
                do{
	                number = 200000* ((Float_t)rand()/(Float_t)RAND_MAX); 
                        //cout<<number<<endl;
                        goodevt = true;
                        for(int i=0; i<S.size();i++){ 
                                if(number == S.at(i)) goodevt = false;
                        }
	        }while(   goodevt == false);

                 Long64_t ientry = LoadTree(number);
                 if (ientry < 0) break;
                 nb = fChain->GetEntry(number);   nbytes += nb;
                 S.push_back(number);
                 //cout<<ContaBg<<"  "<<Vertexbg<<endl;
        }


        vector<double> Vertex(ntracks);
                 Double_t Eloss_Hi[ntracks] ;
                 Double_t RangeLow[ntracks] ;
                 Double_t RangeHi[ntracks] ;
                 Double_t ThetaL[ntracks] ;
                 Double_t PhiL[ntracks] ;
                 Double_t Av_Eloss[ntracks] ;
                 bool GoodFit[ntracks] ;

        for(int i=0; i<S.size();i++){

                  Long64_t ientry = LoadTree(S.at(i));
                  if (ientry < 0) break;
                  nb = fChain->GetEntry(S.at(i));   nbytes += nb;
                 Vertex.at(i) = Vertexbg;
                 Eloss_Hi[i] = Elosshibg ;
                 RangeLow[i] = Rangelowbg ;
                 RangeHi[i] = Rangehibg ;
                 ThetaL[i] = Thetabg ;
                 PhiL[i] = Phibg ;
                 Av_Eloss[i] = Av_elossbg ;
                 GoodFit[i] = Goodfitbg ;
                //cout<<S.at(i)<<"  "<<Vertex.at(i)<<endl;

        }


//---------------encontrar la menor distancia entre vertices
                int* stracks = Selecttracks(&Vertex);
                 int track_i = *(stracks+0);
                 int track_k = *(stracks+1);
                //cout<<*(stracks+0)<<"  "<<*(stracks+1)<<endl;


                bool goodfit = false;
                if(GoodFit[track_i]==true && GoodFit[track_k]==true &&  (fabs(Vertex.at(track_i) - Vertex.at(track_k))< 20)  ) goodfit = true;
                 

                if(goodfit==true && (Av_Eloss[track_i] > 5 && Av_Eloss[track_k] > 5) ){

                        //cout<<"hello!"<<endl;
                         Double_t eLoss_p1_reco = Eloss_Hi[track_i] + GetEloss(Eloss_Hi[track_i], RangeHi[track_i],RangeLow[track_i], X, Y);
		         Double_t eLoss_p2_reco = Eloss_Hi[track_k] + GetEloss(Eloss_Hi[track_k], RangeHi[track_k],RangeLow[track_k], X, Y);


                            // reconstruction of 2He
		//Double_t mom1_norm_res = TMath::Sqrt(energyLoss_p1*energyLoss_p1 + 2.0*energyLoss_p1*proton_mass); //e_perfect
		Double_t mom1_norm_res = TMath::Sqrt(eLoss_p1_reco*eLoss_p1_reco + 2.0*eLoss_p1_reco*proton_mass); //e_reco
		mom_proton1_res.SetX(mom1_norm_res*TMath::Sin(ThetaL[track_i]*TMath::Pi()/180)*TMath::Cos(PhiL[track_i]*TMath::Pi()/180));
		mom_proton1_res.SetY(mom1_norm_res*TMath::Sin(ThetaL[track_i]*TMath::Pi()/180)*TMath::Sin(PhiL[track_i]*TMath::Pi()/180));
		mom_proton1_res.SetZ(mom1_norm_res*TMath::Cos(ThetaL[track_i]*TMath::Pi()/180));

		//Double_t mom2_norm_res = TMath::Sqrt(energyLoss_p2*energyLoss_p2 + 2.0*energyLoss_p2*proton_mass);
		Double_t mom2_norm_res = TMath::Sqrt(eLoss_p2_reco*eLoss_p2_reco + 2.0*eLoss_p2_reco*proton_mass);
		mom_proton2_res.SetX(mom2_norm_res*TMath::Sin(ThetaL[track_k]*TMath::Pi()/180)*TMath::Cos(PhiL[track_k]*TMath::Pi()/180));
		mom_proton2_res.SetY(mom2_norm_res*TMath::Sin(ThetaL[track_k]*TMath::Pi()/180)*TMath::Sin(PhiL[track_k]*TMath::Pi()/180));
		mom_proton2_res.SetZ(mom2_norm_res*TMath::Cos(ThetaL[track_k]*TMath::Pi()/180));

		

		//if(abs(avephi_p1-avephi_p2)>100){
		//if(energyLoss_p1>0.1 && energyLoss_p2>0.1){
		mom_He2_res = mom_proton1_res + mom_proton2_res;

	        //E_tot_2he = (proton_mass + energyLoss_p1) + (proton_mass + energyLoss_p2);
		E_tot_2he = (proton_mass + eLoss_p1_reco) + (proton_mass + eLoss_p2_reco);
		he2_mass_ex = TMath::Sqrt(E_tot_2he*E_tot_2he - mom_He2_res.Mag2());
		ex_he2_res->Fill(he2_mass_ex - he2_mass);

		kin_He2 = TMath::Sqrt(mom_He2_res.Mag2() + he2_mass_ex*he2_mass_ex) -he2_mass_ex;
		//kin_He2 = energyLoss_p1 + energyLoss_p2;
		theta_He2 = mom_He2_res.Theta()*180/TMath::Pi();
		phi_He2 = mom_He2_res.Phi()*180/TMath::Pi();
		theta_r_he2_res->Fill(theta_He2);
		phi_r_he2_res->Fill(phi_He2);
		kin_r_he2_res->Fill(kin_He2);
		theta_kin_2he_res->Fill(theta_He2, kin_He2);

                double *kinematics = kine_2b(proj_mass, target_mass,  he2_mass_ex,  recoil_mass, Ekin_proj, theta_He2*TMath::DegToRad(), kin_He2);

		
        
		theta_cm = *(kinematics+0)*TMath::RadToDeg();
        	Ex4 = *(kinematics+1);

                //cout<<Ex4<<endl;
                thetacm_he2_res->Fill(theta_cm);
		Ex_res_res->Fill(Ex4);
		thetacm_Ex_2he_res->Fill(theta_cm,Ex4);
	
		//vertex_reco->Fill(real_vertex,0.5*(Vertex.at(track_i) + Vertex.at(track_k)) );
		//ener_reco->Fill(energyLoss_p1, eLoss_p1_reco);
		//ener_reco->Fill(energyLoss_p2, eLoss_p2_reco);
		//ener_reco_ratio->Fill(energyLoss_p1, (energyLoss_p1-eLoss_p1_reco)/energyLoss_p1);
		//ener_reco_ratio->Fill(energyLoss_p2, (energyLoss_p2-eLoss_p2_reco)/energyLoss_p2);


                }
		
}




        theta_r_he2_res->Write();
	phi_r_he2_res->Write();
	kin_r_he2_res->Write();
	theta_kin_2he_res->Write();
	thetacm_he2_res->Write();
	Ex_res_res->Write();
	ex_he2_res->Write();
	thetacm_Ex_2he_res->Write();
	vertex_reco->Write();
	theta_res->Write();
	ener_reco->Write();
	ener_reco_ratio->Write();
        outfile->Close();
   
        cout<<"*************************************"<<endl;
	cout<<endl;	
    	final=clock();
	Ttotal=(final-inicio)/(double) CLOCKS_PER_SEC;
	cout<<"tiempo de ejecucion: "<<Ttotal<<" segundos"<<endl;


   
}
