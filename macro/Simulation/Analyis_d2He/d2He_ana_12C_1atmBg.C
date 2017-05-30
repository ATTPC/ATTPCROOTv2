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


double sq(double val ){
	
	return val*val;
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
                        if(dist< min && V->at(i)!=-1000 ){ min = dist; tracksP[0] = i; tracksP[1] = j;}
                        }

        }

        return tracksP;
        }




void d2He_ana_12C_1atmBg(TString geofile = "_1atm", Double_t elhueco=20)
{


	//-----------------tiempo-------------------------------------------
	Float_t Ttotal,inicio, final;
	inicio=clock();
	//------------------------------------------------------------------

    TString pathtodata = "/mnt/simulations/attpcroot/data/";
    TString mcFileNameHead = "attpcsim_d2He_12C" + geofile;
    TString mcFileNameTail = ".root";
    TString mcFileName     = pathtodata + mcFileNameHead + mcFileNameTail;
    //TString mcFileName     = pathtodata + "attpcsim_test.root";
    std:cout << " Analysis of simulation file  " << mcFileName << endl;

    AtTpcPoint* point = new AtTpcPoint();
    TClonesArray *pointArray=0;
    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");

    tree = (TTree*) file -> Get("cbmsim");
    //TBranch *branch = tree->GetBranch("AtTpcPoint");
    tree -> SetBranchAddress("AtTpcPoint", &pointArray);
    Int_t nEvents = tree -> GetEntriesFast();

        //------------------Opening a Bg file
        TString mcFileNameBg     = pathtodata + "attpcsim_d2He_Bg12C_1atm.root";
        std:cout << " Analysis of simulation file  " << mcFileNameBg << endl;
        AtTpcPoint* pointBg = new AtTpcPoint();
        TClonesArray *pointArrayBg=0;
        TFile* fileBg = new TFile(mcFileNameBg.Data(),"READ");
        TTree* treeBg = (TTree*) fileBg -> Get("cbmsim");
        treeBg = (TTree*) fileBg -> Get("cbmsim");
        treeBg -> SetBranchAddress("AtTpcPoint", &pointArrayBg);
        Int_t nEventsBg = treeBg -> GetEntriesFast();

        // ***************Create ATTPC Pad Plane***************************
    TString scriptfile = "Lookup20150611.xml";
    TString dir = getenv("VMCWORKDIR");
    TString scriptdir = dir + "/scripts/"+ scriptfile;

     AtTpcMap *map = new AtTpcMap();
     TH2Poly *fPadPlane;                  //!< pad plane
     map->GenerateATTPC();
     Bool_t MapIn = map->ParseXMLMap(scriptdir);
     fPadPlane = map->GetATTPCPlane();

        //------------------Create ATd2HeAnalysis-------------------------------
      ATd2HeAnalysis *d2heana = new ATd2HeAnalysis();


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
        std::stringstream string_hole;
        string_hole<<"_"<<elhueco;
        TString  outFileNameHead = "hist_d2He_12CBgonly" + geofile + string_hole.str() + ".root";
        //TString  outFileNameHead = "test.root";
	outfile   = TFile::Open(outFileNameHead.Data(),"recreate");
         outfile->mkdir("Parameters");
	 outfile->mkdir("He2_reconstr");
         outfile->mkdir("He2_reconstr_resol");


        
        TTree* rootTree =  new TTree( "bgtracks" , "bgtracks" );
        Double_t elosshibg ;
        Double_t rangelowbg ;
        Double_t rangehibg ;
        Double_t thetabg ;
        Double_t phibg ;
        Double_t av_elossbg ;
        bool goodfitbg ; 
        Double_t vertexbg ; 
        Int_t contaBg = 0;
        rootTree->Branch( "Elosshibg" , &elosshibg );
        rootTree->Branch( "Rangelowbg" , &rangelowbg );
        rootTree->Branch( "Rangehibg" , &rangehibg );
        rootTree->Branch( "Thetabg" , &thetabg );
        rootTree->Branch( "Phibg" , &phibg );
        rootTree->Branch( "Av_elossbg" , &av_elossbg );
        rootTree->Branch( "Goodfitbg" , &goodfitbg );
        rootTree->Branch( "Vertexbg" , &vertexbg );
        rootTree->Branch( "ContaBg" , &contaBg );
        


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
	TH2D *ereal_eloss = new TH2D("ereal_eloss","ereal_eloss",500,0,100,500,0,3);	
	TH2D *ereal_thres = new TH2D("ereal_thres","ereal_thres",500,0,100,500,0,30);	


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
	TH1D *Ex_res_res = new TH1D("Ex_res_res","Ex_res",500,-10,30);
	TH2D *thetacm_Ex_2he_res = new TH2D("thetacm_Ex_2he_res","thetacm_Ex_2he",500,0,20,500,-10,30);
	TH2D *vertex_reco = new TH2D("vertex_reco","vertex_reco",500,-10,1000,500,-10,1000);
	TH2D *theta_res = new TH2D("theta_res","theta_res",500,0,90,500,0,90);
	TH1D *ex_he2_res = new TH1D("ex_he2_res","ex_he2",500,0,20);
	TH2D *ener_reco = new TH2D("ener_reco","ener_reco",300,0,3,300,0,3);
	TH2D *ener_reco_ratio = new TH2D("ener_reco_ratio","ener_reco_ratio",300,0,3,200,-2,2);
        
	//----------------------------
	TRandom3* gRandom = new TRandom3();

	//cout<<nEvents<<endl;
        

    

    //for(Int_t iEvent=0; iEvent<100; iEvent++)
    for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {
        
        
        Double_t realkinE = 0.0;
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
        Double_t zbeam = 0.0;
	Double_t xbeam = 0.0;
	Double_t ybeam = 0.0;
	Double_t rbeam = 0.0;
	Double_t rpos_old = 0.0;
	Double_t zpos_old = 0.0;
	Double_t xpos_old = 0.0;
	Double_t ypos_old = 0.0;
	
	Double_t eLoss_p1_hi=0.0;        
        Double_t eLoss_p2_hi=0.0;        
	Double_t eLoss_p1_reco=0.0;        
        Double_t eLoss_p2_reco=0.0;        
	Double_t zpos1_hi = 0.0;
	Double_t xpos1_hi = 0.0;
	Double_t ypos1_hi = 0.0;
	Double_t zpos2_hi = 0.0;
	Double_t xpos2_hi = 0.0;
	Double_t ypos2_hi = 0.0;
	bool track1_flag = false;
	bool track2_flag = false;
	Double_t range1_hi = 0.0;
	Double_t range2_hi = 0.0;
	Double_t range1_low = 0.0;
	Double_t range2_low = 0.0;
	Double_t holeradius = elhueco; // mm
        Double_t threseloss = 0.0;
        if(geofile == "_1atm") threseloss = 5.0; 
        if(geofile == "_07atm") threseloss = 3.8;
        if(geofile == "_05atm") threseloss = 3.0;
        if(geofile == "_03atm") threseloss = 1.8;


        Int_t npbg1 =0;
        Double_t eLoss_pbg1_hi=0.0;        
        Double_t zposbg1_hi = 0.0;
	Double_t xposbg1_hi = 0.0;
	Double_t yposbg1_hi = 0.0;
        Double_t rangebg1_hi = 0.0;
        Double_t rangebg1_lo = 0.0;
        bool trackbg1_flag = false;
        Double_t av_dE_pbg1 = 0;
        Double_t energyLoss_pbg1 = 0;
        Int_t npbg2 =0;
        Double_t eLoss_pbg2_hi=0.0;        
        Double_t zposbg2_hi = 0.0;
	Double_t xposbg2_hi = 0.0;
	Double_t yposbg2_hi = 0.0;
        Double_t rangebg2_hi = 0.0;
        Double_t rangebg2_lo = 0.0;
        bool trackbg2_flag = false;
        Double_t av_dE_pbg2 = 0;
        Double_t energyLoss_pbg2 = 0;
        Int_t npbg3 =0;
        Double_t eLoss_pbg3_hi=0.0;        
        Double_t zposbg3_hi = 0.0;
	Double_t xposbg3_hi = 0.0;
	Double_t yposbg3_hi = 0.0;
        Double_t rangebg3_hi = 0.0;
        Double_t rangebg3_lo = 0.0;
        bool trackbg3_flag = false;
        Double_t av_dE_pbg3 = 0;
        Double_t energyLoss_pbg3 = 0;
        Int_t npbg4 =0;
        Double_t eLoss_pbg4_hi=0.0;        
        Double_t zposbg4_hi = 0.0;
	Double_t xposbg4_hi = 0.0;
	Double_t yposbg4_hi = 0.0;
        Double_t rangebg4_hi = 0.0;
        Double_t rangebg4_lo = 0.0;
        bool trackbg4_flag = false;
        Double_t av_dE_pbg4 = 0;
        Double_t energyLoss_pbg4 = 0;
        Double_t Pi1bg[3], Pi2bg[3], Pi3bg[3], Pi4bg[3];
        Double_t Pf1bg[3], Pf2bg[3], Pf3bg[3], Pf4bg[3];
        

        Double_t theta_p1_fit ;
        Double_t phi_p1_fit ;
        Double_t vertex_p1_fit ;
        bool goodfit_p1 ;
        Double_t theta_p2_fit ;
        Double_t phi_p2_fit ;
        Double_t vertex_p2_fit ;
        bool goodfit_p2 ;
        Double_t theta_p1bg_fit ;
        Double_t phi_p1bg_fit ;
        Double_t vertex_p1bg_fit = -10000 ;
        bool goodfit_p1bg ;
        Double_t theta_p2bg_fit ;
        Double_t phi_p2bg_fit ;
        Double_t vertex_p2bg_fit = -10000 ;
        bool goodfit_p2bg ;
        Double_t theta_p3bg_fit ;
        Double_t phi_p3bg_fit ;
        Double_t vertex_p3bg_fit = -10000 ;
        bool goodfit_p3bg ;
        Double_t theta_p4bg_fit ;
        Double_t phi_p4bg_fit ;
        Double_t vertex_p4bg_fit = -10000;
        bool goodfit_p4bg ;

        Double_t thetalab=0.0;
	Double_t philab=0.0;
	Double_t avephi_p1=0.0;
	Double_t avephi_p2=0.0;
	Double_t diff_theta=0.0;
	Double_t diff_phi=0.0;
	
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

        Int_t n2=0;
	Int_t nrad=0;

	Double_t energypoint = 0.0;
        Double_t fanofactor = 0.2;
	Double_t ion_pot = 40e-6; //in MeV
	Int_t  Nelectrons = 0;
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
	Double_t random_lenght = 0;
	Double_t random_rad = 0;
	Double_t driftLength_fold = 0;

        Double_t lastZ1 = 0;
        Double_t lastZ2 = 0;
        Double_t lastX1 = 0;
        Double_t lastX2 = 0;
        Double_t lastXbg[4], lastZbg[4];
	Double_t real_vertex = 0;         


        TString VolName;
        tree->GetEvent(iEvent);
        treeBg->GetEvent(iEvent);
        // tree -> GetEntry(iEvent);
        Int_t Npoints = pointArray -> GetEntries();
        Int_t NpointsBg = pointArrayBg -> GetEntries();
        //std::cout<<" Event Number : "<<iEvent<<std::endl;
	
        std::vector<Float_t> PadCenterCoord;

	Double_t P0[3];
	Double_t P1[3];
	Double_t P2[3];
        Int_t pointsbg[4] = {0,0,0,0};
        TGraph2D * gr1 = new TGraph2D();
        TGraph2D * gr2 = new TGraph2D();
        TGraph * grphi1 = new TGraph();
        TGraph * grphi2 = new TGraph();
        
        TGraph2D * gr1bg = new TGraph2D();
        TGraph * grphi1bg = new TGraph();
        TGraph2D * gr2bg = new TGraph2D();
        TGraph * grphi2bg = new TGraph();
        TGraph2D * gr3bg = new TGraph2D();
        TGraph * grphi3bg = new TGraph();
        TGraph2D * gr4bg = new TGraph2D();
        TGraph * grphi4bg = new TGraph();



/*
        for(Int_t i=0; i<Npoints; i++) {

            point = (AtTpcPoint*) pointArray -> At(i);
            VolName=point->GetVolName();
            //std::cout<<" Volume Name : "<<VolName<<std::endl;
             
            Int_t trackID = point -> GetTrackID();

             //std::cout<<iEvent<<"  "<<trackID<<"  "<<i<<std::endl;

            if(iEvent%2!=0 && trackID==1 && point->GetZIn()*10 == 1000 && VolName=="drift_volume"){  //beam-like
                        xbeam = point->GetXIn()*10; //in mm
			ybeam = point->GetYIn()*10; //in mm
			rbeam = sqrt(xbeam*xbeam + ybeam*ybeam);
	                //std::cout<<iEvent<<"  "<<point->GetZIn()*10<<"  "<<rbeam<<std::endl;
                }

		if(iEvent%2!=0 && trackID!=1 && VolName=="drift_volume"){  //solo los protones
			xpos = point->GetXIn()*10; //in mm
			ypos = point->GetYIn()*10; //in mm
			rpos = sqrt(xpos*xpos + ypos*ypos);
			zpos = point->GetZIn()*10; //in mm
			tTime= point->GetTime()/1000; //us
			
			
			tracks_z_r->Fill(zpos, rpos, point -> GetEnergyLoss()*1e6);
			tracks_x_y->Fill(xpos, ypos, point -> GetEnergyLoss()*1e6);

			

			thetalab = fabs(atan2(rpos - rpos_old, zpos - zpos_old))*180/TMath::Pi();
			philab = atan2(ypos - ypos_old, xpos - xpos_old);
			if(philab<0) philab+=2*TMath::Pi();
			philab = philab*180/TMath::Pi();
			angle_r->Fill(rpos, thetalab, point -> GetEnergyLoss()*1e6);
			angle_eloss->Fill(thetalab, point -> GetEnergyLoss()*1e6);
			rpos_old = rpos;
			zpos_old = zpos;
			xpos_old = xpos;
			ypos_old = ypos;

			if(i==0) real_vertex = zpos;

			if(trackID==2){
				np1++;
				//energyLoss_p1 +=  point -> GetEnergyLoss()*1e3;// in MeV				
				energyLoss_p1 += d2heana->EnergyFluctuation(point -> GetEnergyLoss()*1e3);// in MeV     
				range_p1 = point -> GetLength()*10; //mm
				
				if(np1==2){
					avetheta_p1 = point->GetAIni();
					avephi_p1 = philab;
					}
				
				
				eloss_track->Fill(range_p1,point -> GetEnergyLoss()*1e6);
				//cout<<philab<<endl;
				if(rpos>holeradius){
				
					if(!track1_flag){
						 zpos1_hi = point->GetZIn()*10; //in mm
						 xpos1_hi = point->GetXIn()*10; //in mm
						 ypos1_hi = point->GetYIn()*10; //in mm
                                                 //range1_low = sqrt( sq(xpos1_hi) + sq(ypos1_hi) + sq(zpos1_hi - real_vertex));
						 track1_flag = true;

					}
					range1_hi = sqrt( sq(xpos1_hi - xpos) + sq(ypos1_hi - ypos) + sq(zpos1_hi - zpos));                                        
					//eLoss_p1_hi +=  point -> GetEnergyLoss()*1e3;// in MeV
                                        eLoss_p1_hi += d2heana->EnergyFluctuation(point -> GetEnergyLoss()*1e3);// in MeV     ;
					//range1_low = sqrt( sq(xpos1_hi) + sq(ypos1_hi) + sq(zpos1_hi - real_vertex));
					//cout <<energyLoss_p1<<"  "<<range_p1<<"  "<<point -> GetEnergyLoss()*1e3<<"  "<<range1_hi<<endl;
				}


				av_dE_p1 += point -> GetEnergyLoss()*1e6;
				realkinE = point->GetEIni();
				
				//cout<<np1<<" "<<av_dE_p1/np1<<" "<<energyLoss_p1<<" "<<point->GetEIni()<<endl;

			}
			
			if(trackID==3){
				np2++;
				//energyLoss_p2 +=  point -> GetEnergyLoss()*1e3;// in MeV			
				energyLoss_p2 += d2heana->EnergyFluctuation(point -> GetEnergyLoss()*1e3);// in MeV
				range_p2 = point -> GetLength()*10; //mm
				
				if(np2==2){
					avetheta_p2 = point->GetAIni();
					avephi_p2 = philab;
					}
				//cout<<point ->GetEIni() <<"  "<<energyLoss_p2<<endl;
				av_dE_p2 += point -> GetEnergyLoss()*1e6;
                                //cout<<np2-1<<" "<<thetalab<<" "<<philab<<" "<<point->GetAIni()<<endl;

				if(rpos>holeradius){
				
					if(!track2_flag){
						 zpos2_hi = point->GetZIn()*10; //in mm
						 xpos2_hi = point->GetXIn()*10; //in mm
						 ypos2_hi = point->GetYIn()*10; //in mm
                                                 //range2_low = sqrt( sq(xpos2_hi) + sq(ypos2_hi) + sq(zpos2_hi - real_vertex));                        
						 track2_flag = true;

					}
					range2_hi = sqrt( sq(xpos2_hi - xpos) + sq(ypos2_hi - ypos) + sq(zpos2_hi - zpos));
					eLoss_p2_hi +=  d2heana->EnergyFluctuation(point -> GetEnergyLoss()*1e3);// in MeV 
					//range2_low = sqrt( sq(xpos2_hi) + sq(ypos2_hi) + sq(zpos2_hi - real_vertex));
					//cout <<energyLoss_p1<<"  "<<range_p1<<"  "<<eLoss_p1_hi<<"  "<<range1_hi<<endl;
				}
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
                       
                       if(rpos>holeradius){
                       for(int elec =0; elec<100; elec++){
                                trans	       = gRandom -> Gaus(0,sigstrtrans); //in mm
                                angulo         = gRandom->Uniform(0, TMath::TwoPi());
                                propX           = xpos + trans*TMath::Cos(angulo);
                                propY           = ypos + trans*TMath::Sin(angulo);
				random_lenght   = gRandom -> Gaus(0,sigstrlong);
                                driftLength_fold= driftLength + random_lenght; //mm
                                driftTime       = samplingtime*(floor((((driftLength_fold/10)/driftVelocity) +(tTime))/samplingtime) + 0.5); //us
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
		             			//gr1->SetPoint(counter1,PadCenterCoord[0],propZ,PadCenterCoord[1]);
                                    		//if(iEvent==3) cout<<PadCenterCoord[0]<<" "<<propZ<<" "<<PadCenterCoord[1]<<endl;
                                                counter1++;
                                                }
                                        if(trackID==3 ){
                                                meanX2 += PadCenterCoord[0];
                                                meanY2 += PadCenterCoord[1];
                                                meanZ2 += propZ;
                                                lastZ2  = propZ;                                                               
                                                lastX2  = PadCenterCoord[0];                                        
						//gr2->SetPoint(counter2,PadCenterCoord[0],propZ,PadCenterCoord[1]);                                                                         
						//gr2->SetPoint(counter2,xpos,zpos,ypos);                                                                         
						//if(iEvent==45) cout<<counter2<<"  "<<driftLength_fold<<" "<<propZ<<" "<<zpos<<endl;
                                                counter2++;
                                                }                                        
                                             
                                }

                                
                             }
                            }//if r>10

                        if(trackID==2  && counter1>0 ){
                                 
                                 gr1->SetPoint(points1,meanX1/counter1,meanZ1/counter1,meanY1/counter1);
                                 grphi1->SetPoint(points1,meanX1/counter1,meanY1/counter1);
                                 if(points1==0){ P0[0] = meanX1/counter1; P0[1] = meanY1/counter1; P0[2] = meanZ1/counter1;}
				 P1[0] = meanX1/counter1; P1[1] = meanY1/counter1; P1[2] = meanZ1/counter1;
                                 //if(iEvent==73)cout<<points1-1<<"  "<<meanX1/counter1<<"  "<<meanZ1/counter1<<"  "<<meanY1/counter1<<endl;
                                  points1++;
                                       }

                        if(trackID==3 && counter2>0){
                                 gr2->SetPoint(points2,meanX2/counter2,meanZ2/counter2,meanY2/counter2);
				 //gr2->SetPoint(points2,xpos,zpos,ypos);                                                                         
                                 grphi2->SetPoint(points2,meanX2/counter2,meanY2/counter2);
                                 P2[0] = meanX2/counter2; P2[1] = meanY2/counter2; P2[2] = meanZ2/counter2;
				 //if(iEvent==73)cout<<points2-1<<"  "<<meanX2/counter2<<"  "<<meanZ2/counter2<<"  "<<meanY2/counter2<<endl;
                                 points2++;
                                       }

		   }// only protons

		 
                 
        	}//N number of points (track)
*/


                //---------Bg tracks
             for(Int_t i=0; i<NpointsBg; i++) {

                pointBg = (AtTpcPoint*) pointArrayBg -> At(i);
                VolName=pointBg->GetVolName();
                //std::cout<<" Volume Name : "<<VolName<<std::endl;
             
                Int_t trackIDBg = pointBg -> GetTrackID();

                //std::cout<<iEvent<<"  "<<trackIDBg<<"  "<<i<<std::endl;
        
                if(iEvent%2!=0 && VolName=="drift_volume"){  //solo los protones

                        //std::cout<<iEvent<<"  "<<trackIDBg<<"  "<<i<<std::endl;
                        xpos = pointBg->GetXIn()*10; //in mm
			ypos = pointBg->GetYIn()*10; //in mm
			rpos = sqrt(xpos*xpos + ypos*ypos);
			zpos = pointBg->GetZIn()*10; //in mm
			
                        //if(i==0) real_vertex = zpos;

                        if(trackIDBg==1){
				npbg1++;				
				energyLoss_pbg1 += d2heana->EnergyFluctuation(pointBg -> GetEnergyLoss()*1e3);// in MeV     
				av_dE_pbg1 += pointBg -> GetEnergyLoss()*1e6;
                             
				if(rpos>holeradius){				
					if(!trackbg1_flag){
						 zposbg1_hi = pointBg->GetZIn()*10; //in mm
						 xposbg1_hi = pointBg->GetXIn()*10; //in mm
						 yposbg1_hi = pointBg->GetYIn()*10; //in mm
                                                 //rangebg1_lo = sqrt( sq(xposbg1_hi) + sq(yposbg1_hi) + sq(zposbg1_hi - real_vertex));                        
						 trackbg1_flag = true;
					}
					rangebg1_hi = sqrt( sq(xposbg1_hi - xpos) + sq(yposbg1_hi - ypos) + sq(zposbg1_hi - zpos));
					eLoss_pbg1_hi +=  pointBg -> GetEnergyLoss()*1e3;// in MeV
				        }
			              }//if track 1
                                

                        if(trackIDBg==2){
				npbg2++;				
				energyLoss_pbg2 += d2heana->EnergyFluctuation(pointBg -> GetEnergyLoss()*1e3);// in MeV     
				av_dE_pbg2 += pointBg -> GetEnergyLoss()*1e6;
                             
				if(rpos>holeradius){				
					if(!trackbg2_flag){
						 zposbg2_hi = pointBg->GetZIn()*10; //in mm
						 xposbg2_hi = pointBg->GetXIn()*10; //in mm
						 yposbg2_hi = pointBg->GetYIn()*10; //in mm
                                                 //rangebg2_lo = sqrt( sq(xposbg2_hi) + sq(yposbg2_hi) + sq(zposbg2_hi - real_vertex));                        
						 trackbg2_flag = true;
					}
					rangebg2_hi = sqrt( sq(xposbg1_hi - xpos) + sq(yposbg1_hi - ypos) + sq(zposbg1_hi - zpos));
					eLoss_pbg2_hi +=  d2heana->EnergyFluctuation(pointBg -> GetEnergyLoss()*1e3);// in MeV     
				        }
			              }//if track 2

                        if(trackIDBg==3){
				npbg3++;				
				energyLoss_pbg3 += d2heana->EnergyFluctuation(pointBg -> GetEnergyLoss()*1e3);// in MeV     
				av_dE_pbg3 += pointBg -> GetEnergyLoss()*1e6;
                             
				if(rpos>holeradius){				
					if(!trackbg3_flag){
						 zposbg3_hi = pointBg->GetZIn()*10; //in mm
						 xposbg3_hi = pointBg->GetXIn()*10; //in mm
						 yposbg3_hi = pointBg->GetYIn()*10; //in mm
                                                 //rangebg3_lo = sqrt( sq(xposbg3_hi) + sq(yposbg3_hi) + sq(zposbg3_hi - real_vertex));                        
						 trackbg3_flag = true;
					}
					rangebg3_hi = sqrt( sq(xposbg1_hi - xpos) + sq(yposbg1_hi - ypos) + sq(zposbg1_hi - zpos));
					eLoss_pbg3_hi +=  d2heana->EnergyFluctuation(pointBg -> GetEnergyLoss()*1e3);// in MeV     
				        }
			              }//if track 3


                        if(trackIDBg==4){
				npbg4++;				
				energyLoss_pbg4 += d2heana->EnergyFluctuation(pointBg -> GetEnergyLoss()*1e3);// in MeV     
				av_dE_pbg4 += pointBg -> GetEnergyLoss()*1e6;
                             
				if(rpos>holeradius){				
					if(!trackbg4_flag){
						 zposbg4_hi = pointBg->GetZIn()*10; //in mm
						 xposbg4_hi = pointBg->GetXIn()*10; //in mm
						 yposbg4_hi = pointBg->GetYIn()*10; //in mm
                                                 //rangebg4_lo = sqrt( sq(xposbg4_hi) + sq(yposbg4_hi) + sq(zposbg4_hi - real_vertex));                        
						 trackbg4_flag = true;
					}
					rangebg4_hi = sqrt( sq(xposbg1_hi - xpos) + sq(yposbg1_hi - ypos) + sq(zposbg1_hi - zpos));
					eLoss_pbg4_hi +=  d2heana->EnergyFluctuation(pointBg -> GetEnergyLoss()*1e3);// in MeV     
				        }
			              }//if track 4


                        driftLength       = fabs(zpos-zMesh); //mm
                        sigstrtrans       = D_coef_trans* sqrt(driftLength);//transverse diffusion coefficient
                        sigstrlong        = D_coef_long* sqrt(driftLength);//longitudal diffusion coefficient
			

                        Double_t meanXbg[4] = {0,0,0,0};
                        Double_t meanYbg[4] = {0,0,0,0};
                        Double_t meanZbg[4] = {0,0,0,0};
                        Int_t Counter[4] = {0,0,0,0};
                        


                       if(rpos>holeradius){
                       for(int elec =0; elec<100; elec++){
                                trans	       = gRandom -> Gaus(0,sigstrtrans); //in mm
                                angulo         = gRandom->Uniform(0, TMath::TwoPi());
                                propX           = xpos + trans*TMath::Cos(angulo);
                                propY           = ypos + trans*TMath::Sin(angulo);
				random_lenght   = gRandom -> Gaus(0,sigstrlong);
                                driftLength_fold= driftLength + random_lenght; //mm
                                driftTime       = samplingtime*( TMath::Floor( ( (driftLength_fold/10)/driftVelocity )/samplingtime) + 0.5); //us
                                propZ           = zMesh - driftVelocity*driftTime*10; //mm  
                                pBin            = fPadPlane->Fill(propX,propY,10);
                                padNumber       = pBin-1;
                                
                                if((padNumber<10240 && padNumber>0)  ){
                                        PadCenterCoord = map->CalcPadCenter(padNumber);   

                                        if(trackIDBg==1 ){
                                                meanXbg[0] += PadCenterCoord[0];
                                                meanYbg[0] += PadCenterCoord[1];
                                                meanZbg[0] += propZ;
                                                lastZbg[0]  = propZ;
                                                lastXbg[0]  = PadCenterCoord[0];
                                                Counter[0]++;
                                                }
                                        if(trackIDBg==2 ){
                                                meanXbg[1] += PadCenterCoord[0];
                                                meanYbg[1] += PadCenterCoord[1];
                                                meanZbg[1] += propZ;
                                                lastZbg[1]  = propZ;
                                                lastXbg[1]  = PadCenterCoord[0];
                                                Counter[1]++;
                                                }
                                        if(trackIDBg==3 ){
                                                meanXbg[2] += PadCenterCoord[0];
                                                meanYbg[2] += PadCenterCoord[1];
                                                meanZbg[2] += propZ;
                                                lastZbg[2]  = propZ;
                                                lastXbg[2]  = PadCenterCoord[0];
                                                Counter[2]++;
                                                }
                                        if(trackIDBg==4 ){
                                                meanXbg[3] += PadCenterCoord[0];
                                                meanYbg[3] += PadCenterCoord[1];
                                                meanZbg[3] += propZ;
                                                lastZbg[3]  = propZ;
                                                lastXbg[3]  = PadCenterCoord[0];
                                                Counter[3]++;
                                                }
                                             
                                }

                                
                             }
                            }//if r>10


                        if(trackIDBg==1  && Counter[0]>0 ){                                 
                                 gr1bg->SetPoint(pointsbg[0],meanXbg[0]/Counter[0],meanZbg[0]/Counter[0],meanYbg[0]/Counter[0]);
                                 grphi1bg->SetPoint(pointsbg[0],meanXbg[0]/Counter[0],meanYbg[0]/Counter[0]);
                                 if(pointsbg[0]==0){ Pi1bg[0] = meanXbg[0]/Counter[0]; Pi1bg[1] = meanYbg[0]/Counter[0]; Pi1bg[2] = meanZbg[0]/Counter[0];}
				 Pf1bg[0] = meanXbg[0]/Counter[0]; Pf1bg[1] = meanYbg[0]/Counter[0]; Pf1bg[2] = meanZbg[0]/Counter[0];
                                 pointsbg[0]++;
                                       }
                        if(trackIDBg==2  && Counter[1]>0 ){                                 
                                 gr2bg->SetPoint(pointsbg[1],meanXbg[1]/Counter[1],meanZbg[1]/Counter[1],meanYbg[1]/Counter[1]);
                                 grphi2bg->SetPoint(pointsbg[1],meanXbg[1]/Counter[1],meanYbg[1]/Counter[1]);
                                 if(pointsbg[1]==0){ Pi2bg[0] = meanXbg[1]/Counter[1]; Pi2bg[1] = meanYbg[1]/Counter[1]; Pi2bg[2] = meanZbg[1]/Counter[1];}
				 Pf2bg[0] = meanXbg[1]/Counter[1]; Pf2bg[1] = meanYbg[1]/Counter[1]; Pf2bg[2] = meanZbg[1]/Counter[1];
                                 pointsbg[1]++;
                                       }
                        if(trackIDBg==3  && Counter[2]>0 ){                                 
                                 gr3bg->SetPoint(pointsbg[2],meanXbg[2]/Counter[2],meanZbg[2]/Counter[2],meanYbg[2]/Counter[2]);
                                 grphi3bg->SetPoint(pointsbg[2],meanXbg[2]/Counter[2],meanYbg[2]/Counter[2]);
                                 if(pointsbg[2]==0){ Pi3bg[0] = meanXbg[2]/Counter[2]; Pi3bg[1] = meanYbg[2]/Counter[2]; Pi3bg[2] = meanZbg[2]/Counter[2];}
				 Pf3bg[0] = meanXbg[2]/Counter[2]; Pf3bg[1] = meanYbg[2]/Counter[2]; Pf3bg[2] = meanZbg[2]/Counter[2];
                                 pointsbg[2]++;
                                       }
                        if(trackIDBg==4  && Counter[3]>0 ){                                 
                                 gr4bg->SetPoint(pointsbg[3],meanXbg[3]/Counter[3],meanZbg[3]/Counter[3],meanYbg[3]/Counter[3]);
                                 grphi4bg->SetPoint(pointsbg[3],meanXbg[3]/Counter[3],meanYbg[3]/Counter[3]);
                                 if(pointsbg[3]==0){ Pi4bg[0] = meanXbg[3]/Counter[3]; Pi4bg[1] = meanYbg[3]/Counter[3]; Pi4bg[2] = meanZbg[3]/Counter[3];}
				 Pf4bg[0] = meanXbg[3]/Counter[3]; Pf4bg[1] = meanYbg[3]/Counter[3]; Pf4bg[2] = meanZbg[3]/Counter[3];
                                 pointsbg[3]++;
                                       }

                        


                        }// if only proton events

                }//for each all points of all tracks Bg



                //---------------------------------------
                //if( (holeradius < rbeam) && (holeradius!=0) ) continue;  // no coincidences with heavy particle
                //---------------------------------------

                av_dE_p1 = av_dE_p1/np1;
		av_dE_p2 = av_dE_p2/np2;
                av_dE_pbg1 = av_dE_pbg1/npbg1;
                av_dE_pbg2 = av_dE_pbg2/npbg2;
                av_dE_pbg3 = av_dE_pbg3/npbg3;
                av_dE_pbg4 = av_dE_pbg4/npbg4;
		ereal_eloss->Fill(realkinE, energyLoss_p1);
		ereal_thres->Fill(realkinE, av_dE_p1);

                //cout<<np1<<" "<<av_dE_p1<<" "<<realkinE<<endl; 
                //-----------Fitting tracks
                //----------------------------------------------------------------------------
                if(iEvent%2!=0){
                   //track 1
                if(points1>0){	
		   double guess_x1_slope = (P1[0]-P0[0])/(P1[1]-P0[1]);
		   if(fabs(guess_x1_slope)>20 || (P1[1]-P0[1])==0) guess_x1_slope = 0.2;
		   double guess_z1_slope = (P1[2]-P0[2])/(P1[1]-P0[1]);
		   if(fabs(guess_z1_slope)>20 || (P1[1]-P0[1])==0) guess_z1_slope = 0.2;
                   //cout<<res1->Value(0)<<" "<<res1->Value(1)<<" "<<phi_p1_fit<<" "<<theta_p1_fit<<endl;
                  std::vector<Double_t> vectorp1 (6);
                  vectorp1 = {0.2,guess_x1_slope,P0[2],guess_z1_slope,lastX1, lastZ1};

                   d2heana->FitTrack( gr1 , grphi1, &vectorp1 );
                   theta_p1_fit = d2heana->GetThetaFit();
                   phi_p1_fit = d2heana->GetPhiFit();
                   vertex_p1_fit = d2heana->GetVertexFit();
                   goodfit_p1 = d2heana->GetFitStatus();
                   range1_low = sqrt( sq(xpos1_hi) + sq(ypos1_hi) + sq(zpos1_hi - vertex_p1_fit));
                  //cout<<theta_p1_fit<<" "<<phi_p1_fit<<" "<<vertex_p1_fit<<" "<<goodfit_p1<<endl;
                 }

                 //track 2
                 if(points2>0){	
   		   // set the function and the initial parameter values
		   double guess_x2_slope = (P2[0]-P0[0])/(P2[1]-P0[1]);
		   if(fabs(guess_x2_slope)>20 || (P2[1]-P0[1])==0) guess_x2_slope = 0.2;
		   double guess_z2_slope = (P2[2]-P0[2])/(P2[1]-P0[1]);
		   if(fabs(guess_z2_slope)>20 || (P2[1]-P0[1])==0) guess_z2_slope = 0.2;
		
                   std::vector<Double_t> vectorp2 (6);
                   vectorp2 = {0.2,guess_x2_slope,P0[2],guess_z2_slope,lastX2, lastZ2};

                   d2heana->FitTrack( gr2 , grphi2, &vectorp2 );
                   theta_p2_fit = d2heana->GetThetaFit();
                   phi_p2_fit = d2heana->GetPhiFit();
                   vertex_p2_fit = d2heana->GetVertexFit();
                   goodfit_p2 = d2heana->GetFitStatus();
                   range2_low = sqrt( sq(xpos2_hi) + sq(ypos2_hi) + sq(zpos2_hi - vertex_p2_fit));                        
                   //cout<<theta_p2_fit<<" "<<phi_p2_fit<<" "<<vertex_p2_fit<<" "<<goodfit_p2<<endl;
                  }

                    //track 1bg
                  if(pointsbg[0]>0){	
   		   // set the function and the initial parameter values
		   double guess_x1bg_slope = (Pf1bg[0]-Pi1bg[0])/(Pf1bg[1]-Pi1bg[1]);
		   if(fabs(guess_x1bg_slope)>20 || (Pf1bg[1]-Pi1bg[1])==0) guess_x1bg_slope = 0.2;
		   double guess_z1bg_slope = (Pf1bg[2]-Pi1bg[2])/(Pf1bg[1]-Pi1bg[1]);
		   if(fabs(guess_z1bg_slope)>20 || (Pf1bg[1]-Pi1bg[1])==0) guess_z1bg_slope = 0.2;
                   std::vector<Double_t> vectorp1bg (6);
                   vectorp1bg = {0.2,guess_x1bg_slope,Pi1bg[2],guess_z1bg_slope,lastXbg[0], lastZbg[0]};
                   d2heana->FitTrack( gr1bg , grphi1bg, &vectorp1bg );
                   theta_p1bg_fit = d2heana->GetThetaFit();
                   phi_p1bg_fit = d2heana->GetPhiFit();
                   vertex_p1bg_fit = d2heana->GetVertexFit();
                   goodfit_p1bg = d2heana->GetFitStatus();
                   rangebg1_lo = sqrt( sq(xposbg1_hi) + sq(yposbg1_hi) + sq(zposbg1_hi - vertex_p1bg_fit));                        
                   //cout<<theta_p1bg_fit<<" "<<phi_p1bg_fit<<" "<<vertex_p1bg_fit<<" "<<goodfit_p1bg<<endl;
                  }

                   //track 2bg
                 if(pointsbg[1]>0){	
   		   // set the function and the initial parameter values
		   double guess_x2bg_slope = (Pf2bg[0]-Pi2bg[0])/(Pf2bg[1]-Pi2bg[1]);
		   if(fabs(guess_x2bg_slope)>20 || (Pf2bg[1]-Pi2bg[1])==0) guess_x2bg_slope = 0.2;
		   double guess_z2bg_slope = (Pf2bg[2]-Pi2bg[2])/(Pf2bg[1]-Pi2bg[1]);
		   if(fabs(guess_z2bg_slope)>20 || (Pf2bg[1]-Pi2bg[1])==0) guess_z2bg_slope = 0.2;
                   std::vector<Double_t> vectorp2bg (6);
                   vectorp2bg = {0.2,guess_x2bg_slope,Pi2bg[2],guess_z2bg_slope,lastXbg[1], lastZbg[1]};
                   d2heana->FitTrack( gr2bg , grphi2bg, &vectorp2bg );
                   theta_p2bg_fit = d2heana->GetThetaFit();
                   phi_p2bg_fit = d2heana->GetPhiFit();
                   vertex_p2bg_fit = d2heana->GetVertexFit();
                   goodfit_p2bg = d2heana->GetFitStatus();
                   rangebg2_lo = sqrt( sq(xposbg2_hi) + sq(yposbg2_hi) + sq(zposbg2_hi - vertex_p2bg_fit));                        
                   //cout<<theta_p2bg_fit<<" "<<phi_p2bg_fit<<" "<<vertex_p2bg_fit<<" "<<goodfit_p2bg<<endl;
                  }
              


                   //track 3bg
                  if(pointsbg[2]>0){	
   		   // set the function and the initial parameter values
		   double guess_x3bg_slope = (Pf3bg[0]-Pi3bg[0])/(Pf3bg[1]-Pi3bg[1]);
		   if(fabs(guess_x3bg_slope)>20 || (Pf3bg[1]-Pi3bg[1])==0) guess_x3bg_slope = 0.2;
		   double guess_z3bg_slope = (Pf3bg[2]-Pi3bg[2])/(Pf3bg[1]-Pi3bg[1]);
		   if(fabs(guess_z3bg_slope)>20 || (Pf3bg[1]-Pi3bg[1])==0) guess_z3bg_slope = 0.2;
                   std::vector<Double_t> vectorp3bg (6);
                   vectorp3bg = {0.2,guess_x3bg_slope,Pi3bg[2],guess_z3bg_slope,lastXbg[2], lastZbg[2]};
                   d2heana->FitTrack( gr3bg , grphi3bg, &vectorp3bg );
                   theta_p3bg_fit = d2heana->GetThetaFit();
                   phi_p3bg_fit = d2heana->GetPhiFit();
                   vertex_p3bg_fit = d2heana->GetVertexFit();
                   goodfit_p3bg = d2heana->GetFitStatus();
                   rangebg3_lo = sqrt( sq(xposbg3_hi) + sq(yposbg3_hi) + sq(zposbg3_hi - vertex_p3bg_fit));                        
                   //cout<<theta_p3bg_fit<<" "<<phi_p3bg_fit<<" "<<vertex_p3bg_fit<<" "<<goodfit_p3bg<<endl;
                  }


                 //track 4bg
                  if(pointsbg[3]>0){	
   		   // set the function and the initial parameter values
		   double guess_x4bg_slope = (Pf4bg[0]-Pi4bg[0])/(Pf4bg[1]-Pi4bg[1]);
		   if(fabs(guess_x4bg_slope)>20 || (Pf4bg[1]-Pi4bg[1])==0) guess_x4bg_slope = 0.2;
		   double guess_z4bg_slope = (Pf4bg[2]-Pi4bg[2])/(Pf4bg[1]-Pi4bg[1]);
		   if(fabs(guess_z4bg_slope)>20 || (Pf4bg[1]-Pi4bg[1])==0) guess_z4bg_slope = 0.2;
                   std::vector<Double_t> vectorp4bg (6);
                   vectorp4bg = {0.2,guess_x4bg_slope,Pi4bg[2],guess_z4bg_slope,lastXbg[3], lastZbg[3]};
                   d2heana->FitTrack( gr4bg , grphi4bg, &vectorp4bg );
                   theta_p4bg_fit = d2heana->GetThetaFit();
                   phi_p4bg_fit = d2heana->GetPhiFit();
                   vertex_p4bg_fit = d2heana->GetVertexFit();
                   goodfit_p4bg = d2heana->GetFitStatus();
                   rangebg4_lo = sqrt( sq(xposbg4_hi) + sq(yposbg4_hi) + sq(zposbg4_hi - vertex_p4bg_fit));                        
                   //cout<<theta_p4bg_fit<<" "<<phi_p4bg_fit<<" "<<vertex_p4bg_fit<<" "<<goodfit_p4bg<<endl;
                  }



                 //find the 2 tracks with same vertex
                 std::vector<Double_t> Vertex(6);
                 Double_t Eloss_Hi[6] = {eLoss_p1_hi,eLoss_p2_hi,eLoss_pbg1_hi,eLoss_pbg2_hi,eLoss_pbg3_hi,eLoss_pbg4_hi};
                 Double_t RangeLow[6] = {range1_low,range2_low,rangebg1_lo,rangebg2_lo,rangebg3_lo,rangebg4_lo};
                 Double_t RangeHi[6] = {range1_hi,range2_hi,rangebg1_hi,rangebg2_hi,rangebg3_hi,rangebg4_hi};
                 Double_t ThetaL[6] = {theta_p1_fit,theta_p2_fit,theta_p1bg_fit,theta_p2bg_fit,theta_p3bg_fit,theta_p4bg_fit};
                 Double_t PhiL[6] = {phi_p1_fit,phi_p2_fit,phi_p1bg_fit,phi_p2bg_fit,phi_p3bg_fit,phi_p4bg_fit};
                 Double_t Av_Eloss[6] = {av_dE_p1,av_dE_p2,av_dE_pbg1,av_dE_pbg2,av_dE_pbg3,av_dE_pbg4};
                 bool GoodFit[6] = {goodfit_p1,goodfit_p2,goodfit_p1bg,goodfit_p2bg,goodfit_p3bg,goodfit_p4bg};

               

                 
                  //---------------------salvar el roottree
                 Double_t Eloss_HiT[4] = {eLoss_pbg1_hi,eLoss_pbg2_hi,eLoss_pbg3_hi,eLoss_pbg4_hi};
                 Double_t RangeLowT[4] = {rangebg1_lo,rangebg2_lo,rangebg3_lo,rangebg4_lo};
                 Double_t RangeHiT[4] = {rangebg1_hi,rangebg2_hi,rangebg3_hi,rangebg4_hi};
                 Double_t ThetaLT[4] = {theta_p1bg_fit,theta_p2bg_fit,theta_p3bg_fit,theta_p4bg_fit};
                 Double_t PhiLT[4] = {phi_p1bg_fit,phi_p2bg_fit,phi_p3bg_fit,phi_p4bg_fit};
                 Double_t Av_ElossT[4] = {av_dE_pbg1,av_dE_pbg2,av_dE_pbg3,av_dE_pbg4};
                 bool GoodFitT[4] = {goodfit_p1bg,goodfit_p2bg,goodfit_p3bg,goodfit_p4bg};
                Double_t VertexT[4] = {vertex_p1bg_fit, vertex_p2bg_fit, vertex_p3bg_fit, vertex_p4bg_fit};

                for(int w1 = 0; w1<4; w1++){
                         elosshibg = Eloss_HiT[w1];
                         rangelowbg =  RangeLowT[w1] ;
                         rangehibg =  RangeHiT[w1];
                         thetabg =  ThetaLT[w1];
                         phibg =  PhiLT[w1];
                         av_elossbg = Av_ElossT[w1];
                         goodfitbg =  GoodFitT[w1]; 
                         vertexbg =  VertexT[w1]; 
                         contaBg = 2*(iEvent - 1) + w1;
                         rootTree->Fill();
                }
                
                //-----------------------------

                 //Vertex.at(0) = vertex_p1_fit;
                 //Vertex.at(1) = vertex_p2_fit;
                 Vertex.at(0) = -1000; //solo para remover este track
                 Vertex.at(1) = -1000; //solo para remover este track
                 Vertex.at(2) = vertex_p1bg_fit;
                 Vertex.at(3) = vertex_p2bg_fit;
                 Vertex.at(4) = vertex_p3bg_fit;
                 Vertex.at(5) = vertex_p4bg_fit;
                 int* stracks = Selecttracks(&Vertex);
                 int track_i = *(stracks+0);
                 int track_k = *(stracks+1);
                 //int track_i = 0;
                 //int track_k = 1;
                 //cout<<*(stracks+0)<<"  "<<*(stracks+1)<<endl;


                   theta_res->Fill(avetheta_p1,theta_p1_fit);
		   theta_res->Fill(avetheta_p2,theta_p2_fit);

                bool goodfit = false;
                if(GoodFit[track_i]==true && GoodFit[track_k]==true &&  (fabs(Vertex.at(track_i) - Vertex.at(track_k))< 20)  ) goodfit = true;
                 

                if(goodfit==true && (Av_Eloss[track_i] >threseloss && Av_Eloss[track_k] > threseloss) ){
		
		
		
                if(geofile == "_03atm"){
                        eLoss_p1_reco = Eloss_Hi[track_i] + (Eloss_Hi[track_i]/ RangeHi[track_i])*RangeLow[track_i];
		        eLoss_p2_reco = Eloss_Hi[track_k] + (Eloss_Hi[track_k]/RangeHi[track_k])*RangeLow[track_k];
                }
                else{                        
                        eLoss_p1_reco = Eloss_Hi[track_i] + GetEloss(Eloss_Hi[track_i], RangeHi[track_i],RangeLow[track_i], X, Y);
		        eLoss_p2_reco = Eloss_Hi[track_k] + GetEloss(Eloss_Hi[track_k], RangeHi[track_k],RangeLow[track_k], X, Y);
		}
		
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

		

                d2heana->kine_2b(proj_mass, target_mass,  he2_mass_ex,  recoil_mass, Ekin_proj, theta_He2*TMath::DegToRad(), kin_He2);

		
        
		theta_cm = d2heana->GetThetaCM();
        	Ex4 = d2heana->GetMissingMass();
		
		thetacm_he2_res->Fill(theta_cm);
		Ex_res_res->Fill(Ex4);
		thetacm_Ex_2he_res->Fill(theta_cm,Ex4);
	
		vertex_reco->Fill(real_vertex,0.5*(Vertex.at(track_i) + Vertex.at(track_k)) );
		ener_reco->Fill(energyLoss_p1, eLoss_p1_reco);
		ener_reco->Fill(energyLoss_p2, eLoss_p2_reco);
		ener_reco_ratio->Fill(energyLoss_p1, (energyLoss_p1-eLoss_p1_reco)/energyLoss_p1);
		ener_reco_ratio->Fill(energyLoss_p2, (energyLoss_p2-eLoss_p2_reco)/energyLoss_p2);

		
		
			

                } //succesful fit for the two tracks

                  

                } //fitting tracks

                //----------------------------------------------------------------------------
               
               

		
		if(av_dE_p1 >threseloss && av_dE_p2 > threseloss && iEvent%2!=0){

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
		
                //cout<<energyLoss_p1<<"  "<<energyLoss_p2<<endl;

		// reconstruction of 2He
		mom1_norm = TMath::Sqrt(energyLoss_p1*energyLoss_p1 + 2.0*energyLoss_p1*proton_mass);
		mom_proton1.SetX(mom1_norm*TMath::Sin(avetheta_p1*TMath::Pi()/180)*TMath::Cos(avephi_p1*TMath::Pi()/180));
		mom_proton1.SetY(mom1_norm*TMath::Sin(avetheta_p1*TMath::Pi()/180)*TMath::Sin(avephi_p1*TMath::Pi()/180));
		mom_proton1.SetZ(mom1_norm*TMath::Cos(avetheta_p1*TMath::Pi()/180));

		mom2_norm = TMath::Sqrt(energyLoss_p2*energyLoss_p2 + 2.0*energyLoss_p2*proton_mass);
		mom_proton2.SetX(mom2_norm*TMath::Sin(avetheta_p2*TMath::Pi()/180)*TMath::Cos(avephi_p2*TMath::Pi()/180));
		mom_proton2.SetY(mom2_norm*TMath::Sin(avetheta_p2*TMath::Pi()/180)*TMath::Sin(avephi_p2*TMath::Pi()/180));
		mom_proton2.SetZ(mom2_norm*TMath::Cos(avetheta_p2*TMath::Pi()/180));

		

		//if(fabs(avephi_p1-avephi_p2)>100){
		//if(energyLoss_p1>0.1 && energyLoss_p2>0.1){
		mom_He2 = mom_proton1 + mom_proton2;

		E_tot_2he = (proton_mass + energyLoss_p1) + (proton_mass + energyLoss_p2);
		he2_mass_ex = TMath::Sqrt(E_tot_2he*E_tot_2he - mom_He2.Mag2());
		ex_he2->Fill(he2_mass_ex - he2_mass);

		kin_He2 = TMath::Sqrt(mom_He2.Mag2() + he2_mass_ex*he2_mass_ex) -he2_mass_ex;
		//kin_He2 = energyLoss_p1 + energyLoss_p2;
		theta_He2 = mom_He2.Theta()*180/TMath::Pi();
		phi_He2 = mom_He2.Phi()*180/TMath::Pi();
		theta_r_he2->Fill(theta_He2);
		phi_r_he2->Fill(phi_He2);
		kin_r_he2->Fill(kin_He2);
		theta_kin_2he->Fill(theta_He2, kin_He2);


                //cout<<theta_He2<<" "<<mom1_norm<<" "<<mom2_norm<<" "<<avetheta_p1<<" "<<avetheta_p2<<" "<<avephi_p1<<" "<<avephi_p2<<endl;		

                d2heana->kine_2b(proj_mass, target_mass,  he2_mass_ex,  recoil_mass, Ekin_proj, theta_He2*TMath::DegToRad(), kin_He2);

		
        
		theta_cm = d2heana->GetThetaCM();
        	Ex4 = d2heana->GetMissingMass();
		
		thetacm_he2->Fill(theta_cm);
		Ex_res->Fill(Ex4);
		thetacm_Ex_2he->Fill(theta_cm,Ex4);

		
	        }//if cut high energy particles
                
                delete gr1;
                delete gr2;
                delete grphi1;
                delete grphi2;
                delete gr1bg;
                delete grphi1bg;
                 delete gr2bg;
                delete grphi2bg;
                delete gr3bg;
                delete grphi3bg;
                delete gr4bg;
                delete grphi4bg;
                
              
	
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
	ereal_eloss->Write();
	ereal_thres->Write();

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
	theta_res->Write();
	ener_reco->Write();
	ener_reco_ratio->Write();


        rootTree->Write();
	outfile->Close();


	cout<<"*************************************"<<endl;
	cout<<endl;	
    	final=clock();
	Ttotal=(final-inicio)/(double) CLOCKS_PER_SEC;
	cout<<"tiempo de ejecucion: "<<Ttotal<<" segundos"<<endl;

}

