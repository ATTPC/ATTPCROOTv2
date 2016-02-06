#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TH2.h"
#include "TH1.h"
#include "TH3.h"
#include "TClonesArray.h"
#include "TCanvas.h"
#include "TMath.h"
//#include </home/ayyadlim/fair_install_ROOT6/ATTPCROOT/tpc/AtTpcPoint.h>

#include <iostream>
#include <fstream>


void sim_ana(Int_t num_ev=10000,TString reactionName="dummy")
{

    TH2D *Eloss_vs_Range_Sca = new TH2D("Eloss_vs_Range_Sca","ELoss_vs_Range_Sca",100,0,1000,300,0,300);
    Eloss_vs_Range_Sca->SetMarkerStyle(20);
    Eloss_vs_Range_Sca->SetMarkerSize(0.5);

    TH2D *Eloss_vs_Range_Rec = new TH2D("Eloss_vs_Range_Rec","ELoss_vs_Range_Rec",1000,0,2000,1000,0,10);
    Eloss_vs_Range_Rec->SetMarkerStyle(20);
    Eloss_vs_Range_Rec->SetMarkerSize(0.5);
    Eloss_vs_Range_Rec->SetMarkerColor(2);

    TH2D *tracks = new TH2D("tracks","tracks",1000,-100,1000,1000,-300,300);


    TH1D *ELossRatio = new TH1D("ElossRatio","ELossRatio",1000,0,1000);

    TH1D *ICELoss = new TH1D("ICELoss","ICELoss",1000,0,100);

    TH1D *rad = new TH1D("rad","rad",2000,0,2000);

    TH2D *HKineRecoil =  new TH2D("HKineRecoil","HKineRecoil",1000,0,180,1000,0,200);
    TH2D *HKineSca =  new TH2D("HKineSca","HKineSca",1000,0,180,1000,0,200);
    TH2D *AngRange[10];
    Char_t hh_name[10][256];

           for(Int_t i=0;i<10;i++){
			std::sprintf(hh_name[i],"angrange_%d",i);
			AngRange[i]= new TH2D(hh_name[i],hh_name[i],100,0,180,100,0,2000);
		}

    TH2D *RadAng[10];
    TH2D *Range_vs_Energy_Sca = new TH2D("Range_vs_Energy_Sca","Range_vs_Energy_Sca",200,0,1000,500,0,50);
    TH2D *Range_vs_Energy_Rec = new TH2D("Range_vs_Energy_Rec","Range_vs_Energy_Rec",200,0,1000,500,0,50);
    TH2D *Range_vs_Energy_Beam = new TH2D("Range_vs_Energy_Beam","Range_vs_Energy_Beam",200,0,1000,500,0,50);

    TH3D *Vertex_vs_Angle_Rec = new TH3D("Vertex_vs_Angle_Rec","Vertex_vs_Angle_Rec",100,0,500,100,0,500,100,0,180);

    TH2D *Angle_sca_vs_Angle_rec = new TH2D("Angle_sca_vs_Angle_rec","Angle_sca_vs_Angle_rec",200,0,180,200,0,180);

    TH2D *Rad_rec_vs_Vertex = new TH2D("Rad_rec_vs_Vertex","Rad_rec_vs_Vertex",200,0,250,200,0,500);

    Char_t hra_name[10][256];

           for(Int_t i=0;i<10;i++){
			std::sprintf(hra_name[i],"radang_%d",i);
			RadAng[i]= new TH2D(hra_name[i],hra_name[i],100,0,180,100,0,200);
		}




    TCanvas *c1 = new TCanvas();
    c1->Divide(2,2);
    c1->Draw();

    TCanvas *c2 = new TCanvas();
    c2->Divide(2,2);
    c2->Draw();

    TCanvas *c3 = new TCanvas();
    c3->Divide(3,3);
    c3->Draw();

    TCanvas *c4 = new TCanvas();
    c4->Divide(3,3);
    c4->Draw();

    TCanvas *c5 = new TCanvas();
    c5->Divide(2,2);
    c5->Draw();

    TCanvas *c6 = new TCanvas();
    c6->Divide(2,3);
    c6-> Draw();

    /*TCanvas *c7 = new TCanvas();
    c7-> Draw();*/

    TCanvas *c8 = new TCanvas();
    c8->Divide(2,2);
    c8-> Draw();

    TString mcFileNameHead = "data/attpcsim";
    TString mcFileNameTail = ".root";
    TString mcFileName     = mcFileNameHead + mcFileNameTail;
    TString outFileNameHead = "data/";
    TString outFileNameTail = ".root";
    TString outFileName     = outFileNameHead + reactionName + outFileNameTail;

    Double_t RecAng,RecEner,RecRange,RecELoss;
    Double_t ScaAng,ScaEner,ScaRange,ScaELoss;
    Double_t RecRad, ScaRad,VertexPos,BeamEner,BeamRange;

    TTree *treeOut = new TTree("treeOut","ATTPCSimTree");
    treeOut->Branch("RecAng",&RecAng,"RecAng/D");
    treeOut->Branch("RecEner",&RecEner,"RecEner/D");
    treeOut->Branch("RecRange",&RecRange,"RecRange/D");
    treeOut->Branch("RecELoss",&RecELoss,"RecELoss/D");
    treeOut->Branch("RecRad",&RecRad,"RecRad/D");
    treeOut->Branch("ScaAng",&ScaAng,"ScaAng/D");
    treeOut->Branch("ScaEner",&ScaEner,"ScaEner/D");
    treeOut->Branch("ScaRange",&ScaRange,"ScaRange/D");
    treeOut->Branch("ScaELoss",&ScaELoss,"ScaELoss/D");
    treeOut->Branch("ScaRad",&ScaRad,"ScaRad/D");
    treeOut->Branch("VertexPos",&VertexPos,"VertexPos/D");
    treeOut->Branch("BeamEner",&BeamEner,"BeamEner/D");
    treeOut->Branch("BeamRange",&BeamRange,"BeamRange/D");
    //treeOut->Branch("",&zdtree,"zdtree/F");

    std::cout << " Analysis of simulation file  " << mcFileName << std::endl;
    std::cout << " Output file  " << outFileName << std::endl;
    if(reactionName.Contains("dummy")) std::cout << " Warning! Reaction Name has not been changed! Output file has been overwritten!  "  << std::endl;

    AtTpcPoint* point = new AtTpcPoint();
    AtTpcPoint* point_forw = new AtTpcPoint();
    AtTpcPoint* point_back = new AtTpcPoint();
    TClonesArray *pointArray=0;

    TFile* outfile = new TFile(outFileName.Data(),"RECREATE");
    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");


    tree = (TTree*) file -> Get("cbmsim");
    //TBranch *branch = tree->GetBranch("AtTpcPoint");
    tree -> SetBranchAddress("AtTpcPoint", &pointArray);
    Int_t nEvents = tree -> GetEntriesFast();




    Double_t vertex =0.0;

    if(nEvents>num_ev) nEvents=num_ev;

    for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {
        Double_t energyLoss_sca=0.0;
        Double_t range_sca=0.0;
        Double_t energyLoss_rec=0.0;
        Double_t range_rec=0.0;
	Double_t range_beam=0.0;
        Double_t BeamEnergyLoss_IC=0.0;
        Double_t EnergyRecoil = 0.0;
	Double_t EnergyBeam=0.0;
        Double_t EnergySca=0.0;
        Double_t AngleRecoil = 0.0;
        Double_t AngleSca = 0.0;
      	Double_t zpos = 0.0;
      	Double_t xpos = 0.0;
        Double_t radius=0.0;
      	Double_t radmean=0.0;
	Double_t thetamean=0.0;
      	Double_t theta=0.0;
        Double_t rad_rec=0.0;
        Double_t rad_sca=0.0;
        Double_t BeamEnergyLoss=0.0;
        Int_t n2=0;
      	Int_t nrad=0;

        TString VolName;
        tree->GetEvent(iEvent);
        // tree -> GetEntry(iEvent);
        Int_t n = pointArray -> GetEntries();
        std::cout<<" Event Number : "<<iEvent<<std::endl;
	      rad->Reset();


        for(Int_t i=0; i<n; i++) {

            point = (AtTpcPoint*) pointArray -> At(i);
            VolName=point->GetVolName();
            //std::cout<<" Volume Name : "<<VolName<<std::endl;
            Int_t trackID = point -> GetTrackID();

	   // if(i==0){
	//	zpos=point->GetZ();
               // std::cout<<" Z pos : "<<zpos<<std::endl;
		//std::cout<<" Track ID : "<<trackID<<std::endl;
	 //   }
            //std::cout<<" Track ID : "<<trackID<<std::endl;
            //std::cout<<" Point number : "<<i<<std::endl;

            //BEAM INFORMATION
            if(trackID==0 && VolName.Contains("IC_")){

                BeamEnergyLoss_IC+=( point -> GetEnergyLoss() )*1000;//MeV



            }

	          if(trackID==0 && VolName=="drift_volume"){
                vertex=point->GetZ()*10;
			         	range_beam=point -> GetLength()*10;//mm
			          EnergyBeam=point -> GetEIni();



                BeamEnergyLoss+=( point -> GetEnergyLoss() )*1000;//MeV


               // std::cout<<" Vertex : "<<vertex<<std::endl;

            }


            //RECOIL INFORMATION
            if(trackID==2 && VolName=="drift_volume"){ //RECOIL
		            n2++;
                range_rec = point -> GetLength()*10; //mm
                RecRange = range_rec;
                energyLoss_rec+=( point -> GetEnergyLoss() )*1000;//MeV
                EnergyRecoil= point->GetEIni();
                RecEner = EnergyRecoil;
                AngleRecoil= point->GetAIni();
                RecAng = AngleRecoil;
		            zpos=point->GetZ()*10;
		            xpos=point->GetXIn()*10;
                rad_rec = TMath::Sqrt( TMath::Power(point->GetXIn()*10,2) + TMath::Power(point->GetYIn()*10,2)  );
                RecRad = rad_rec;
		            //if(AngleRecoil>65 && AngleRecoil<70 && vertex>200.0 && vertex<210.0 && zpos>200.0 ){
			          tracks->Fill(zpos,xpos);

		             //}



			//Radius of curvature calculation
				//if(i<(n-2)){
				if(i<5){
				 nrad++;
				 point_forw = (AtTpcPoint*) pointArray -> At(i+1);
				 point_back = (AtTpcPoint*) pointArray -> At(i+2);
				 Double_t x1 = point->GetXIn()*10;
         Double_t x2 = point_forw->GetXIn()*10;
         Double_t x3 = point_back->GetXIn()*10;
         Double_t y1 = point->GetYIn()*10;
         Double_t y2 = point_forw->GetYIn()*10;
         Double_t y3 = point_back->GetYIn()*10;
				 Double_t z2 = point_forw->GetZIn()*10;


				 Double_t dl2 = pow((x1 - x3),2) + pow((y1 - y3),2);
           		         Double_t dr1 = pow( x2 - (x3+x1)/2.0 ,2);
                                 Double_t dr2 = pow( y2 - (y3+y1)/2.0 ,2);
                                 Double_t dr = TMath::Sqrt(dr1+dr2)*8.0;
                                 radius = dl2/dr;
            			 Double_t dlt = TMath::Sqrt( pow(x1-x2,2) + pow(y1-y2,2)  );
            			 Double_t dy = y2 - y1;
            			 Double_t dx = x2 - x1;

				 theta = TMath::ATan2(dlt,zpos-z2);
				 //std::cout<<zpos-z2<<std::endl;
                                 //Double_t phi = TMath::ATan2(dy,dx);
				/*std::cout<<" ======================= "<<std::endl;
				std::cout<<" Point : "<<i<<" of "<<n<<std::endl;
				std::cout<<180-theta*180/TMath::Pi()<<std::endl;
			        std::cout<<AngleRecoil<<std::endl;*/
			        rad->SetBinContent(i+1,radius);
			        radmean+=radius;
			        thetamean+=theta;
				//std::cout<<" Point : "<<i<<" radius : "<<radius<<std::endl;
				}

				//}





                //std::cout<<point->GetEIni()<<std::endl;
                //std::cout<<" Track ID : "<<trackID<<std::endl;

                //std::cout<<" Point number : "<<i<<std::endl;
                //std::cout<<" Event Number : "<<iEvent<<std::endl;
                //std::cout<<" Range_rec : "<<range_rec<<std::endl;
                //std::cout<<" energyLoss_rec : "<<energyLoss_rec<<std::endl;

            }//TrackID == 2

            //SCATTERED INFORMATION
            if(trackID==1 && VolName=="drift_volume"){
                range_sca = point -> GetLength()*10; //mm
                ScaRange = range_sca;
                EnergySca = point ->GetEIni();
                ScaEner = EnergySca;
                energyLoss_sca+=( point -> GetEnergyLoss() )*1000;//MeV
                AngleSca= point->GetAIni();
                ScaAng = AngleSca;
                rad_sca = TMath::Sqrt( TMath::Power(point->GetXIn()*10,2) + TMath::Power(point->GetYIn()*10,2)  );
                ScaRad = rad_sca;
                // std::cout<<" Track ID : "<<trackID<<std::endl;
                // std::cout<<" Range_sca : "<<range_sca<<std::endl;
                // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;

            }//TrackID == 1


        }//n number of points

	//radmean=rad->GetMean();

      	radmean=radmean/nrad;
        thetamean=thetamean/nrad;
       // std::cout<<" Theta mean : "<<thetamean*180/TMath::Pi()<<std::endl;
      //  std::cout<<" Theta sim : "<<AngleRecoil<<std::endl;


	//std::cout<<" Mean radius of curvature : "<<radmean<<std::endl;
	//std::cout<<" Mean radius of curvature (hist) : "<<rad->GetMean()<<std::endl;

    //SEPARATION BETWEEN BEAM and REACTION EVENTS
    if(iEvent%2!=0){
            //std::cout<<" Range_rec : "<<range_rec<<std::endl;
            //std::cout<<" energyLoss_rec : "<<energyLoss_rec<<std::endl;
            Eloss_vs_Range_Rec->Fill(range_rec,energyLoss_rec);


           // std::cout<<" Range_sca : "<<range_sca<<std::endl;
           // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
            Eloss_vs_Range_Sca->Fill(range_sca,energyLoss_sca);
            ELossRatio->Fill(energyLoss_sca/energyLoss_rec);

            HKineRecoil->Fill(AngleRecoil,EnergyRecoil);
            Angle_sca_vs_Angle_rec->Fill(AngleRecoil,AngleSca);
            Vertex_vs_Angle_Rec->Fill(vertex,range_rec,AngleRecoil);
            Range_vs_Energy_Sca->Fill(range_sca,EnergySca);
            Range_vs_Energy_Rec->Fill(range_rec,EnergyRecoil);
            HKineRecoil->Fill(AngleRecoil,EnergyRecoil);
            HKineSca->Fill(AngleSca,EnergySca);

            //std::cout<<" Recoil Energy : "<<EnergyRecoil/4.0<<" Recoil Angle : "<<AngleRecoil<<std::endl;
          //  std::cout<<" Scatter Energy : "<<EnergySca/8.0<<" Scatter Angle : "<<AngleSca<<std::endl;


    		for(Int_t i=0;i<10;i++){

             if(vertex>0.0+i*100 && vertex<100.0*(i+1)){
    				 AngRange[i]->Fill(AngleRecoil,range_rec);
    				 //std::cout<<radmean<<std::endl;
    				 RadAng[i]->Fill(AngleRecoil,radmean);
    				//RadAng[i]->Fill(180-thetamean*180/TMath::Pi(),radmean);
    			}
    		}


        treeOut->Fill(); // We only fill the information on the Scattering Event

  }else if(iEvent%2==0){
     ICELoss->Fill(BeamEnergyLoss_IC);
     BeamEner=BeamEnergyLoss;
     BeamRange=vertex;
     Range_vs_Energy_Beam->Fill(vertex,BeamEnergyLoss);
      VertexPos=vertex;

     //std::cout<<" Beam Energy Loss : "<<17.0-BeamEnergyLoss<<" Vertex : "<<vertex<<std::endl;
   }


     Rad_rec_vs_Vertex->Fill(rad_rec,vertex);





    }//number of events

    //c4->cd(1);
    //rad->Draw();

    c1->cd(1);
    Eloss_vs_Range_Sca->Draw("scat");
    c1->cd(2);
    Eloss_vs_Range_Rec->Draw("scat");
    c1->cd(3);
    ELossRatio->Draw();
    c1->cd(4);
    ICELoss->Draw();

    c2->cd(1);
    HKineRecoil->Draw("scat");
    c2->cd(2);
    tracks->Draw("col");

	for(Int_t i=0;i<10;i++){
             c3->cd(i+1);
	     AngRange[i]->Draw("col");
	}

	for(Int_t i=0;i<10;i++){
             c4->cd(i+1);
	     RadAng[i]->Draw("col");
	}

      c5->cd(1);
      //rad->Draw();
    Angle_sca_vs_Angle_rec->Draw("zcol");

    /*c7->cd();
    Vertex_vs_Angle_Rec-> GetXaxis() -> SetTitle("Vertex Position [mm]");
    Vertex_vs_Angle_Rec-> GetYaxis() -> SetTitle("Recoil Range [mm]");
    Vertex_vs_Angle_Rec-> GetZaxis() -> SetTitle("Recoil Angle [deg]");
    Vertex_vs_Angle_Rec->Draw("zcol");*/

      c6->cd(1);
      Range_vs_Energy_Sca->Draw();
      c6->cd(2);
      Range_vs_Energy_Rec->Draw();
      c6->cd(3);
      HKineRecoil->Draw();
      c6->cd(4);
      HKineSca->Draw();
      c6->cd(5);
      Range_vs_Energy_Beam->Draw();

      c8->cd(1);
     Rad_rec_vs_Vertex->Draw();


      outfile->cd();
      treeOut->Write();
      outfile->Close();



}
