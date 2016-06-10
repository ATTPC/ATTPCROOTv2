#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"


#include <iostream>
#include <fstream>


void fission_ana(Int_t num_ev=1000)
{


  TH2D *Eloss_vs_Range_Sca = new TH2D("Eloss_vs_Range_Sca","ELoss_vs_Range_Sca",1000,0,900,1000,0,600);
  Eloss_vs_Range_Sca->SetMarkerStyle(20);
  Eloss_vs_Range_Sca->SetMarkerSize(0.1);

  TH2D *Eloss_vs_Range_Rec = new TH2D("Eloss_vs_Range_Rec","ELoss_vs_Range_Rec",1000,0,900,1000,0,600);
  Eloss_vs_Range_Rec->SetMarkerStyle(20);
  Eloss_vs_Range_Rec->SetMarkerSize(0.1);
  Eloss_vs_Range_Rec->SetMarkerColor(2);

  TH2D *tracks = new TH2D("tracks","tracks",1000,-100,1000,1000,-300,300);


  TH1D *ELossRatio = new TH1D("ElossRatio","ELossRatio",1000,0,7);
  TH1D *ICELoss = new TH1D("ICELoss","ICELoss",1000,0,100);
  TH1D *rad = new TH1D("rad","rad",2000,0,2000);
  TH1D *Totalmomentum_dist = new TH1D("Totalmomentum_dist","Total Momentum Distribution", 1000, -2,150);
  TH2D *GasELoss1 = new TH2D("GasELoss1", "GasEloss vs A", 1000, 0, 200, 1000, 0, 1000);
  GasELoss1->SetMarkerStyle(20);
  GasELoss1->SetMarkerSize(0.5);
  GasELoss1->SetMarkerColor(2);
  TH2D *TargetELoss1 = new TH2D("TargetEloss1", "TargetELoss1", 1000, 0, 200, 1000, 0, 1000);
  TH2D *PvZ = new TH2D("PvZ","PvZ", 1000, 0, 150, 1000, -2,15);
  TH2D *RvP = new TH2D("RvP","RvP", 1000, 0, 15, 1000, -2,500);
  TargetELoss1->SetMarkerStyle(20);
  TargetELoss1->SetMarkerSize(0.5);
  TargetELoss1->SetMarkerColor(2);
	TH1D *Momentum_dist1 = new TH1D("Momentum_dist1","Momentum distribution of fragment 1", 1000,-2,200);
	TH1D *Momentum_dist2 = new TH1D("Momentum_dist2","Momentum distribution of fragment 2", 1000,-2,200);
  TH1D *Zdist = new TH1D("Zdist","Zdist", 1000,-2,200);
  TH1D *TotalRange = new TH1D("TotalRange","Range of both fragments", 1000, 0,10);
  TH1D *Range_frag1 = new TH1D("Range_frag1","Range of fragment 1", 1000, 0,300);
  TH1D *Range_frag2 = new TH1D("Range_frag2","Range of fragment 2", 1000, 0,300);


  /*TH2D *HKineRecoil =  new TH2D("HKineRecoil","HKineRecoil",1000,0,180,1000,0,200);
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
  Char_t hra_name[10][256];

         for(Int_t i=0;i<10;i++){
    std::sprintf(hra_name[i],"radang_%d",i);
    RadAng[i]= new TH2D(hra_name[i],hra_name[i],100,0,180,100,0,200);
  }

*/


  TCanvas *c1 = new TCanvas();
  c1->Divide(2,2);
  c1->Draw();

  TCanvas *c2 = new TCanvas();
  c2->Divide(2,2);
  c2->Draw();

  /*TCanvas *c3 = new TCanvas();
  c3->Divide(3,3);
  c3->Draw();

  TCanvas *c4 = new TCanvas();
  c4->Divide(3,3);
  c4->Draw();

  TCanvas *c5 = new TCanvas();
  c5->Divide(2,2);
  c5->Draw();

  TCanvas *c6 = new TCanvas();
  c6->Divide(1,3);
  c6-> Draw();

  TCanvas *c7 = new TCanvas();
  c7-> Draw();*/


  TString mcFileNameHead = "data/attpcsim_2";
  TString mcFileNameTail = ".root";
  TString mcFileName     = mcFileNameHead + mcFileNameTail;
  std:cout << " Analysis of simulation file  " << mcFileName << endl;

  AtTpcPoint* point = new AtTpcPoint();
  AtTpcPoint* point1 = new AtTpcPoint();
  AtTpcPoint* point_forw = new AtTpcPoint();
  AtTpcPoint* point_back = new AtTpcPoint();
  TClonesArray *pointArray=0;
  TFile* file = new TFile(mcFileName.Data(),"READ");
  TTree* tree = (TTree*) file -> Get("cbmsim");

                    tree = (TTree*) file -> Get("cbmsim");
                    //TBranch *branch = tree->GetBranch("AtTpcPoint");
                    tree -> SetBranchAddress("AtTpcPoint", &pointArray);
                    Int_t nEvents = tree -> GetEntriesFast();
                    std::cout<<" Number of events : "<<nEvents<<std::endl;

                  /*  Double_t Ztot = 98; //Pb+S
                    Double_t Atot = 246; //Pb+S
                    Double_t E_beam = .0004; //GeV/u
                    Double_t c=29.972; //c in cm/ns
                    Double_t uma=0.93149; //GeV/c^2
                    Double_t v1_cm[3],v2_cm[3],v1_lab[3],v2_lab[3],beta[3],pos1[3],pos2[3],pos0[3];
                    Double_t Z1,Z2,A1,A2,N1,N2,TKE,tof1,tof2, B;
                    beta[0] = 0;
                    beta[1] = 0;
                    beta[2] = sqrt(1-(1/pow((E_beam/uma+1),2)));
                    Double_t aux = sqrt(pow(beta[0],2)+pow(beta[1],2)+pow(beta[2],2));
                    Double_t gamma = pow((1-aux*aux),-0.5);
*/


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
              Double_t vertex = 0.0;
              Double_t radmean=0.0;
              Double_t thetamean=0.0;
              Double_t theta=0.0;
              Double_t windowEloss_rec=0.0;
              Double_t windowEloss_sca=0.0;
              Double_t counter1 =0.0;
              Double_t counter2 = 0.0;
              Double_t counter3 = 0.0;
              Double_t counter4 = 0.0;
              Double_t pz1, pz2,pz3,pz4;
              Double_t v1_cm[3],v2_cm[3],v1_lab[3],v2_lab[3],beta[3],pos1[3],pos2[3],pos0[3];
              Double_t Z1,Z2,Z3,Z4,A1,A2,A3,A4,N1,N2,TKE,tof1,tof2, vtot_cm, Px, Py, Pz;
              Int_t n2=0;
              Int_t nrad=0;
              Double_t pi = 3.14;
              Double_t c=29.972; //c in cm/ns
              Double_t Ekin = 0;
              Double_t uma=0.93149; //GeV/c^2


              TString VolName;
              tree->GetEvent(iEvent);
              // tree -> GetEntry(iEvent);
              Int_t n = pointArray -> GetEntries();
              std::cout<<" Event Number : "<<iEvent<<std::endl;


              for(Int_t i=0; i<n; i++) {
                  point = (AtTpcPoint*) pointArray -> At(i);
                  VolName=point->GetVolName();
                  //std::cout<<" Volume Name : "<<VolName<<std::endl;
                  Int_t trackID = point -> GetTrackID();

                       if(trackID==0 && VolName=="drift_volume"){
                          vertex=point->GetZ()*10;
                          range_beam=point -> GetLength()*10;//mm
                          EnergyBeam=point -> GetEIni();


                        }



                        if(trackID==1 && VolName=="drift_volume"){

                            //std::cout<<pz1<<std::endl;
                            range_sca = point -> GetLength()*10; //mm
                            EnergySca = point -> GetEIni();
                            energyLoss_sca+=( point -> GetEnergyLoss() )*1000;//MeV
                            if(counter1==0){
                              counter1++;
                              A1 = point -> GetMassNum();
                              Z1 = point -> GetAtomicNum();
                              pz1 = point-> GetPzOut();
                              RvP->Fill(pz1,range_sca);
                              //Momentum_dist1->Fill(pz1);
                              //Totalmomentum_dist->Fill(pz1);
                          }
                            //AngleSca= point->GetAIni();
                            // std::cout<<" Track ID : "<<trackID<<std::endl;
                            // std::cout<<" Range_sca : "<<range_sca<<std::endl;
                            // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
                            /*Px = point -> GetPxOut();
                            Py = point -> GetPyOut();
                            Pz = point -> GetPzOut();

                            v1_lab[0] = Px/(A1*0.0312);
                            v1_lab[1] = Py/(A1*0.0312);
                            v1_lab[2] = Pz/(A1*0.0312);

                            v1_cm[2] = ((beta[2]*c)-v1_lab[2])/((beta[2]*v1_lab[2]/c)-1);
                            v1_cm[0] = v1_lab[0]*(gamma*(1+beta[2]*v1_cm[2]/c));
                            v1_cm[1] = v1_lab[1]*(gamma*(1+beta[2]*v1_cm[2]/c));


                            vtot_cm = pow(pow(v1_cm[0],2)+pow(v1_cm[1],2)+pow(v1_cm[2],2), 0.5);
                            Momentum_dist1->Fill(vtot_cm);



                            Z1 = point -> GetAtomicNum();
                            if(Z1 !=0){
                            //std::cout<<Z1<<std::endl;
                            A1 = point -> GetMassNum();
                            //std::cout<<A1<<std::endl;
                            Z2 = Ztot - Z1;
                            //std::cout<<Z2<<std::endl;
                            A2 = Atot - A1;
                            //std::cout<<A2<<std::endl;

                            //Total Kinetic energy (Wilkins model)(GeV)
                          	 TKE = (0.00144*Z1*Z2)/(1.16*(pow(A1,1./3.)*(1.+2./3.*0.6)+pow(A2,1./3.)*(1.+2./3.*0.6))+2);
                             //TKEJ = TKE*1.6*pow(10, -10);
                             //std::cout<<TKE<<std::endl;

                             //Velocities moduli of the fragments in CM frame (cm/ns)
                             Double_t v11 = sqrt(2*A2*TKE/(uma*A1*(A1+A2)))*c;
                             Double_t v22 = v11*A1/A2;
                             Momentum_dist2-> Fill(v11);

                           }//Z1!=0

*/


                        }//TrackID == 1

                        if(trackID==2 && VolName=="drift_volume"){ //RECOIL

                            range_rec = point -> GetLength()*10; //mm
                            energyLoss_rec+=( point -> GetEnergyLoss() )*1000;//MeV
                            EnergyRecoil= point->GetEIni();
                            AngleRecoil= point->GetAIni();
                            if(counter2==0){
                              counter2++;
                              Z2 = point-> GetAtomicNum();
                              A2 = point-> GetMassNum();
                              pz2 = point-> GetPzOut();
                              RvP->Fill(pz2,range_rec);
                              //Momentum_dist2->Fill(pz2);
                              //Totalmomentum_dist->Fill(pz2);
                          }
                            //zpos=point->GetZ()*10;
                            //xpos=point->GetXIn()*10;

                      }
                        if(trackID==2 && VolName=="tpc_window"){
                        windowEloss_rec+=(point-> GetEnergyLoss())*1000;
                          if(counter3==0){
                            counter3++;
                            A3 = point-> GetMassNum();
                            Z3 = point-> GetAtomicNum();
                            pz3 = point-> GetPzOut();
                            TargetELoss1->Fill(pz4, windowEloss_rec);

                      }}
                      if(trackID==1 && VolName=="tpc_window"){
                      windowEloss_sca+=(point-> GetEnergyLoss())*1000;
                        if(counter4==0){
                          counter4++;
                          A4 = point-> GetMassNum();
                          Z4 = point-> GetAtomicNum();
                          pz4 = point-> GetPzOut();
                          TargetELoss1->Fill(pz4, windowEloss_sca);
                    }}
                                          if(iEvent%2!=0){
                                              //std::cout<<" Range_rec : "<<range_rec<<std::endl;
                                              //std::cout<<" energyLoss_rec : "<<energyLoss_rec<<std::endl;
                                              Eloss_vs_Range_Rec->Fill(range_rec,energyLoss_rec);
                                              //TotalRange->Fill(range_rec);
                                              TotalRange->Fill(range_sca);
                                              Range_frag1->Fill(energyLoss_rec);
                                              Range_frag2->Fill(energyLoss_sca);


                                             // std::cout<<" Range_sca : "<<range_sca<<std::endl;
                                             // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
                                              Eloss_vs_Range_Sca->Fill(range_sca,energyLoss_sca);
                                              ELossRatio->Fill(energyLoss_sca/energyLoss_rec);

                                              //HKineRecoil->Fill(AngleRecoil,EnergyRecoil);
                                              //Angle_sca_vs_Angle_rec->Fill(AngleRecoil,AngleSca);
                                              //Vertex_vs_Angle_Rec->Fill(vertex,range_rec,AngleRecoil);
                                              }
                                                  /*  for(Int_t i=0;i<10;i++){

                                                        if(vertex>0.0+i*100 && vertex<100.0*(i+1)){
                                                         AngRange[i]->Fill(AngleRecoil,range_rec);
                                                         //std::cout<<radmean<<std::endl;
                                                         RadAng[i]->Fill(AngleRecoil,radmean);
                                                        //RadAng[i]->Fill(180-thetamean*180/TMath::Pi(),radmean);
                                                      }*/
                                      //}// Reaction event

                              }//Points Loop
                              //Ekin = pow(pow(pz2*c,2)+pow(A2*0.931*pow(c,2),2),0.5)/1000;
                              //Total Kinetic energy (Wilkins model)(GeV)
                                TKE = (0.00144*Z1*Z2)/(1.16*(pow(A1,1./3.)*(1.+2./3.*0.6)+pow(A2,1./3.)*(1.+2./3.*0.6))+2);
                            	  Double_t v11 = sqrt(2*A2*TKE/(uma*A1*(A1+A2)))*c; //cm/ns
                                Double_t v22 = v11*A1/A2;
                                Ekin = 0.5*A2*pow(v22,2)*.931; //should be GeV


                              GasELoss1->Fill(A1, energyLoss_sca);
                              Momentum_dist1-> Fill(A1);
                              Momentum_dist1->Fill(A2);
                              Zdist->Fill(Z1);
                              Zdist->Fill(Z2);
                              PvZ->Fill(Z1, pz1);
                              std::cout<<A3<<" "<<Z3<<" "<<pz3<<" "<<windowEloss_rec<<std::endl;
                              //GasELoss1->Fill(A2, energyLoss_rec);






            /*c2->cd(1);
            TargetELoss1->Draw();
            c2->cd(2);
            TotalRange->Draw();
            c2->cd(3);
            Range_frag1->Draw();
            c2->cd(4);
            Range_frag2->Draw();
*/

              c1->cd(1);
              Momentum_dist1->Draw();
              //Eloss_vs_Range_Sca->Draw("scat");
              c1->cd(2);
              Zdist->Draw();
              //Momentum_dist2->Draw();
              //Eloss_vs_Range_Rec->Draw("scat");
              c1->cd(3);
              //PvZ->Draw();
              TargetELoss1->Draw();
              c1->cd(4);
              //GasELoss1->Draw();
              RvP->Draw();


             c2->cd(1);
              TargetELoss1->Draw();

            /*for(Int_t i=0;i<10;i++){
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

              c7->cd();
              Vertex_vs_Angle_Rec-> GetXaxis() -> SetTitle("Vertex Position [mm]");
              Vertex_vs_Angle_Rec-> GetYaxis() -> SetTitle("Recoil Range [mm]");
              Vertex_vs_Angle_Rec-> GetZaxis() -> SetTitle("Recoil Angle [deg]");
              Vertex_vs_Angle_Rec->Draw("zcol");

                c6->cd(1);
                Range_vs_Energy_Sca->Draw();
                c6->cd(2);
                Range_vs_Energy_Rec->Draw();
                c6->cd(3);
                Range_vs_Energy_Beam->Draw();*/


}// Events loop
}
