#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"

#include <iostream>
#include <fstream>


void fission_ana(Int_t num_ev=10000)
{


  TH2D *Eloss_vs_Range_Sca = new TH2D("Eloss_vs_Range_Sca","ELoss_vs_Range_Sca",1000,0,2000,1000,0,500);
  Eloss_vs_Range_Sca->SetMarkerStyle(20);
  Eloss_vs_Range_Sca->SetMarkerSize(0.5);

  TH2D *Eloss_vs_Range_Rec = new TH2D("Eloss_vs_Range_Rec","ELoss_vs_Range_Rec",1000,0,2000,1000,0,500);
  Eloss_vs_Range_Rec->SetMarkerStyle(20);
  Eloss_vs_Range_Rec->SetMarkerSize(0.5);
  Eloss_vs_Range_Rec->SetMarkerColor(2);

  TH2D *tracks = new TH2D("tracks","tracks",1000,-100,1000,1000,-300,300);


  TH1D *ELossRatio = new TH1D("ElossRatio","ELossRatio",1000,0,1000);

  TH1D *ICELoss = new TH1D("ICELoss","ICELoss",1000,0,100);

  TH1D *rad = new TH1D("rad","rad",2000,0,2000);

  TH2D *HKineRecoil =  new TH2D("HKineRecoil","HKineRecoil",1000,0,180,1000,0,200);
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




  TCanvas *c1 = new TCanvas();
  c1->Divide(2,2);
  c1->Draw();

  /*TCanvas *c2 = new TCanvas();
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
  c6->Divide(1,3);
  c6-> Draw();

  TCanvas *c7 = new TCanvas();
  c7-> Draw();*/


  TString mcFileNameHead = "data/attpcsim_2";
  TString mcFileNameTail = ".root";
  TString mcFileName     = mcFileNameHead + mcFileNameTail;
  std:cout << " Analysis of simulation file  " << mcFileName << endl;

  AtTpcPoint* point = new AtTpcPoint();
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
              Int_t n2=0;
              Int_t nrad=0;

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
                            range_sca = point -> GetLength()*10; //mm
                            EnergySca = point ->GetEIni();
                            energyLoss_sca+=( point -> GetEnergyLoss() )*1000;//MeV
                            //AngleSca= point->GetAIni();
                            // std::cout<<" Track ID : "<<trackID<<std::endl;
                            // std::cout<<" Range_sca : "<<range_sca<<std::endl;
                            // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
                        }//TrackID == 1

                        if(trackID==2 && VolName=="drift_volume"){ //RECOIL
                            range_rec = point -> GetLength()*10; //mm
                            energyLoss_rec+=( point -> GetEnergyLoss() )*1000;//MeV
                            EnergyRecoil= point->GetEIni();
                            AngleRecoil= point->GetAIni();
                            //zpos=point->GetZ()*10;
                            //xpos=point->GetXIn()*10;
                      }

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
                                              //Vertex_vs_Angle_Rec->Fill(vertex,range_rec,AngleRecoil);


                                                  /*  for(Int_t i=0;i<10;i++){

                                                        if(vertex>0.0+i*100 && vertex<100.0*(i+1)){
                                                         AngRange[i]->Fill(AngleRecoil,range_rec);
                                                         //std::cout<<radmean<<std::endl;
                                                         RadAng[i]->Fill(AngleRecoil,radmean);
                                                        //RadAng[i]->Fill(180-thetamean*180/TMath::Pi(),radmean);
                                                      }*/
                                      }// Reaction event


                }// Points loop



            }// Points Loop


              c1->cd(1);
              Eloss_vs_Range_Sca->Draw("scat");
              c1->cd(2);
              Eloss_vs_Range_Rec->Draw("scat");
              c1->cd(3);
              ELossRatio->Draw();
              c1->cd(4);
              ICELoss->Draw();

            /*  c2->cd(1);
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
