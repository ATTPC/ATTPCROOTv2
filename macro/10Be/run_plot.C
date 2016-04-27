#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TH2.h"
#include "TH1.h"
#include "TH3.h"
#include "TClonesArray.h"
#include "TCanvas.h"
#include "TMath.h"
#include "TClonesArray.h"
#include "TStyle.h"
#include "TTreeReader.h"
#include "TChain.h"
#include "TFileCollection.h"
#include "TError.h"
#include "TMinuit.h"

#include "../../include/ATEvent.hh"
#include "../../include/ATHit.hh"
#include "../../include/ATProtoEvent.hh"
#include "../../include/ATAnalysis.hh"
#include "../../include/ATHoughSpaceLine.hh"
#include "../../include/ATHoughSpaceCircle.hh"

#include <ios>
#include <iostream>
#include <istream>
#include <limits>

void run_plot(TString FileNameHead = "output_proto_ana_10Be")
{

  TStopwatch timer;
  timer.Start();
  TString workdir = getenv("VMCWORKDIR");
  TString FilePath = workdir + "/macro/10Be/";
  TString FileNameTail = ".root";
  TString FileName     = FilePath + FileNameHead + FileNameTail;
  TString FileNameHeadU = "output_proto_10Be";
  TString FileNameU = FilePath + FileNameHeadU + FileNameTail;
  std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
  TFile* file = new TFile(FileName.Data(),"READ");
  TFile* fileu = new TFile(FileNameU.Data(),"READ");
  //TFile* file = new TFile(FileNameHead_chain.Data(),"READ");
  TTree* tree = (TTree*) file -> Get("cbmsim");
  Int_t nEvents = tree -> GetEntriesFast();
  std::cout<<" Number of events : "<<nEvents<<std::endl;

  TObjArray PlotList(0);
  TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
  c2->Divide(1,1);
  TCanvas *c4 = new TCanvas("c4","c4",200,10,700,700);
  c4->Divide(2,2);
  TCanvas *c5 = new TCanvas("c5","c5",200,10,700,700);
  c5->Divide(4,2);
  TCanvas *c6 = new TCanvas("c6","c6",200,10,700,700);
  c6->Divide(2,2);
  TCanvas *c7 = new TCanvas("c7","c7",200,10,700,700);
  c7->Divide(2,1);
  TCanvas *c8 = new TCanvas("c8","c8",200,10,700,700);
  c8->Divide(2,1);
  TCanvas *c9 = new TCanvas("c9","c9",200,10,700,700);
  c9->Divide(1,1);


  TH2D* Q02_Kine = new TH2D("Kinematics","Kinematics",360,0,180,360,0,180);
  Q02_Kine->SetMarkerColor(2);
  Q02_Kine->SetMarkerStyle(20);
  Q02_Kine->SetMarkerSize(0.7);
  TH2D* Q13_Kine = new TH2D("Q13_Kine","Q13_Kine",250,0,180,250,0,180);
  Q13_Kine->SetMarkerColor(2);
  Q13_Kine->SetMarkerStyle(20);
  Q13_Kine->SetMarkerSize(0.7);

  PlotList.Add(Q02_Kine);
  PlotList.Add(Q13_Kine);

  TH2D* Kine_cut= new TH2D("Kine_cut","Kine_cut",250,0,180,250,0,180);

  PlotList.Add(Kine_cut);

  TH1D* Diff02 = new TH1D("Diff02","Diff02",600,0,600);
  TH1D* Diff13 = new TH1D("Diff13","Diff13",600,0,600);

  PlotList.Add(Diff02);
  PlotList.Add(Diff13);

  TH2D* RangeRange = new TH2D("Range","Range",100,0,1000,100,0,1000);

  PlotList.Add(RangeRange);

  TH1D* RangeDist = new TH1D("RangeDist","RangeDist",20000,0,20000);

  PlotList.Add(RangeDist);

  TH2D* HoughFitAngle[4];
  for(Int_t i=0;i<4;i++){
    HoughFitAngle[i] = new TH2D(Form("HoughFit[%i]",i),Form("HoughFit[%i]",i),1000,0,180,1000,0,180);
    HoughFitAngle[i]->SetMarkerColor(2);
    HoughFitAngle[i]->SetMarkerStyle(20);
    HoughFitAngle[i]->SetMarkerSize(0.7);
    PlotList.Add(HoughFitAngle[i]);
  }



  TH2D* Range_hist[4];
  for(Int_t i=0;i<4;i++) Range_hist[i] = new TH2D(Form("Range[%i]",i),Form("Range[%i]",i),100,0,180,100,0,1000);

  TH1D* HoughSlope[4];
  for(Int_t i=0;i<4;i++) HoughSlope[i] = new TH1D(Form("Slope[%i]",i),Form("Slope[%i]",i),100,-10,10);

  TH1D* HoughInt[4];
  for(Int_t i=0;i<4;i++) HoughInt[i] = new TH1D(Form("Intercept[%i]",i),Form("Intercept[%i]",i),750,0,750);

  for(Int_t i=0;i<4;i++){
    PlotList.Add(Range_hist[i]);
    PlotList.Add(HoughSlope[i]);
    PlotList.Add(HoughInt[i]);
  }

  TH2D* Excitation_EL = new TH2D("Elastic", "Elastic",36,0,180,125,0,10);
  TH2D* Excitation_IN = new TH2D("Inelastic", "Inelastic",36,0,180,125,0,10);

  PlotList.Add(Excitation_EL);
  PlotList.Add(Excitation_IN);

  Double_t *ThetaCMS = new Double_t[20000];
  Double_t *ThetaLabRec = new Double_t[20000];
  Double_t *EnerLabRec = new Double_t[20000];
  Double_t *ThetaLabSca = new Double_t[20000];
  Double_t *EnerLabSca = new Double_t[20000];
  Double_t *ThetaCMSIn = new Double_t[20000];
  Double_t *ThetaLabRecIn = new Double_t[20000];
  Double_t *EnerLabRecIn = new Double_t[20000];
  Double_t *ThetaLabScaIn = new Double_t[20000];
  Double_t *EnerLabScaIn = new Double_t[20000];
  Double_t *LengthMM = new Double_t[600];
  Double_t *EnergyMM = new Double_t[600];

  TString kinfile="../Kinematics/Decay_kinematics/Kine.txt";
  std::ifstream *kineStr = new std::ifstream(kinfile.Data());
  Int_t numKin=0;

    if(!kineStr->fail()){
      while(!kineStr->eof()){
          *kineStr>>ThetaCMS[numKin]>>ThetaLabRec[numKin]>>EnerLabRec[numKin]>>ThetaLabSca[numKin]>>EnerLabSca[numKin];
          numKin++;
      }
    }else if(kineStr->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

  TString kinfileIn = "KineInEl.txt";
  std::ifstream *kineStrIn = new std::ifstream(kinfileIn.Data());
  numKin=0;

    if(!kineStrIn->fail()){
      while(!kineStrIn->eof()){
          *kineStrIn>>ThetaCMSIn[numKin]>>ThetaLabRecIn[numKin]>>EnerLabRecIn[numKin]>>ThetaLabScaIn[numKin]>>EnerLabScaIn[numKin];
          numKin++;
      }
    }else if(kineStrIn->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

  std::ifstream *energyTable = new std::ifstream("energy.dat");
  Int_t numLen=0;
  while(!energyTable->eof()){
    *energyTable>>LengthMM[numLen]>>EnergyMM[numLen];
    numLen++;
  }



  TGraph *Kine_AngRec_AngSca = new TGraph(numKin,ThetaLabRec,ThetaLabSca);
  Kine_AngRec_AngSca->SetLineColor(6);
  TGraph *Kine_AngRec_AngSca_In = new TGraph(numKin,ThetaLabRecIn,ThetaLabScaIn);
  Kine_AngRec_AngSca_In->SetLineColor(7);
  TGraph *Kine_AngRec_AngSca_vert = new TGraph(numKin,ThetaLabSca,ThetaLabRec);
  Kine_AngRec_AngSca_vert->SetLineColor(6);
  TGraph *Kine_AngRec_AngSca_In_vert = new TGraph(numKin,ThetaLabScaIn,ThetaLabRecIn);
  Kine_AngRec_AngSca_In_vert->SetLineColor(7);

  PlotList.Add(Kine_AngRec_AngSca);
  PlotList.Add(Kine_AngRec_AngSca_vert);

  Double_t xa[2] = {0.0,90.0};
  Double_t ya[2] = {90.0,0.0};
  TGraph *alpha = new TGraph(2,xa,ya);

  PlotList.Add(alpha);

  leg = new TLegend(0.1,0.1,0.2,0.2);
  leg->AddEntry(Kine_AngRec_AngSca, "elastic","l");
  leg->AddEntry(Kine_AngRec_AngSca_In, "inelastic","l");
  leg->AddEntry(alpha, "alpha-alpha scattering","l");


  TCutG *anglecut = new TCutG("elastic",8);
  anglecut->SetVarX("Angle0");
  anglecut->SetVarY("Angle2");
  anglecut->SetTitle("Graph");
  anglecut->SetLineColor(2);
  anglecut->SetPoint(0,47.0,25.0);
  anglecut->SetPoint(1,85.0,10.0);
  anglecut->SetPoint(2,82.0,2.0);
  anglecut->SetPoint(3,40.0,20.0);
  anglecut->SetPoint(4,30.0,20.0);
  anglecut->SetPoint(5,10.0,5.0);
  anglecut->SetPoint(6,5.0,5.0);
  anglecut->SetPoint(7,25.0,25.0);
  anglecut->SetPoint(8,47.0,25.0);

  TCutG *diffcut = new TCutG("gooddiff",4);
  diffcut->SetVarX("Angle0");
  diffcut->SetVarY("Angle2");
  diffcut->SetTitle("Graph");
  diffcut->SetLineColor(2);
  diffcut->SetPoint(0,47.0,25.0);
  diffcut->SetPoint(1,85.0,10.0);
  diffcut->SetPoint(2,82.0,2.0);
  diffcut->SetPoint(3,40.0,20.0);
  diffcut->SetPoint(4,47.0,25.0);

  PlotList.Add(anglecut);

/*
  TCutG *backanglecut = new TCutG("CUTG",8);
  backanglecut->SetVarX("Back1");
  backanglecut->SetVarY("Back2");
  backanglecut->SetTitle("Graph2");
  backanglecut->SetPoint(0,0.0,0.0);
  backanglecut->SetPoint(1,6.5,8.3);
  backanglecut->SetPoint(2,11.1,13.4);
  backanglecut->SetPoint(3,17.3,18.7);
  backanglecut->SetPoint(4,20.7,20.7);
  backanglecut->SetPoint(5,17.4,15.6);
  backanglecut->SetPoint(6,11.1,9.1);
  backanglecut->SetPoint(7,6.5,4.9);
  backanglecut->SetPoint(8,0.0,0.0);
*/
/*  TCutG *rangecut = new TCutG("CutG",8);
  rangecut->SetVarX("Range0");
  rangecut->SetVarY("Range2");
  rangecut->SetTitle("Graph1");
  rangecut->SetPoint(0,0.0,0.0);
  rangecut->SetPoint(1,1000.0,1000.0);
  rangecut->SetPoint(2,0.0,1000.0);
  rangecut->SetPoint(3,0.0,0.0);

*/
  TCutG *cut2 = new TCutG("inelastic",4); //INELASTIC
  cut2->SetVarX("Angle0");
  cut2->SetVarY("Angle2");
  cut2->SetTitle("Graph");
  cut2->SetLineColor(3);
  cut2->SetPoint(0,40.0,20.0);
  cut2->SetPoint(1,80.0,0.0);
  cut2->SetPoint(2,10.0,0.0);
  cut2->SetPoint(3,30.0,20.0);
  cut2->SetPoint(4,40.0,20.0);

  PlotList.Add(cut2);

  ofstream filelist;
  filelist.open("list.txt");


  int eventnum=0;

  std::vector<Double_t> *AngleFit;
  std::vector<Double_t> *IntFit;
  std::vector<Double_t> *HoughAngle;
  std::vector<Double_t> *Range;
  std::vector<std::pair<Double_t,Double_t>> *Houghparam;
  Double_t max;
  Double_t start;
  Double_t x1=0;
  Double_t y1=0;
  Double_t x2=0;
  Double_t y2=0;
  Double_t thetalab;
  Double_t thetacm;

  TTree plotvals("plotvals","values for plots of 10Be");
  plotvals.Branch("max", &max, "max/D");
  plotvals.Branch("start", &start, "start/D");
  plotvals.Branch("x1", &x1, "x1/D");
  plotvals.Branch("y1", &y1, "y1/D");
  plotvals.Branch("x2", &x2, "x2/D");
  plotvals.Branch("y2", &y2, "y2/D");
  plotvals.Branch("AngleFit", &AngleFit, "AngleFit/D");
  plotvals.Branch("IntFit", &IntFit, "IntFit/D");
  plotvals.Branch("HoughAngle", &HoughAngle, "HoughAngle/D");
  plotvals.Branch("Range",&Range,"Range/D");
  plotvals.Branch("Houghparam",&Houghparam,"Houghparam/D");
  plotvals.Branch("thetalab",&thetacm,"thetalab/D");
  plotvals.Branch("thetacm",&thetacm,"thetacm/D");


  ofstream myfile;
  myfile.open ("stuff.txt");

  TTreeReader Reader1("cbmsim", file);
  TTreeReaderValue<TClonesArray> analysisArray(Reader1, "ATAnalysis");

  TTreeReader Reader2("cbmsim", fileu);
  TTreeReaderValue<TClonesArray> eventArray(Reader2, "ATEventH");



        		while (Reader1.Next()&&Reader2.Next()) {

              ATProtoAnalysis* analysis = (ATProtoAnalysis*) analysisArray->At(0);
              ATEvent* event = (ATEvent*) eventArray->At(0);
              std::vector<Double_t> *AngleFit = analysis->GetAngleFit();
              std::vector<Double_t> *IntFit = analysis->GetPar0();
              std::vector<Double_t> *HoughAngle = analysis->GetAngle();
              std::vector<Double_t> *Range = analysis->GetRange();
              std::vector<std::pair<Double_t,Double_t>> *Houghparam = analysis->GetHoughPar();
              Float_t *MeshArray = event->GetMesh();

              Double_t Diff02_val = TMath::Abs(Houghparam->at(0).second - Houghparam->at(2).second);
              Double_t Diff13_val = TMath::Abs(Houghparam->at(1).second - Houghparam->at(3).second);
              Diff02->Fill(Diff02_val);
              Diff13->Fill(Diff13_val);

              RangeRange->Fill(Range->at(1),Range->at(3));
              RangeRange->Fill(Range->at(0),Range->at(2));


              Double_t start=0;
              for (Int_t i = 380; i<=385;i++)start+=MeshArray[i];
              Double_t max = TMath::MaxElement(512, MeshArray);
              RangeDist->Fill(start);
              if(1==1){//start>5400&&start<12000
              if(AngleFit->at(2)>AngleFit->at(0)) {
                Q02_Kine->Fill(AngleFit->at(0),AngleFit->at(2));
                x1 = AngleFit->at(2);
                y1 = AngleFit->at(0);
              }
              else{
                Q02_Kine->Fill(AngleFit->at(0),AngleFit->at(2));
                x1 = AngleFit->at(0);
                y1 = AngleFit->at(2);
              }
              if(AngleFit->at(3)>AngleFit->at(1)){
                Q02_Kine->Fill(AngleFit->at(1),AngleFit->at(3));
                x2 = AngleFit->at(3);
                y2 = AngleFit->at(1);
              }
              else{
                Q02_Kine->Fill(AngleFit->at(1),AngleFit->at(3));
                x2 = AngleFit->at(1);
                y2 = AngleFit->at(3);
              }

              if(anglecut->IsInside(x1,y1)){
                Kine_cut->Fill(x1,y1);
              }
              if(anglecut->IsInside(x2,y2)){
                Kine_cut->Fill(x2,y2);
              }

              //if(x1<60&&x1>40)myfile<<eventnum<<endl;
              //if(x2<60&&x2>40)myfile<<eventnum<<endl;
              //if(diffcut->IsInside(x1,y1)) Diff02->Fill(Diff02_val);
              //if(diffcut->IsInside(x2,y2)) Diff13->Fill(Diff13_val);
            }
          /*    if((backanglecut->IsInside(AngleFit->at(0),AngleFit->at(2)))||(backanglecut->IsInside(AngleFit->at(1),AngleFit->at(3)))) {
                Q02_Kine->Fill(AngleFit->at(0),AngleFit->at(2));
                Q02_Kine->Fill(AngleFit->at(1),AngleFit->at(3));
                filelist<<eventnum<<std::endl;
              }*/





                //filelist<<eventnum<<endl;

                for(Int_t i=0;i<4;i++){
                    Double_t thetalab = 0.0;
                    Double_t thetacm = 0.0;
                    //    if (i<2) Excitation->Fill(Houghparam->at(i).second,x1);
                    //    if (i>1) Excitation->Fill(Houghparam->at(i).second,x2);
                    if (i<2) thetalab = x1;
                    if (i>1) thetalab = x2;

                    //if (thetacm<0) thetacm= thetacm+180.0;
                    if((anglecut->IsInside(x1,y1))||(anglecut->IsInside(x2,y2))) {
                      for(Int_t j = 1; j<20000; j++){
                        if(ThetaLabRec[j]<=thetalab){
                          thetacm = ThetaCMS[j];
                          //filelist<<eventnum<<"\t"<<i<<"\t"<<thetalab<<"\t"<<thetacm<<"\t"<<ThetaCMS[j]<<endl;
                          break;
                        }
                      }
                      Excitation_EL->Fill(thetacm,(2.0/7000000.0)*EnergyMM[int(floor(Houghparam->at(i).second))]);
                      //cout<<eventnum<<endl;
                    }
                    else if((cut2->IsInside(x1,y1))||(cut2->IsInside(x2,y2))) {
                      for(Int_t j = 360; j<20000; j++){
                        if(ThetaLabRecIn[j]<=thetalab){
                          thetacm = ThetaCMSIn[j];
                          //filelist<<eventnum<<"\t"<<i<<"\t"<<thetalab<<"\t"<<thetacm<<"\t"<<ThetaCMS[j]<<endl;
                          break;
                        }
                      }
                      Excitation_IN->Fill(thetacm,(2.0/7000000.0)*EnergyMM[int(floor(Houghparam->at(i).second))]);

                    }
                    Range_hist[i]->Fill(AngleFit->at(i),Range->at(i));
                    HoughSlope[i]->Fill(Houghparam->at(i).second);
                    HoughInt[i]->Fill(Houghparam->at(i).second);

                    HoughFitAngle[i]->Fill(HoughAngle->at(i),AngleFit->at(i));
                }

              eventnum ++;
              plotvals.Fill();
            if(eventnum%10000==0) cout<<eventnum<<endl;;



            }

            c2->cd(1);

            c2->cd(2);
            Q02_Kine->Draw("zcol");
            //anglecut->Draw("SAME");
            gPad->SetLogz();
            Kine_AngRec_AngSca->Draw("C");
            Kine_AngRec_AngSca_In->Draw("C");
    		  	Kine_AngRec_AngSca_vert->Draw("C");
            Kine_AngRec_AngSca_In_vert->Draw("C");
            alpha->Draw("L");
            //leg->Draw();
            diffcut->Draw("same");
            cut2->Draw("same");


        //    c2->cd(2);
          //  Q13_Kine->Draw("zcol");
            //gPad->SetLogz();
            //Kine_AngRec_AngSca->Draw("C");
            //Kine_AngRec_AngSca_vert->Draw("C");

            c4->Draw();
            c4->cd(1);
            Range_hist[0]->Draw("colz");
            c4->cd(2);
            Range_hist[1]->Draw("colz");
            c4->cd(3);
            Range_hist[2]->Draw("colz");
            c4->cd(4);
            Range_hist[3]->Draw("colz");

            c5->Draw();
            c5->cd(1);
            HoughSlope[0]->Draw();
            c5->cd(2);
            HoughSlope[1]->Draw();
            c5->cd(3);
            HoughSlope[2]->Draw();
            c5->cd(4);
            HoughSlope[3]->Draw();
            c5->cd(5);
            HoughInt[0]->Draw();
            c5->cd(6);
            HoughInt[1]->Draw();
            c5->cd(7);
            HoughInt[2]->Draw();
            c5->cd(8);
            HoughInt[3]->Draw();

            c6->Draw();
            c6->cd(1);
            HoughFitAngle[0]->Draw();
            c6->cd(2);
            HoughFitAngle[1]->Draw();
            c6->cd(3);
            HoughFitAngle[2]->Draw();
            c6->cd(4);
            HoughFitAngle[3]->Draw();

            c7->Draw();
            c7->cd(1);
            Diff02->Draw("colz");
            c7->cd(2);
            Diff13->Draw("colz");

            c8->Draw();
            Excitation_EL->GetXaxis()->SetTitle("Center of Mass Angle (degrees)");
            Excitation_EL->GetYaxis()->SetTitle("Center of Mass Energy (MeV)");
            Excitation_EL->GetXaxis()->CenterTitle();
            Excitation_EL->GetYaxis()->CenterTitle();

            Excitation_IN->GetXaxis()->SetTitle("Center of Mass Angle (degrees)");
            Excitation_IN->GetYaxis()->SetTitle("Center of Mass Energy (MeV)");
            Excitation_IN->GetXaxis()->CenterTitle();
            Excitation_IN->GetYaxis()->CenterTitle();

            c8->cd(1);
            gPad->SetLogz();

            Excitation_EL->Draw("colz");
            c8->cd(2);
            gPad->SetLogz();

            Excitation_IN->Draw("colz");

            c9->Draw();
            c9->cd(1);
            RangeDist->Draw("colz");





            TFile f("plots.root","recreate");
            plotvals.Write();
            PlotList.Write();
            f.Close();

            timer.Stop();
	          Double_t rtime = timer.RealTime();
	          Double_t ctime = timer.CpuTime();
	          cout << endl << endl;
	          cout << "Macro finished successfully." << endl;
	          cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
	          cout << endl;




}
