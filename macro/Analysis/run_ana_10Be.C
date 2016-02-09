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
#include "../../include/ATHoughSpaceLine.hh"
#include "../../include/ATHoughSpaceCircle.hh"

#include <ios>
#include <iostream>
#include <istream>
#include <limits>


std::pair<Double_t,Double_t> GetHoughParameters(TH2F* hist);
void myflush ( std::istream& in );
void mypause();

void run_ana_10Be(Int_t num_ev=6000, Int_t file_ini=0, Int_t file_end=0, Int_t runnum=571, Float_t HoughDist=2.0, Bool_t debug=kFALSE, Bool_t stdhough=kFALSE, TString file="../Kinematics/Decay_kinematics/Kine.txt")
{

	    //gStyle->SetCanvasPreferGL(1);
      //gStyle->SetPalette(1);
		  if(!debug)	gErrorIgnoreLevel=kFatal; //Avoid printing Minuit errors

			TCanvas *ctest = new TCanvas("ctest","ctest",200,10,700,700);

	    TCanvas *c1 = new TCanvas("c1","c1",200,10,700,700);
	    c1->Divide(2,2);
	    TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
	    c2->Divide(2,3);
      TCanvas *c3 = new TCanvas("c3","c3",200,10,700,700);
	    c3->Divide(2,1);
			TCanvas *c4 = new TCanvas("c4","c4",200,10,700,700);
	    c4->Divide(2,2);
      TCanvas *c5 = new TCanvas("c5","c5",200,10,700,700);
	    c5->Divide(2,2);
      TCanvas *c6 = new TCanvas("c6","c6",200,10,700,700);
      c6->Divide(2,1);

	    TH2F* fQuadrant[4];
	     for (Int_t i=0;i<4;i++){
			 fQuadrant[i] = new TH2F(Form("fQuadrant[%i]",i),Form("fQuadrant%i",i),500,0,3.15,2500,0,1000);
			 fQuadrant[i]->SetFillColor(2);
		}


      TH1I* DistHist[4];
			for (Int_t i=0;i<4;i++) DistHist[i] = new TH1I(Form("DistHist[%i]",i),Form("DistHit%i",i),100,0,1000);



	    TF1 *HoughFit[4];
			TF1 *HoughFitSTD[4];
			TF1 *fitResult[4];
      TGraph *HitPattern[4];
			TGraph *HitPatternFilter[4];


			for (Int_t i=0;i<4;i++){
						HoughFit[i] =new TF1(Form("HoughFit%i",i)," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])",0,120);
						HoughFit[i]->SetLineColor(kRed);
						HoughFitSTD[i] =new TF1(Form("HoughFitSTD%i",i)," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])",0,120);
						HoughFitSTD[i]->SetLineColor(kBlue);
						HitPatternFilter[i] = new TGraph();
						HitPatternFilter[i]->SetMarkerStyle(22);
						HitPatternFilter[i]->SetMarkerSize(0.7);

			}

      TH2D* Q02_Kine = new TH2D("Q02_Kine","Q02_Kine",1000,0,180,1000,0,180);
	    Q02_Kine->SetMarkerColor(2);
	    Q02_Kine->SetMarkerStyle(20);
	    Q02_Kine->SetMarkerSize(0.7);
	    TH2D* Q13_Kine = new TH2D("Q13_Kine","Q13_Kine",1000,0,180,1000,0,180);
	    Q13_Kine->SetMarkerColor(2);
	    Q13_Kine->SetMarkerStyle(20);
	    Q13_Kine->SetMarkerSize(0.7);

	    TH2D* Q02_Kine_buff = new TH2D("Q02_Kine_buff","Q02_Kine_buff",1000,0,180,1000,0,180);
	    Q02_Kine_buff->SetMarkerColor(4);
	    Q02_Kine_buff->SetMarkerStyle(20);
      Q02_Kine_buff->SetMarkerSize(0.7);
	    TH2D* Q13_Kine_buff = new TH2D("Q13_Kine_buff","Q13_Kine_buff",1000,0,180,1000,0,180);
	    Q13_Kine_buff->SetMarkerColor(4);
	    Q13_Kine_buff->SetMarkerStyle(20);
	    Q13_Kine_buff->SetMarkerSize(0.7);

      TH2D* Vertex_Angle0 = new TH2D("Vertex_Angle0","Vertex_Angle0",1000,0,180,500,0,500);
			Vertex_Angle0->SetMarkerColor(4);
	    Vertex_Angle0->SetMarkerStyle(20);
	    Vertex_Angle0->SetMarkerSize(0.7);

      TH2D* Vertex_Angle1 = new TH2D("Vertex_Angle1","Vertex_Angle1",1000,0,180,500,0,500);
			Vertex_Angle1->SetMarkerColor(4);
	    Vertex_Angle1->SetMarkerStyle(20);
	    Vertex_Angle1->SetMarkerSize(0.7);

			TH2D* Vertex_Angle2 = new TH2D("Vertex_Angle2","Vertex_Angle2",1000,0,180,500,0,500);
			Vertex_Angle2->SetMarkerColor(4);
	    Vertex_Angle2->SetMarkerStyle(20);
	    Vertex_Angle2->SetMarkerSize(0.7);

			TH2D* Vertex_Angle3 = new TH2D("Vertex_Angle3","Vertex_Angle3",1000,0,180,500,0,500);
			Vertex_Angle3->SetMarkerColor(4);
	    Vertex_Angle3->SetMarkerStyle(20);
	    Vertex_Angle3->SetMarkerSize(0.7);

      TH1D* Vertex_Histo = new TH1D("Vertex_Histo","Vertex_Histo",100,0,500);
      TH1D* Vertex_Diff = new TH1D("Vertex_Diff","Vertex_Diff",10,0,50);


	    TH2F* fQuadHist[4];
		for (Int_t i=0;i<4;i++){
			 fQuadHist[i] = new TH2F(Form("fQuadHist[%i]",i),Form("fQuadHist%i",i),100,0,130,200,0,500);
			 fQuadHist[i]->SetFillColor(2);
		}


	    TChain *chain = new TChain("cbmsim");
	    TFileCollection *filecol = new TFileCollection();
	    TString FileNameHead_num;
	    TString FileNameHead_chain;

        Double_t *ThetaCMS = new Double_t[20000];
       	Double_t *ThetaLabRec = new Double_t[20000];
				Double_t *EnerLabRec = new Double_t[20000];
				Double_t *ThetaLabSca = new Double_t[20000];
				Double_t *EnerLabSca = new Double_t[20000];

				std::ifstream *kineStr = new std::ifstream(file.Data());
				Int_t numKin=0;

				  if(!kineStr->fail()){
						while(!kineStr->eof()){
								*kineStr>>ThetaCMS[numKin]>>ThetaLabRec[numKin]>>EnerLabRec[numKin]>>ThetaLabSca[numKin]>>EnerLabSca[numKin];
								numKin++;
						}
					}else if(kineStr->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

					TGraph *Kine_AngRec_AngSca = new TGraph(numKin,ThetaLabRec,ThetaLabSca);
					TGraph *Kine_AngRec_AngSca_vert = new TGraph(numKin,ThetaLabSca,ThetaLabRec);


      for(Int_t i=0;i<=file_end;i++){
				if(i<10) FileNameHead_num.Form("%i_000%i",runnum,i);
				else if(i<100) FileNameHead_num.Form("%i_00%i",runnum,i);
				else if(i<1000) FileNameHead_num.Form("%i_0%i",runnum,i);
				FileNameHead_chain = "../run"+FileNameHead_num+".root";
				std::cout<<" File : "<<FileNameHead_chain<<" added"<<std::endl;
				//filecol->Add(FileNameHead_chain);
				//chain->Add(FileNameHead_chain);





	    //TString FileNameHead = "../run571_0003";
      TString FileNameHead = "../output_proto";
	    TString FileNameTail = ".root";
	    TString FileName     = FileNameHead + FileNameTail;
	    //TString outFileNameHead = "data/";
	    //TString outFileNameTail = ".root";
	    //TString outFileName     = outFileNameHead + reactionName + outFileNameTail;

	     //TFile* file = new TFile(FileName.Data(),"READ");
	    TFile* file = new TFile(FileNameHead_chain.Data(),"READ");
      TTree* tree = (TTree*) file -> Get("cbmsim");
	    Int_t nEvents = tree -> GetEntriesFast();
	    std::cout<<" Number of events : "<<nEvents<<std::endl;

	    TTreeReader Reader1("cbmsim", file);
	    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
	    TTreeReaderValue<TClonesArray> protoeventArray(Reader1, "ATProtoEvent");
	    TTreeReaderValue<TClonesArray> houghArray(Reader1, "ATHough");
	    Bool_t fIsLinear=kFALSE;
	    Bool_t fIsCircular=kFALSE;

	    Int_t value;
	    Int_t nEve=0;

		while (Reader1.Next() && nEve<num_ev) {
			nEve++;
			if(debug) mypause();
			Q02_Kine_buff->Reset(0);
			Q13_Kine_buff->Reset(0);
			for (Int_t i=0;i<4;i++){
				fQuadHist[i]->Reset();
				HitPatternFilter[i]->Set(0);
			}

			ATEvent* event = (ATEvent*) eventArray->At(0);
			Int_t nHits = event->GetNumHits();
			//std::cout<<" ==================================================================================="<<std::endl;
			if(nEve%100==0) std::cout<<" Event number : "<<event->GetEventID()<<" - Number of Hits : "<<nHits<<std::endl;
      if(debug)cout<<event->GetEventID()<<endl;
			if(debug) for (Int_t i=0;i<4;i++) DistHist[i]->Reset(0);



			   ATHoughSpaceLine* fHoughSpaceLine_buff  = dynamic_cast<ATHoughSpaceLine*> (houghArray->At(0));

       //  ATHoughSpaceCircle* fHoughSpaceCircle_buff;
			/*   if( (fHoughSpaceLine_buff = dynamic_cast<ATHoughSpaceLine*> (houghArray->At(0)) )){
				//std::cout<<" Linear Hough Space Found!"<<std::endl;
				fIsLinear=kTRUE;
     			   }else if( (fHoughSpaceCircle_buff = dynamic_cast<ATHoughSpaceCircle*> (houghArray->At(0)) )){
				//std::cout<<"Circular Hough Space Found!"<<std::endl;
				fIsCircular=kTRUE;
			}else std::cout<<"Hough Space Type NOT Found!"<<std::endl;*/



			   ATProtoEvent* protoevent = (ATProtoEvent*) protoeventArray->At(0);
			   std::vector<ATProtoQuadrant> quadrantArray;
     		 ATProtoQuadrant* quadrant;
			   std::vector<Int_t> qNumHits;
			   std::vector<Double_t> qRad;



				 std::vector<std::pair<Double_t,Double_t>> HoughPar = fHoughSpaceLine_buff->GetHoughPar(); //No argument retrieve parameters from Hough Histogram
				 std::vector<std::pair<Double_t,Double_t>> HoughParSTD;
				 if(stdhough) HoughParSTD = fHoughSpaceLine_buff->GetHoughPar("STD");

				 std::vector<Double_t> par0_fit;
				 std::vector<Double_t> par1_fit;
				 std::vector<Double_t> Angle;
				 std::vector<Double_t> Angle_fit;
				 //par0_fit.resize(4);
				 //par1_fit.resize(4);

         Int_t qFitStatus [4] = {0,0,0,0};

				 for(Int_t i=0;i<HoughPar.size();i++){

									Angle.push_back(180-HoughPar.at(i).first*180/TMath::Pi());

									if(debug){
									std::cout<<" ------ Hough Parameters for Quadrant : "<<i<<std::endl;
									std::cout<<" Angle HIST : "<<180-HoughPar.at(i).first*180/TMath::Pi()<<std::endl;
									std::cout<<" Distance HIST : "<<HoughPar.at(i).second<<std::endl;

									if(stdhough){
									std::cout<<" Angle STD : "<<180-HoughParSTD.at(i).first*180/TMath::Pi()<<std::endl;
									std::cout<<" Distance STD : "<<HoughParSTD.at(i).second<<std::endl;
									}


									HoughFit[i]->SetParameter(0,HoughPar.at(i).first);
									HoughFit[i]->SetParameter(1,HoughPar.at(i).second);
									//HoughFit[i]->SetRange(0,qRad.at(i));

									if(stdhough){

									HoughFitSTD[i]->SetParameter(0,HoughParSTD.at(i).first);
									HoughFitSTD[i]->SetParameter(1,HoughParSTD.at(i).second);
									//HoughFitSTD[i]->SetRange(0,qRad.at(i));

									}
								}

				}// HoughPar Loop




				for (Int_t i=0;i<4;i++){
          qFitStatus[i]=0;
					//fQuadrant[i]  = fHoughSpaceLine_buff->GetHoughQuadrant(i); //Obsolete
					quadrant = &protoevent->GetQuadrantArray()->at(i);
          qNumHits.push_back(quadrant->GetNumHits());
					Int_t qNumHit = quadrant->GetNumHits();
          Double_t *rad_graph = new Double_t[qNumHit];
          Double_t *posz_graph = new Double_t[qNumHit];



			   	Double_t rad_max=0.0;

							for(Int_t j=0;j<qNumHit;j++){
								ATHit* qhit = quadrant->GetHit(j);
								TVector3 position = qhit->GetPosition();
								Double_t radius = TMath::Sqrt( TMath::Power(position.X(),2) + TMath::Power(position.Y(),2) );
								if(radius>rad_max) rad_max=radius;
								fQuadHist[i]->Fill(radius,position.Z(),qhit->GetCharge());
                rad_graph[j] = radius;
                posz_graph[j] = position.Z();


								Int_t pixelY = ctest->YtoAbsPixel(position.Z());
  							Int_t pixelX = ctest->XtoAbsPixel(radius);
								Int_t pixelYback = ctest->AbsPixeltoY(pixelY);
  							Int_t pixelXback = ctest->AbsPixeltoX(pixelX);

								Int_t mda = HoughFit[i]->DistancetoPrimitive(-pixelX,pixelY); //Minimum Distance of Approach in Pixels
								DistHist[i]->Fill(mda);

								Double_t geo_dist = TMath::Abs (TMath::Cos(HoughPar.at(i).first)*radius  + TMath::Sin(HoughPar.at(i).first)*position.Z()  - HoughPar.at(i).second);

                // An example to make annotations to each one of the TGraph points
							  /*	TLatex *latex = new TLatex(gr->GetX()[5], gr->GetY()[5],"my annotation");
    						gr->GetListOfFunctions()->Add(latex);
    						gr->Draw("alp");
								latex->SetTextSize(0.07);
                latex->SetTextColor(kRed);*/


								/*if(debug){
									    std::cout<<" ___________________________________________________________________________________________________"<<std::endl;
										  std::cout<<" Hit : "<<j<<" - Radius : "<<radius<<" - Z pos : "<<position.Z()<<"        -  Distance to Hough Space : "<<mda<<" Geomtrical Distance : "<<geo_dist<<std::endl;
											//std::cout<<" Pixel X : "<<pixelX<<" Pixel Y : "<<pixelY<<std::endl;
											//std::cout<<" Pixel Back X : "<<pixelXback<<" Pixel Back Y : "<<pixelYback<<std::endl;
							  	}*/

									if(geo_dist<HoughDist) HitPatternFilter[i]->SetPoint(HitPatternFilter[i]->GetN(),radius,position.Z());

							}//NumHit loop

								Double_t par0=0.0;
								Double_t par1=0.0;
								Double_t afit=0.0;



					 qRad.push_back(rad_max);
					 	//HoughFit[i]->SetRange(0,rad_max);

				   if(debug) HitPattern[i] = new TGraph(qNumHit,rad_graph,posz_graph);

						 if(debug)std::cout<<" Fitting Quadrant : "<<i<<std::endl;
						 if(qNumHit>3){
							  HitPatternFilter[i]->Fit("pol1","FQ");
							  fitResult[i] = HitPatternFilter[i]->GetFunction("pol1");

								//if(!fitResult[i]) std::cout<<" NULL "<<std::endl;

								if(fitResult[i]){
									Bool_t IsValid = fitResult[i]->IsValid();
									fitResult[i] ->SetName(Form("fitResult%i",i));
				   			  fitResult[i] ->SetLineColor(kRed);
				   				fitResult[i] ->SetLineWidth(1);
									par0 = fitResult[i]->GetParameter(0);
								 	par1 = fitResult[i]->GetParameter(1);

									if(par1>=0) afit = TMath::ATan2(1,TMath::Abs(par1));
									else if(par1<0)  afit = TMath::Pi()-TMath::ATan2(1,TMath::Abs(par1));

												//std::cout<<" Par 0 : "<<par0<<std::endl;
												//std::cout<<" Par 1 : "<<par1<<std::endl;
				                if(gMinuit && debug) std::cout<<gMinuit->fCstatu<<std::endl;
                        if(gMinuit->fCstatu != "NOT POSDEF") qFitStatus[i]=1;

									}

						 }

						 par0_fit.push_back(par0);
						 par1_fit.push_back(par1);
						 Angle_fit.push_back(afit*180/TMath::Pi());



				  }// Quadrant Loop


        Bool_t goodFit = kFALSE;
        if(qFitStatus[0]+qFitStatus[1]+qFitStatus[2]+qFitStatus[3]==4) goodFit = kTRUE;
        //cout<<goodFit<<endl;


					/*std::cout<<" Radius Q0 : "<<qRad.at(0)<<std::endl;
					std::cout<<" Radius Q1 : "<<qRad.at(1)<<std::endl;
					std::cout<<" Radius Q2 : "<<qRad.at(2)<<std::endl;
					std::cout<<" Radius Q3 : "<<qRad.at(3)<<std::endl;*/


					if((qRad.at(0)>5 && qRad.at(2)>5)  && (qNumHits.at(0)>5 && qNumHits.at(2)>5) && (TMath::Abs(par0_fit.at(0)-par0_fit.at(2))<50)&&goodFit){
						Q02_Kine->Fill(Angle_fit.at(0),Angle_fit.at(2));
						Q02_Kine_buff->Fill(Angle_fit.at(0),Angle_fit.at(2));
            Vertex_Angle0->Fill(Angle_fit.at(0),par0_fit.at(0));
            Vertex_Angle2->Fill(Angle_fit.at(2),par0_fit.at(2));
            Vertex_Histo->Fill(par0_fit.at(0));
            Vertex_Diff->Fill(TMath::Abs(par0_fit.at(0)-par0_fit.at(2)));

						if(debug){
 							std::cout<<" Vertex difference : "<<TMath::Abs(par0_fit.at(0)-par0_fit.at(2))<<std::endl;
							std::cout<<" Quadrant 0 - Par 0 : "<<par0_fit.at(0)<<" Par 1 : "<<par1_fit.at(0)<<std::endl;
							std::cout<<" Quadrant 2 - Par 0 : "<<par0_fit.at(2)<<" Par 1 : "<<par1_fit.at(2)<<std::endl;
							std::cout<<" Angle from Hough Space Quadrant 0 : "<<Angle.at(0)<<" Angle from Hough Space Quadrant 2 : "<<Angle.at(2)<<std::endl;
							std::cout<<" Angle from Fit Quadrant 0 : "<<Angle_fit.at(0)<<" Angle from Fit Quadrant 2 : "<<Angle_fit.at(2)<<std::endl;
						}
					}
					if((qRad.at(1)>5 && qRad.at(3)>5)  && (qNumHits.at(1)>5 && qNumHits.at(3)>5) && (TMath::Abs(par0_fit.at(1)-par0_fit.at(3))<50)&&goodFit){
						Q02_Kine->Fill(Angle_fit.at(1),Angle_fit.at(3));
						Q02_Kine_buff->Fill(Angle_fit.at(1),Angle_fit.at(3));
            Vertex_Angle3->Fill(Angle_fit.at(3),par0_fit.at(3));
            Vertex_Angle1->Fill(Angle_fit.at(1),par0_fit.at(1));
            Vertex_Histo->Fill(par0_fit.at(1));
            Vertex_Diff->Fill(TMath::Abs(par0_fit.at(1)-par0_fit.at(3)));

						if(debug){
 						std::cout<<" Vertex difference : "<<TMath::Abs(par0_fit.at(1)-par0_fit.at(3))<<std::endl;
						std::cout<<" Quadrant 1 - Par 0 : "<<par0_fit.at(1)<<" Par 1 : "<<par1_fit.at(1)<<std::endl;
						std::cout<<" Quadrant 3 - Par 0 : "<<par0_fit.at(3)<<" Par 1 : "<<par1_fit.at(3)<<std::endl;
						std::cout<<" Angle from Hough Space Quadrant 1 : "<<Angle.at(1)<<" Angle from Hough Space Quadrant 3 : "<<Angle.at(3)<<std::endl;
						std::cout<<" Angle from Fit Quadrant 1 : "<<Angle_fit.at(1)<<" Angle from Fit Quadrant 3 : "<<Angle_fit.at(3)<<std::endl;
				  	}
					}



				/*for(Int_t iHit=0; iHit<nHits; iHit++)
  					{
    						ATHit hit = event->GetHitArray()->at(iHit);
                */
       if(debug){
				c1->cd(1);
				HitPattern[0]->Draw("A*");
				//HitPatternFilter[0]->Draw("A*");
				HoughFit[0]->Draw("SAME");
				if(stdhough) HoughFitSTD[0]->Draw("SAME");

				c1->cd(2);
				HitPattern[1]->Draw("A*");
				HoughFit[1]->Draw("SAME");
				if(stdhough) HoughFitSTD[1]->Draw("SAME");

				c1->cd(3);
				HitPattern[2]->Draw("A*");
				HoughFit[2]->Draw("SAME");
				if(stdhough) HoughFitSTD[2]->Draw("SAME");

				c1->cd(4);
				HitPattern[3]->Draw("A*");
				HoughFit[3]->Draw("SAME");
				if(stdhough) HoughFitSTD[3]->Draw("SAME");

				c1->Modified();
				c1->Update();

				c4->cd(1);
				//DistHist[0]->Draw();
				HitPatternFilter[0]->Draw("A*");
				c4->cd(2);
				//DistHist[1]->Draw();
				HitPatternFilter[1]->Draw("A*");
				c4->cd(3);
				//DistHist[2]->Draw();
				HitPatternFilter[2]->Draw("A*");
				c4->cd(4);
				//DistHist[3]->Draw();
				HitPatternFilter[3]->Draw("A*");
				c4->Modified();
				c4->Update();


				c2->cd(1);
				fQuadHist[0]->Draw("contz");
				c2->cd(2);
				fQuadHist[1]->Draw("contz");
				c2->cd(3);
				fQuadHist[2]->Draw("contz");
				c2->cd(4);
				fQuadHist[3]->Draw("contz");
				c2->cd(5);
				Q02_Kine->Draw("");
				Q02_Kine_buff->Draw("SAME");
				c2->cd(6);
				Q13_Kine->Draw("");
				Q13_Kine_buff->Draw("SAME");
				c2->Modified();
				c2->Update();

        c3->cd(1);
				Q02_Kine->Draw("");
				Kine_AngRec_AngSca->Draw("C");
				Kine_AngRec_AngSca_vert->Draw("C");
        c3->Modified();
        c3->Update();

				ctest->cd();
				HitPattern[0]->Draw("A*");
				HoughFit[0]->Draw("SAMES");
				ctest->Modified();
				ctest->Update();

			}

				/*c1->cd(1); //Obsolete
				fQuadrant[0]->Draw("zcol");
				c1->cd(2);
				fQuadrant[1]->Draw("zcol");
				c1->cd(3);
				fQuadrant[2]->Draw("zcol");
				c1->cd(4);
				fQuadrant[3]->Draw("zcol");
				c1->Modified();
				c1->Update();	*/

				//myflush ( std::cin );
  				//mypause();
  				//std::cin.get();



		}//While

	}//for files



        if(!debug){
				c3->cd(1);
				Q02_Kine->Draw("");
				Kine_AngRec_AngSca->Draw("C");
				Kine_AngRec_AngSca_vert->Draw("C");


				c3->cd(2);
				Q13_Kine->Draw("");
				Kine_AngRec_AngSca->Draw("C");
				Kine_AngRec_AngSca_vert->Draw("C");

        c5->Draw();
        c5->cd(1);
        Vertex_Angle0->Draw("");
        c5->cd(2);
        Vertex_Angle1->Draw("");
        c5->cd(3);
        Vertex_Angle2->Draw("");
        c5->cd(4);
        Vertex_Angle3->Draw("");

        c6->Draw();
        c6->cd(1);
        Vertex_Histo->Draw("");
        c6->cd(2);
        Vertex_Diff->Draw("");

			 }







}

/*std::pair<Double_t,Double_t> GetHoughParameters(TH2F* hist){
		std::pair<Double_t,Double_t> HoughPar;
		Int_t locmaxx,locmaxy,locmaxz;
                hist->GetMaximumBin(locmaxx,locmaxy,locmaxz);
                Double_t xpos = hist->GetXaxis()->GetBinCenter(locmaxx);
                Double_t ypos = hist->GetYaxis()->GetBinCenter(locmaxy);
		//std::cout<<" X Hough Position : "<<xpos<<std::endl;
		//std::cout<<" Y Hough Position : "<<ypos<<std::endl;
		HoughPar.first= xpos;
		HoughPar.second= ypos;
		return HoughPar;
}*/

void myflush ( std::istream& in )
{
  in.ignore ( std::numeric_limits<std::streamsize>::max(), '\n' );
  in.clear();
}

void mypause()
{
  std::cout<<"Press [Enter] to continue . . .";
  std::cin.get();
}
