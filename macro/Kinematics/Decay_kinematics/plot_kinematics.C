void plot_kinematics()
{

  TCanvas *c1 = new TCanvas("c1","c1",200,10,700,700);

  Double_t *ThetaCMS = new Double_t[20000];
  Double_t *ThetaLabRec = new Double_t[20000];
  Double_t *EnerLabRec = new Double_t[20000];
  Double_t *ThetaLabSca = new Double_t[20000];
  Double_t *EnerLabSca = new Double_t[20000];

  Double_t *ThetaCMS2 = new Double_t[20000];
  Double_t *ThetaLabRec2 = new Double_t[20000];
  Double_t *EnerLabRec2 = new Double_t[20000];
  Double_t *ThetaLabSca2 = new Double_t[20000];
  Double_t *EnerLabSca2 = new Double_t[20000];

  Double_t *ThetaCMS3 = new Double_t[20000];
  Double_t *ThetaLabRec3 = new Double_t[20000];
  Double_t *EnerLabRec3 = new Double_t[20000];
  Double_t *ThetaLabSca3 = new Double_t[20000];
  Double_t *EnerLabSca3 = new Double_t[20000];

  TString fileKine="12Be_elas.txt";
	std::ifstream *kineStr = new std::ifstream(fileKine.Data());
	Int_t numKin=0;

		if(!kineStr->fail()){
			while(!kineStr->eof()){
					*kineStr>>ThetaCMS[numKin]>>ThetaLabRec[numKin]>>EnerLabRec[numKin]>>ThetaLabSca[numKin]>>EnerLabSca[numKin];
					numKin++;
			}
		}else if(kineStr->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

		TGraph *Kine_AngRec_AngSca = new TGraph(numKin,ThetaLabRec,ThetaLabSca);
		TGraph *Kine_AngRec_AngSca_vert = new TGraph(numKin,ThetaLabSca,ThetaLabRec);
		TGraph *Kine_AngRec_EnerRec = new TGraph(numKin,ThetaLabRec,EnerLabRec);

		TString fileKine2="12Be_2.1.txt";
		std::ifstream *kineStr2 = new std::ifstream(fileKine2.Data());
	  numKin=0;

		if(!kineStr2->fail()){
			while(!kineStr2->eof()){
					*kineStr2>>ThetaCMS2[numKin]>>ThetaLabRec2[numKin]>>EnerLabRec2[numKin]>>ThetaLabSca2[numKin]>>EnerLabSca2[numKin];
					numKin++;
			}
		}else if(kineStr2->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

		TGraph *Kine_AngRec_AngSca2 = new TGraph(numKin,ThetaLabRec2,ThetaLabSca2);
		TGraph *Kine_AngRec_AngSca_vert2 = new TGraph(numKin,ThetaLabSca2,ThetaLabRec2);
		TGraph *Kine_AngRec_EnerRec2 = new TGraph(numKin,ThetaLabRec2,EnerLabRec2);

    TString fileKine3="12Be_4He_10Be_6He.txt";
		std::ifstream *kineStr3 = new std::ifstream(fileKine3.Data());
		numKin=0;

		if(!kineStr3->fail()){
			while(!kineStr3->eof()){
					*kineStr3>>ThetaCMS3[numKin]>>ThetaLabRec3[numKin]>>EnerLabRec3[numKin]>>ThetaLabSca3[numKin]>>EnerLabSca3[numKin];
					numKin++;
			}
		}else if(kineStr3->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

		TGraph *Kine_AngRec_AngSca3 = new TGraph(numKin,ThetaLabRec3,ThetaLabSca3);
		TGraph *Kine_AngRec_AngSca_vert3 = new TGraph(numKin,ThetaLabSca3,ThetaLabRec3);
		TGraph *Kine_AngRec_EnerRec3 = new TGraph(numKin,ThetaLabRec3,EnerLabRec3);

    Kine_AngRec_AngSca2->SetMarkerColor(kRed);
    Kine_AngRec_AngSca3->SetMarkerColor(kBlue);

    c1->cd();
    Kine_AngRec_AngSca->Draw("A*");
    Kine_AngRec_AngSca2->Draw("*");
    Kine_AngRec_AngSca3->Draw("*");

}
