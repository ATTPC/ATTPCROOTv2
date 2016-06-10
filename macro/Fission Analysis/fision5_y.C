Double_t erf1(Double_t *x, Double_t *par){

	Double_t fitval1=par[0]*TMath::Erf((par[1]-x[0])/par[2]);
	return fitval1;

}

Double_t erf2(Double_t *x, Double_t *par){

  Double_t fitval2=par[0]*TMath::Erf((par[3]-x[0])/par[2]);
  return fitval2;

}

Double_t fitErf(Double_t *x, Double_t *par){

  return erf2(x,par)-erf1(x,par);

}

#include <iostream>


void fision5_y()
{
  TRandom q,o,ga;
  TH1D theta_1dist("his1","Angular distribution in LAB frame Fragment 1",500,0,.06);
  TH1D zdist("his14","Angular distribution in LAB frame Fragment 1",500,15,15.0002);
  TH1D theta_2dist("his2","Angular distribution in LAB frame Fragment 2",500,0,.06);
  TH2D theta_1theta_2dist("his11","Angular distribution in LAB frame  both Fragments",500,0,.06,500,0,.06);
  //TH1D theta_1dist("his1","Angular distribution in LAB frame Fragment 1",500,0.995,1.001);
  //TH1D theta_2dist("his2","Angular distribution in LAB frame Fragment 2",500,0.995,1.001);
  TH1D theta_dist("his3","Angular distribution in LAB both Fragments",200,0,0.06);
  TH1D costheta_disth("his8","Angular distribution in LAB heavy Fragments (cos)",200,0,0.002);
  TH1D costheta_distl("his9","Angular distribution in LAB light Fragments (cos)",200,0,0.002);
  TH1D costheta_dist("his7","Angular distribution in LAB both Fragments (cos)",200,0,0.002);
  TH1D CM_dist("his6","Angular distribution in CM both Fragments",500,0,4);
  TH1D Zeta_dist("his4","Charge distribution in LAB both Fragments",100,0,100);
  TH2D ZetaTheta_dist("his5","Charge distribution VS Theta LAB both Fragments",500,-0.01,3.2,100,0,100);
  TH2D *XY_dist=new TH2D("his10","position distribution of the fragments xVSy",1000,-150,150,500,-150,150);
  TH1D *Y_dist=new TH1D("his11111","position distribution of the fragments Y",1000,-100,100);
  TH1D separation_dist("hist12","SEparation between Fission Fragments",500,0,150);
  TH2D *X0Y0_dist= new TH2D("his13","position distribution of the fissioning nuclei",1000,-1,1,1000,-1,1);
  TH2D *XY_dist2=new TH2D("his102","position distribution of the lost fragments  (Twin cathode)",1000,-10,10,1000,-10,10);
  TH2D *XY_dist3=new TH2D("his103","position distribution of the detected fragments xVSy (Twin cathode)",1000,-100,100,1000,-100,100);
  TH1D *Y_dist2=new TH1D("his2222","position distribution of the fragments Y (cathode cut)",1000,-5,10);
	TH1D *Momentum_dist1 = new TH1D("Momentum_dist1","Momentum distribution of fragment 1", 1000,-2,12);
	TH1D *Momentum_dist2 = new TH1D("Momentum_dist2","Momentum distribution of fragment 2", 1000,-2,12);
	TH1D *Totalmomentum_dist = new TH1D("Totalmomentum_dist","Total Momentum Distribution", 1000, -2,12);
	TH1D *Totalmass_dist = new TH1D("Totalmass_dist","Total Mass Distribution", 1000,50,200);
	TH1D *Mass_dist1 = new TH1D("Mass_dist","Mass Distribution of fragment 1", 1000,50,200);
	TH1D *Mass_dist2 = new TH1D("Mass_dist","Mass Distribution of fragment 2", 1000,50,200);
	TH2D *TKEvM = new TH2D("TKEvM", "TKE as a function of Mass", 1000,40,200,1000,40,400);


 TH1D* theta_his=&theta_dist;
 Char_t filename[25];
 Int_t Multihit,k,stripsize;//(cm)
 Float_t beamspot;
 TCanvas win("win");
 //win.Divide(2,2);
 win.Update();
 win.cd(1);

 Double_t x1[5]={-11,11,11,-11,-11};
 Double_t y1[5]={0.5,0.5,-0.5,-0.5,0.5};
 Double_t x2[5]={-1,1,1,-1,-1};
 Double_t y2[5]={1.0,1.0,0.065,0.065,1.0};
 Double_t x3[5]={-1,1,1,-1,-1};
 Double_t y3[5]={-1.0,-1.0,-0.025,-0.025,-1.0};


 TCutG* gcut1 = new TCutG("cut1",5,x1,y1);
 TCutG* gcut2 = new TCutG("cut2",5,x2,y2);
 TCutG* gcut3 = new TCutG("cut3",5,x3,y3);

 Float_t hi=0;
	Float_t ki=0;
	Float_t li=0;
	Float_t fi=0;
	Float_t xi=0;

	struct event {
	  Double_t theta1_lab;
	  Double_t theta2_lab;
	  Double_t costheta_1;
	  Double_t costheta_2;
	  Double_t x1;
	  Double_t y1;
	  Double_t x2;
	  Double_t y2;
	};
	event evnt;
	Double_t Z1,Z2,A1,A2,N1,N2,E_beam,TKE,tof1,tof2, B;
	Double_t variable1,variable2;
	Double_t c=29.972; //c in cm/ns
  Double_t c1 = 3*pow(10,8);
	Double_t A = 208; //mass of the fissioning nucleus 208Pb
	Double_t uma=0.93149; //GeV/c^2
	Double_t pi=TMath::Pi(); //pi
	Double_t v1_cm[3],v2_cm[3],v1_lab[3],v2_lab[3],beta[3],pos1[3],pos2[3],pos0[3];
	Double_t gamma;
	Double_t dist[4]={5,10,15,20};//distance for the detector (m)
	Double_t size[4];//Detector size needed
  Double_t p1; //Momentum of fragment 1 (N*s)
  Double_t p1G; //Momeutnum of fragment 1 (GeV/c)
	Double_t p2;
	Double_t p2G;
  Double_t TKEJ; //Total Kinetic Energy (Joules)
	Double_t pz1, pz2, E1, E2;

	TString dir = getenv("VMCWORKDIR");
	std::ifstream*  fInputFilebase;
	TString fFileNamebase;

	fFileNamebase = dir+"/macro/Simulation/database/Ir175Mass_dist_dummy.txt";
	//	std::cout << " ATTPCFissionGenerator: Opening input file " << fFileNamebase << std::endl;
		 // Open first the file to register all new ions.
		fInputFilebase = new std::ifstream(fFileNamebase);
		if ( ! fInputFilebase->is_open() )
				 Fatal("ATTPCFissionGenerator","Cannot open input file.");

	//	std::cout << "ATTPCFissionGenerator: Looking for ions..." << std::endl;




	cout << "Calculation of fission kinematics for uranium" <<endl;
	cout << "please insert the energy of the beam (GeV/u):";
	cin >>E_beam;
	cout << "please insert the size of the gap (cm):";
	cin >>stripsize;
	Int_t d;
	d=sprintf(filename,"hist_%g.root",E_beam);
	//cout << "please insert the charge of one of the fragments:";
	//cin>>Z1;
	//TFile *f = new TFile(filename,"RECREATE");
	//TTree* tree=new TTree("T","Fission Fragments");
	Char_t buffer[23];
	Int_t w;
	w = sprintf(buffer,"Energy_%gGeV_2.out",E_beam);
	cout << "Calculating... Please wait"<<endl;
	//Double_t aux = E_beam;
	//E_beam = m;
	cout << " Energy of the beam "<< E_beam <<" GeV/u"<<endl;
	ofstream* out = new ofstream(buffer);
	//tree->Branch("FF",,&evnt.theta1_lab,"evnt.theta1_lab/D:evnt.theta2_lab:
	//evnt.costheta_1:evnt.costheta_2:evnt.x1:evnt.y1:evnt.x2:evnt.y2");

	*out << "**********************************************************"<<endl;
	*out << "**********************************************************"<<endl;
	//Lorentz boost in z axis to LAB frame
	beta[0] = 0;
	beta[1] = 0;
	beta[2] = sqrt(1-(1/pow((E_beam/uma+1),2)));
	Double_t aux = sqrt(pow(beta[0],2)+pow(beta[1],2)+pow(beta[2],2));
	gamma = pow((1-aux*aux),-0.5);
	*out << " Energy of the beam \t"<< E_beam <<" GeV/u"<<endl;
	*out << " Beam velocity (beta) \t"<< aux <<"c"<< endl;
	*out << " Lorentz factor (gamma) \t"<< gamma << endl;
	cout << " Beam velocity (beta) \t"<< aux <<"c"<< endl;
	cout << " Lorentz factor (gamma) \t"<< gamma << endl;
				for( k=1;k<=20000;k++){
	  Double_t phi0 = o.Rndm(1)*2*pi;
	  Double_t costheta0 = 1-2*o.Rndm();
	  Double_t theta0 =acos(costheta0);
	  Double_t r=o.Gaus(0,0.2); // sigma=0.2 Reference value (Yassid style) sigma=0.4 Reference value (David style)
	  Double_t ra=o.Gaus(0,0.2);
	  //pos0[0]=r*sin(theta0)*cos(phi0); // David style
	  //pos0[1]=r*sin(theta0)*sin(phi0);
	  pos0[0]=r;                        // Yassid style
	  pos0[1]=ra;
	  pos0[2]=0;

	  X0Y0_dist->Fill(pos0[0],pos0[1]);



		//*fInputFilebase>> A1 >> Z1;

	  Z1=ga.Gaus(41,6);

	  //Z1= 27+q.Integer(16);
	  //Z1= 40;
	  Zeta_dist.Fill(Z1);
	  Z2 = 98 - Z1;
	  Zeta_dist.Fill(Z2);
	  N1 = 148./98.*Z1;
	  N2 = 148-N1;
	  A1 = Z1+N1;
	  A2 = Z2+N2;
		Totalmass_dist->Fill(A1);
		Totalmass_dist->Fill(A2);
		Mass_dist1->Fill(A1);
		Mass_dist2->Fill(A2);
  //Total Kinetic energy (Wilkins model)(GeV)
	  TKE = (0.00144*Z1*Z2)/(1.16*(pow(A1,1./3.)*(1.+2./3.*0.6)+pow(A2,1./3.)*(1.+2./3.*0.6))+2);

	  //Velocities moduli of the fragments in CM frame (cm/ns)
	  Double_t v11 = sqrt(2*A2*TKE/(uma*A1*(A1+A2)))*c;
	  Double_t v22 = v11*A1/A2;
  //Maximum polar angle and azimuth = 0
  //for(Int_t t= 1;t<=20000;t++){
	  Double_t phi = q.Rndm()*2*pi;
			//std::cout<<phi<<std::endl;
	  Double_t costheta = 1-2*q.Rndm();
  Double_t theta =acos(costheta);
  CM_dist.Fill(theta);
  //Fragment 1
  v1_cm[0] = v11*sin(theta)*cos(phi);
  v1_cm[1] = v11*sin(theta)*sin(phi);
  v1_cm[2] = v11*cos(theta);
  //Fragment 2
  v2_cm[0] = v22*sin(pi-theta)*cos(phi+pi);
  v2_cm[1] = v22*sin(pi-theta)*sin(phi+pi);
  v2_cm[2] = v22*cos(pi-theta);
  //LAB frame
  //*******************************velocities************************************
  //Fragment 1
  v1_lab[0] = v1_cm[0]/(gamma*(1+beta[2]*v1_cm[2]/c));
  v1_lab[1] = v1_cm[1]/(gamma*(1+beta[2]*v1_cm[2]/c));
  v1_lab[2] = (v1_cm[2]+beta[2]*c)/(1+beta[2]*v1_cm[2]/c);
	pz1 = v1_lab[2]*A1*0.0312; //GeV/c
	//std::cout<<pz1<<std::endl;
	Momentum_dist1->Fill(pz1);
	Totalmomentum_dist->Fill(pz1);
	E1 = sqrt(pow((pz1*c),2)+pow((A1*0.00103643*pow(c,2)),2)); //GeV
	TKEvM->Fill(A1,E1);
  //Fragment 2
  v2_lab[0] = v2_cm[0]/(gamma*(1+beta[2]*v2_cm[2]/c));
  v2_lab[1] = v2_cm[1]/(gamma*(1+beta[2]*v2_cm[2]/c));
  v2_lab[2] = (v2_cm[2]+beta[2]*c)/(1+beta[2]*v2_cm[2]/c);
	pz2 = v2_lab[2]*A2*0.0312;
	Momentum_dist2->Fill(pz2);
	Totalmomentum_dist->Fill(pz2);
	E2 = sqrt(pow((pz2*c),2)+pow((A2*0.00103643*pow(c,2)),2));
	TKEvM->Fill(A2,E2);
  //**********************Positions****************************************
  tof1=100.0/v1_lab[2];
  tof2=100.0/v2_lab[2];
  //Fragment 1
  pos1[0] = pos0[0]+v1_lab[0]*tof1;
  pos1[1] = pos0[1]+v1_lab[1]*tof1;
  pos1[2] = pos0[2]+v1_lab[2]*tof1;
  evnt.x1=pos1[0];
  evnt.y1=pos1[1];
  XY_dist->Fill(pos1[0],pos1[1]);
  zdist.Fill(pos1[2]/100);
  //Fragment 2
  pos2[0] =pos0[0]+ v2_lab[0]*tof2;
  pos2[1] =pos0[1]+v2_lab[1]*tof2;
  pos2[2] =pos0[2]+ v2_lab[2]*tof2;
  evnt.x2=pos2[0];
  evnt.y2=pos2[1];
  separation_dist.Fill(fabs(pos2[1]-pos1[1]));


  if((pos2[1]<0.25 && pos2[1]>-0.25) || (pos1[1]<0.25 && pos1[1]>-0.25)  ){

  		//if(pos1[1]<0.5 && pos1[1]>-0.5){  //fabs(pos2[1]-pos1[1])<=stripsize &&
    	//
  		//}
		XY_dist2->Fill(pos2[0],pos2[1]);
	    hi++;


  } else{

	  XY_dist3->Fill(pos2[0],pos2[1]);
	  Y_dist2->Fill(pos2[1]);

  }

  //if(pos2[1]<0.030 && pos2[1]>-0.020) ki++;
  //if(pos1[1]<0.030 && pos1[1]>-0.020) li++;
  //if(pos1[1]>0.030 && pos2[1]>0.020)  fi++;
  //if(pos1[1]<-0.030 && pos2[1]<-0.020) xi++;


  XY_dist->Fill(pos2[0],pos2[1]);

	Y_dist->Fill(pos2[1]);







//****************************************************************************************************************************************************************************

   //angles (theta) in LAB frame
  Double_t theta_lab1 = atan(sqrt(pow(v1_lab[0],2)+pow(v1_lab[1],2))/ v1_lab[2]);
  evnt.theta1_lab=theta_lab1;
  theta_1dist.Fill(theta_lab1);
  //theta_dist.Fill(theta_lab1);
  variable1=cos(theta_lab1);
  evnt.costheta_1=variable1;
  costheta_dist.Fill(variable1);
  ZetaTheta_dist.Fill(theta,Z1);
  if(Z1>46)
    costheta_disth.Fill(variable1);
  else
    costheta_distl.Fill(variable1);
  theta_his->Fill(theta_lab1);
 Double_t theta_lab2 = atan(sqrt(pow(v2_lab[0],2)+pow(v2_lab[1],2))/ v2_lab[2]);
  evnt.theta2_lab=theta_lab2;
  variable2=cos(theta_lab2);
  evnt.costheta_2=variable2;
  costheta_dist.Fill(variable2);
  if(Z2>46)
    costheta_disth.Fill(variable1);
  else
    costheta_distl.Fill(variable1);
  theta_2dist.Fill(theta_lab2);
  theta_1theta_2dist.Fill(theta_lab1,theta_lab2);
  theta_his->Fill(theta_lab2);
  //ZetaTheta_dist.Fill(theta_lab2,Z2);
  for(Int_t j=0;j<4;j++){
    size[j]=dist[j]*theta_lab1;
  }
  *out << " TKE [GeV] \t"<< TKE << endl;
  *out << "\t FRAGMENT 1 Z:"<< Z1 << endl;
  //  cout << "\t FRAGMENT 1 Z:"<< Z1 << endl;
  *out << "  N\t"<< N1 << endl;
  *out << "  A\t"<< A1 << endl;
  *out << "  velocity of fragment\t"<< v11 <<" cm/ns"<< endl;
  /*    *out << "  v1_cm[x] \t" << v1_cm[0]<<endl;
   *out << "  v1_cm[y] \t" << v1_cm[1]<<endl;
   *out << "  v1_cm[z] \t" << v1_cm[2]<<endl;*/
  *out << "  v1_lab[x] \t" << v1_lab[0]<<endl;
  *out << "  v1_lab[y] \t" << v1_lab[1]<<endl;
  *out << "  v1_lab[z] \t" << v1_lab[2]<<endl;
  *out << "  theta_lab1 \t" << theta_lab1<<endl;
  *out <<"Distance "<<dist[0]<<"m Size needed for the detector "<<2*size[0]<<"m"<<endl;
  *out <<"Distance "<<dist[1]<<"m Size needed for the detector "<<2*size[1]<<"m"<<endl;
  *out <<"Distance "<<dist[2]<<"m Size needed for the detector "<<2*size[2]<<"m"<<endl;
  *out <<"Distance "<<dist[3]<<"m Size needed for the detector "<<2*size[3]<<"m"<<endl;
  *out << "\t FRAGMENT 2 Z:"<< Z2 << endl;
  //cout << "\t FRAGMENT 2 Z:"<< Z2 << endl;
  *out << "  N\t"<< N2 << endl;
  *out << "  A\t"<< A2 << endl;
  *out << "  velocity of fragment\t"<< v22 <<" cm/ns" << endl;
  /* *out << "  v2_cm[x] \t" << v2_cm [0]<<endl;
   *out << "  v2_cm[y] \t" << v2_cm [1]<<endl;
   *out << "  v2_cm[z] \t" << v2_cm [2]<<endl;*/
  *out << "  v2_lab[x] \t" << v2_lab[0]<<endl;
  *out << "  v2_lab[y] \t" << v2_lab[1]<<endl;
  *out << "  v2_lab[z] \t" << v2_lab[2]<<endl;
  *out << "  theta_lab2 \t" << theta_lab2<<endl;
  *out << "**********************************************************"<<endl;
  *out << "**********************************************************"<<endl;
  //tree->Fill();

  //gcut1->SetVarY("pos1[0]");
 // gcut1->SetVarX("pos1[1]");
  //gcut2->SetVarY("pos1[0]");
  //gcut2->SetVarX("pos1[1]");
  //gcut3->SetVarY("pos1[0]");
  //gcut3->SetVarX("pos1[1]");

  //int inn1=gcut1->IsInside(pos1[0],pos1[1]);

   // if(inn1==1){
   //  j++;
   // }


}//} End of loop
cout<<"Number of fission events lost in cathode : "<<hi<< endl;
cout<<" Twin MUSIC efficiency : "<<(hi/2000000)*100<<endl;
//cout<<"Number of fission events fragment 1 lost in gap : "<<ki<< endl;
//cout<<"Number of fission events fragment 2 lost in gap : "<<li<< endl;
//cout<<"Number of fission events both fragments in paddles : "<<fi<< endl;

//cout<<" Number of lost fission events : "<<ki+li+fi+xi<< " ("<<((ki+li+fi+xi)/2000000)*100<<" %)"<<endl;
Double_t Prob=Double_t(Multihit)/Double_t(k);
//cout<<"Probability of Multihit: " <<Prob<<endl;
//cout<<"DONE"<<endl;
delete out;
gStyle->SetPalette(1);
//theta_1dist.Write();
//theta_2dist.Write();
//theta_his->Write();
//f->Write();
//f->Close();
//theta_1dist.Draw();
//ZetaTheta_dist.Draw();
//costheta_dist.Draw();
//win.cd(2);
//zdist.Draw();
//costheta_disth.Draw();
//win.cd(3);
//costheta_distl.Draw();
//theta_1theta_2dist.Draw("ZCOL");
//separation_dist.Draw();
//X0Y0_dist.Draw("ZCOL");
//win.cd(4);
//XY_dist.SetMarkerColor(kRed);
TCanvas *gc=new TCanvas();
	gc->Divide(2,2);
	gc->cd(1);
	//Zeta_dist.Draw();
	Y_dist->Draw();
	gc->cd(2);
	XY_dist->Draw("ZCOL");
	//gcut1->Draw("SAME");
	gc->cd(3);
	X0Y0_dist->Draw("ZCOL");

	TCanvas *f=new TCanvas();
	f->Divide(2,1);
	f->cd(2);

	Y_dist2->Draw();
	TF1 *fitFcn = new TF1("fitFcn",fitErf,-150,150,4);

	fitFcn->FixParameter(0,1330.1);
	fitFcn->FixParameter(1,-7.569);
	fitFcn->FixParameter(2,1.68639);
	fitFcn->FixParameter(3,7.56654);




	/*fitFcn->SetParameter(0,1500.0); //S0 Height
	 fitFcn->SetParameter(1,-100.0); // E2 Width
	 fitFcn->SetParameter(2,100.0); // E1 Width
	 fitFcn->SetParameter(3,34.42); // sqrt(2)*sigma*/
	//fitFcn->SetParameter(4,2.0);
	//fitFcn->SetParameter(5,-1.e-3);//Slope*/

	Y_dist2->Fit(fitFcn,"RV+");
	X0Y0_dist->Draw("ZCOL");
	f->cd(1);
	XY_dist3->Draw("COL");

	TCanvas *p_dist = new TCanvas();
	p_dist->Divide(3,2);
	p_dist->cd(1);
	Momentum_dist1->Draw();
	p_dist->cd(2);
	Momentum_dist2->Draw();
	p_dist->cd(3);
	Totalmomentum_dist->Draw();
	p_dist->cd(4);
	Mass_dist1->Draw();
	p_dist->cd(5);
	Mass_dist2->Draw();
	p_dist->cd(6);
	Totalmass_dist->Draw();

	TCanvas *EvM = new TCanvas();
	EvM->cd(1);
	//TKEvM->Draw();



	TFile *fil=new TFile("output.root","recreate");
	//gc->Write();
	delete fil;
//CM_dist.Draw();
//delete f;

}
