void decay(){
	
	
	double m1=16.; // Mother mass
	
	double m3=4.0; // Daughter 1 mass
	double m4=10.; // Daughter 2 mass
	
	double ex1=0.0;
	
	double ex3=0.0;
	double ex4=0.0;
	
	double AngleD1[20000];
	double EnergyD1[20000];
	double AngleD2[20000];
	double EnergyD2[20000];
	
	double ebInput = 18.0; // Kin energy of the recoil
	
	ANGA1 = new double[2];
	ANGA2 = new double[2];
	
	for(int i = 0;i<2;i++){
		ANGA1[i] = 0.;
		ANGA2[i] = 0.;
	}
	


	const double U = 931.49401;
	const double rad2deg = 0.0174532925;
	const double PI=3.14159265358979323846;
	
	double wm1=m1*U+ex1; // Rest mass of the mother nucleus M P=(E,0) in CMS
	
	double wm3=m3*U+ex3;
	double wm4=m4*U+ex4;
	
	double thetacm1Input = 0.0;
	int aux=0;
	
	double E3 = ((wm1*wm1) + (wm3*wm3) - (wm4*wm4))/(2.0*wm1); // Energy of daughter 1 in CMS
	double E4 = ((wm1*wm1) + (wm4*wm4) - (wm3*wm3))/(2.0*wm1); // Energy of daughter 2 in CMS 
	//double P =  1/(2.0*wm1) ; // CM momentum
	//double P2 = (wm1*wm1) - ((wm3-wm4)*(wm3-wm4));
	//double P3 = (wm1*wm1) - ((wm3+wm4)*(wm3+wm4));
	//double Ptot=P*sqrt(P2*P3);
	double Ptot= (1/(2.0*wm1)) * sqrt ((wm1*wm1 + wm3*wm3 - wm4*wm4)*(wm1*wm1 + wm3*wm3 - wm4*wm4)-4*wm1*wm1*wm3*wm3);
	//cout<< wm1<<endl;
	
	double eb=ebInput+wm1; // This is the kinetic energy of the recoil nucleus, the total energy includes the mass
	//cout<<ebInput<<endl;
	double pb=sqrt((eb*eb)-(wm1*wm1)); // Momentum in the lab system of the recoil nucleus
	double beta=pb/eb;
	double gamma=eb/wm1;
	//double gamma=1.0/sqrt(1.0-beta*beta);
	//cout<<gamma<<endl;
	
	
	for(Float_t jj=0;jj<=180;jj=jj+0.01){

		thetacm1Input=jj;
		aux++;
	
	double thetacm1=thetacm1Input*rad2deg;  // degree to radian
	double thetacm2=PI-thetacm1;
	//cout<<ebInput<<endl;
		double p3_cmx = Ptot*sin(thetacm1);
		double p3_cmz = Ptot*cos(thetacm1);
		double p3_labx = p3_cmx;
		double p3_labz = gamma*(p3_cmz+(beta*E3));
		double p3_lab = sqrt(p3_labx*p3_labx+p3_labz*p3_labz);
		//cout<<p3_lab<<" "<<thetacm1<<endl;
		
		ANGA1[1]= (gamma*(E3+(beta*p3_cmz)))-wm3;
		//ANGA1[1]=sqrt(p3_lab*p3_lab+wm3*wm3)-wm3;
		//cout<<" Anga1  "<<p3_lab<<endl;
		
		double p4_cmx = Ptot*sin(thetacm2);
		double p4_cmz = Ptot*cos(thetacm2);
		double p4_labx = p4_cmx;
		double p4_labz = gamma*(p4_cmz+(beta*E4));
		double p4_lab = sqrt(p4_labx*p4_labx+p4_labz*p4_labz);
		
		ANGA2[1]= (gamma*(E4+(beta*p4_cmz)))-wm4;				 
		//ANGA2[1] = sqrt(p4_lab*p4_lab+wm4*wm4)-wm4;		// Kinetic energy, T + m = sqrt(p^2 + m^2)
		//cout<<"Decay "<<p4_cmx<<endl;
		
		
		double tg_thetalab1=p3_labx/p3_labz;
		//cout<<thetacm1<<"   "<<tg_thetalab1<<endl;
		
		if(tg_thetalab1>=1.0e6){
			ANGA1[0]=PI/2;
		}
		else{
			ANGA1[0]=atan(tg_thetalab1);
		}
		
		if(ANGA1[0]<0.0) ANGA1[0]=PI+ANGA1[0];
		
		double tg_thetalab2=p4_labx/p4_labz;
		
		if(tg_thetalab2>1.0e6){
			ANGA2[0]=PI/2.0;
		}
		else{
			ANGA2[0]=atan(tg_thetalab2);
		}
		
		if(ANGA2[0]<0.0) ANGA2[0]=PI+ANGA2[0];

		AngleD1[aux]=ANGA1[0]*180/TMath::Pi();
		EnergyD1[aux]=ANGA1[1];
		AngleD2[aux]=ANGA2[0]*180/TMath::Pi();
		EnergyD2[aux]=ANGA2[1];
		
		
	}
	
	TGraph *g5=new TGraph(aux,AngleD1,EnergyD1);
	//TGraph *g6=new TGraph(aux,ThetaCMdec,AngleD2);
	TGraph *g4=new TGraph(aux,AngleD2,EnergyD2);
	
	
	TCanvas* c2=new TCanvas();
	c2->Divide(2,2);
	c2->Draw();
	c2->cd(1);
	g4->Draw("AC");
	c2->cd(2);
	g5->Draw("AC");
	c2->cd(3);
	//g6->Draw("AC");
	

}