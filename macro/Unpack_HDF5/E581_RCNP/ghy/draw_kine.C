#include <vector>

const	Double_t MC15 = 931.478*15.0 + 9.8731;
const   Double_t Mproton = 931.478*1.0 + 7.289;
const   Double_t MC13 = 931.478*13.0 + 3.1250;
const   Double_t Mtriton = 931.478*3.0 + 14.9498;
const   Double_t MC12 = 931.478*12.0;
const   Double_t Mneuteron = 931.478*1.0 + 8.071;
    //Double_t MBe9 = 931.478*9.0 + 11.3485;
const   Double_t MHe4 = 931.478*4.0 + 2.4249;
const   Double_t MBe8 = 931.478*8.0 + 4.9417;
const 	Double_t MBe10 = 931.478*10.0 + 12.6075;
const 	Double_t MBe11 = 931.478*11.0 + 20.1772;
const 	Double_t Mdeuteron = 931.478*2.0 + 13.1357;
const 	Double_t MHe3 = 931.478*3.0 + 14.9312;
const 	Double_t MLi11 = 931.478*11.0 + 40.7283;
const 	Double_t MLi10 = 931.478*10.0 + 33.0526;
const   Double_t MLi9 = 931.478*9.0 + 24.9549;
const Double_t BM = 11.021661; //u;
const Double_t TM = 2.01410177785; //u;
const Double_t he3_mass =3.01602932197 *931.494; //u;
const Double_t RM =3.01602932197; //u;
const Double_t SM = 10.0354834; //u;

Double_t ExEne(Double_t BM,Double_t  TM, Double_t RM, Double_t SM, Double_t K1, Double_t K3, Double_t theta){
	const Double_t unit = 931.494102;
	
	Double_t m1 = BM * unit;
	Double_t m2 = TM * unit;
	Double_t m3 = RM * unit;
	Double_t m4 = SM * unit;
	
	Double_t E1 = m1 + K1;
	Double_t E3 = m3 + K3;

	Double_t p1 = sqrt(pow(E1,2) - pow(m1,2));
	Double_t p3 = sqrt(pow(E3,2) - pow(m3,2));

	Double_t thetalab = (TMath::Pi() * theta)/180;
	Double_t s = pow(m1,2) + pow(m2,2) + 2*m2*E1;
	Double_t t = pow(m1,2) + pow(m3,2) + 2*(p1*p3*cos(thetalab) - E1*E3);
	Double_t u = pow(m2,2) + pow(m3,2) - 2*m2*E3;

	Double_t Ex = sqrt(s + t + u - pow(m1,2) - pow(m2,2) - pow(m3,2)) - m4;
	
	return Ex;
}

Double_t calib_beam_energy(TGraph * dedx_beam, Double_t beam_energy, Double_t beam_range){
	//beam energy calibration
	//beam_energy: MeV/u
	//beam_range: mm
	Double_t calib_energy = 0;
	Double_t step = 0.01; //mm

	TSpline3 *spline_beam = new TSpline3("spline_beam", dedx_beam);
	while(beam_range>0){
		Double_t dedx = spline_beam->Eval(beam_energy);
		beam_energy = beam_energy - dedx*step;
		beam_range = beam_range-step;	 //0.01mm

	}

	calib_energy = beam_energy;
	return calib_energy;
}

TGraph * read_dedx_from_lise(string inputFile, int A){// given the elose file from lise, particle mass number A, return e_x_graph, e: MeV, x: mg/cm2 or micron
  ifstream inFile(inputFile);
  if (!(inFile.is_open())) {
    cout << "Cannot find input file :" <<inputFile << endl;
    return 0;
  }
  TGraph *gEdEdx = new TGraph;
  string str;
  for(int i=0;i<3;i++)
    getline(inFile, str); // skip the first three lines

  // energy: MeV/u, Range: micron - do not depends on A.

  double energy, R[10], dummy;
  int n=0;
  while (getline(inFile,str) )
    {
        stringstream ss(str);
        ss  >> energy >> R[0]
            >> dummy  >> R[1]
            >> dummy  >> R[2]
            >> dummy  >> R[3]
            >> dummy  >> R[4]  /* used for calculation */
            >> dummy  >> R[5];

        gEdEdx->SetPoint(n++, energy*A, R[4]*1000);
        //cout<<energy*4<<" "<<R[4]<<endl;
    }
  inFile.close();
  //cout << "Read " << n << " lines of data from the file: " << inputFile << endl;
  //cout << "TGraph: x = E(MeV/u), y = dEdx(MeV/um) " << endl;
  return gEdEdx;
}


TGraph* generate_particle(Double_t Ekb, Double_t mbc2, Double_t mtc2, Double_t mec2, Double_t mrc2, Double_t theta_c_min, Double_t theta_c_max){
	
    Double_t Eb = Ekb + mbc2;
    Double_t Et = mtc2;

    Double_t beta_c = sqrt(Eb*Eb - mbc2*mbc2)/(Ekb + mbc2 + mtc2);
    Double_t gamma_c = 1/sqrt(1 - beta_c*beta_c);

    
    Double_t E_relc = sqrt((Eb + Et)*(Eb + Et) - (Eb*Eb - mbc2*mbc2));

    // p_ec : p_ec*c
    Double_t p_ec = sqrt((E_relc - mec2 -mrc2)*(E_relc - mec2 + mrc2)*(E_relc + mec2 + mrc2)*(E_relc + mec2 - mrc2))/(2.0*E_relc);
    Double_t E_ec = sqrt(p_ec*p_ec + mec2*mec2);

	TGraph *e_theta_plot = new TGraph();

	for(int i = 0; i < 500; i++){

		Double_t theta_ec = (theta_c_min + (theta_c_max - theta_c_min)*i/500)*TMath::Pi()/180.0;
		Double_t Ee = gamma_c*E_ec + gamma_c*beta_c*p_ec*cos(theta_ec);
			
		Double_t Eke = Ee - mec2;
		Double_t theta_el = atan(p_ec*sin(theta_ec)/(gamma_c*p_ec*cos(theta_ec) + gamma_c*beta_c*E_ec))*180.0/TMath::Pi();

		if(theta_el < 0)theta_el += 180.0;
		e_theta_plot->SetPoint(i, theta_el, Eke);
		//cout<<"theta : "<<theta_el <<" E : "<<Eke<<endl;
	}

    return e_theta_plot;
}


void draw_kine(){
	TCanvas *c1 = new TCanvas();
	TChain *fc = new TChain("tree");
	TString ss;
	for(int run_num = 4024; run_num <= 4045; run_num ++){
		ss.Form("../transfer_data/simple_%d.root",run_num);
		fc->Add(ss.Data());
	}

	TGraph *e_theta_Be11_d3He_Li10_0 = generate_particle(23.5*11, MBe11, Mdeuteron, MHe3, MLi10, 0, 180);
	TGraph *e_theta_Be11_d3He_Li10_1 = generate_particle(23.5*11, MBe11, Mdeuteron, MHe3, MLi10 + 1.0, 0, 180);
	TGraph *e_theta_Be11_d3He_Li10_2 = generate_particle(23.5*11, MBe11, Mdeuteron, MHe3, MLi10 + 2.0, 0, 180);	
	
	TGraph *e_theta_Be11_dd_Be11 = generate_particle(23.5*11, MBe11, Mdeuteron, Mdeuteron, MBe11, 0, 180);
	TGraph *e_theta_Be11_dd_Be11_5 = generate_particle(23.5*11, MBe11, Mdeuteron, Mdeuteron, MBe11 +5, 0, 180);
	TGraph *e_theta_Be11_dd_Be11_10 = generate_particle(23.5*11, MBe11, Mdeuteron, Mdeuteron, MBe11 +10, 0, 180);
	
	TGraph *e_theta_Be11_dt_0 = generate_particle(23.5*11, MBe11, Mdeuteron, Mtriton, MBe10, 0, 180);

	TGraph *e_theta_Li9_dd_gs = generate_particle(25.5*9, MLi9, Mdeuteron, Mdeuteron, MLi9, 0, 180);
	TGraph *e_theta_Li9_dd_2691 = generate_particle(25.5*9, MLi9, Mdeuteron, Mdeuteron, MLi9 + 2.691, 0, 180);
	TGraph *e_theta_Li9_dd_4301 = generate_particle(25.5*9, MLi9, Mdeuteron, Mdeuteron, MLi9 + 4.301, 0, 180);
	TGraph *e_theta_Li9_dd_5380 = generate_particle(25.5*9, MLi9, Mdeuteron, Mdeuteron, MLi9 + 5.380, 0, 180);

	TGraph *e_theta_Li9_dd_16000 = generate_particle(25.5*9, MLi9, Mdeuteron, Mdeuteron, MLi9 + 16.0, 0, 180);
	
	TGraph *e_theta_Li9_dp_0 = generate_particle(25.5*9, MLi9, Mdeuteron, Mproton, MLi10, 0, 180);
	TGraph *e_theta_Li9_dp_5 = generate_particle(25.5*9, MLi9, Mdeuteron, Mproton, MLi10 + 5, 0, 180);

	gROOT->Macro("p_cut.C");
	gROOT->Macro("d_cut.C");
	gROOT->Macro("He_cut.C");
	gROOT->Macro("Li_cut.C");
	gROOT->Macro("Be11_cut.C");
	gROOT->Macro("d_cut_newpar.C");


	fc->Draw("kineE_d[0] : thetaLab[0]>>(200, 0, 100, 200, 0, 8)","d_cut && Be11_cut && SiFn1 == 1 && SiFn2 == 1 && punch_through[0] == 0","colz");
	e_theta_Be11_dd_Be11->Draw("Lsame");
	//e_theta_Be11_dt_0->Draw("Lsame");
	e_theta_Be11_dd_Be11_5->Draw("Lsame");
	e_theta_Be11_dd_Be11_10->Draw("Lsame");
	
/*	fc->Draw("kineE_d[0] : thetaLab[0]>>(200, 0, 100, 200, 0, 15)","d_cut && Li_cut","colz");
	
	e_theta_Li9_dd_gs->Draw("Lsame");
	e_theta_Li9_dd_2691->Draw("Lsame");
	e_theta_Li9_dd_4301->Draw("Lsame");
	e_theta_Li9_dd_5380->Draw("Lsame");
	e_theta_Li9_dd_16000->Draw("Lsame");
	*/
	
	/*fc->Draw("kineE_p[0] : thetaLab[0]>>(200, 0, 180, 200, 0, 10)","(p_cut || d_cut) && Li_cut && SiFn1 == 1 && SiFn2 == 1 && punch_through[0] == 0","colz");
	e_theta_Li9_dp_0->Draw("Lsame");
	e_theta_Li9_dp_5->Draw("Lsame");
	c1->Draw();
*/
}
