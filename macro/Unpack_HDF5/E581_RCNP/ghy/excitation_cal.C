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

Double_t ExEne(Double_t MB,Double_t  MT, Double_t MR, Double_t MS, Double_t EkB, Double_t EkR, Double_t theta){
	const Double_t unit = 931.494102;
	
	/*Double_t m1 = BM * unit;
	Double_t m2 = TM * unit;
	Double_t m3 = RM * unit;
	Double_t m4 = SM * unit;
	Double_t E1 = m1 + K1;
	Double_t E3 = m3 + K3;
	*/
	Double_t m1 = MB; Double_t m2 = MT;
	Double_t m3 = MR; Double_t m4 = MS;
	Double_t E1 = MB + EkB;
	Double_t E3 = MR + EkR;

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

Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

std::tuple<double, double>
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject)
{

   // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
   double Et1 = K_proj + m1;
   double Et2 = m2;
   double Et3 = K_eject + m3;
   double Et4 = Et1 + Et2 - Et3;
   double m4_ex, Ex, theta_cm;
   double s, t, u; //---Mandelstam variables

   s = pow(m1, 2) + pow(m2, 2) + 2 * m2 * Et1;
   u = pow(m2, 2) + pow(m3, 2) - 2 * m2 * Et3;

   m4_ex = sqrt((cos(thetalab) * omega(s, pow(m1, 2), pow(m2, 2)) * omega(u, pow(m2, 2), pow(m3, 2)) -
                 (s - pow(m1, 2) - pow(m2, 2)) * (pow(m2, 2) + pow(m3, 2) - u)) /
                   (2 * pow(m2, 2)) +
                s + u - pow(m2, 2));
   Ex = m4_ex - m4;

   t = pow(m2, 2) + pow(m4_ex, 2) - 2 * m2 * Et4;

   // for inverse kinematics Note: this angle corresponds to the recoil
    theta_cm = TMath::Pi() - acos((pow(s, 2) + s * (2 * t - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m3, 2) - pow(m4_ex, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m3, 2), pow(m4_ex, 2))));

   /*theta_cm = acos((pow(s, 2) + s * (2 * u - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m4_ex, 2) - pow(m3, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m4_ex, 2), pow(m3, 2))));*/

   theta_cm = theta_cm * TMath::RadToDeg();
   return std::make_tuple(Ex, theta_cm);
}

void excitation_cal(){
	TCanvas *c1 = new TCanvas();
	TChain *fc = new TChain("tree");
	TChain *fc_gagg = new TChain("tree");

	TString ss;
	for(int run_num = 4024; run_num <= 4045; run_num ++){
		ss.Form("../transfer_data/simple_%d_test.root",run_num);
		fc->Add(ss.Data());
                ss = TString::Format("/data/sustech/user/ghy/frib-decode/data/gagg_%04d.root", run_num);
                cout<<ss.Data()<<endl;
                fc_gagg->Add(ss.Data());
	}
	fc->AddFriend(fc_gagg);

	double SiFe1;
	double SiFe2;
	int SiFn1;
	int SiFn2;
	double e_g1[25];
	int id_g1[25];

	double first_z[10], angle[10], Ek[10];
	double dedx[10], range[10];
	UChar_t punch_through[10];
	
	fc->SetBranchAddress("SiFe1", &SiFe1);
	fc->SetBranchAddress("SiFe2", &SiFe2);
	fc->SetBranchAddress("SiFn1", &SiFn1);
	fc->SetBranchAddress("SiFn2", &SiFn2);
	fc->SetBranchAddress("first_z", first_z);
	fc->SetBranchAddress("thetaLab", angle);
	fc->SetBranchAddress("kineE_d", Ek);
	fc->SetBranchAddress("range", range);
	fc->SetBranchAddress("dedx", dedx);
	fc->SetBranchAddress("punch_through", punch_through);
	fc->SetBranchAddress("e_g1", e_g1);
	fc->SetBranchAddress("id_g1", id_g1);

	gROOT->Macro("p_cut.C");
	gROOT->Macro("d_cut_newpar.C");
	gROOT->Macro("d_cut.C");

	gROOT->Macro("He_cut.C");
	gROOT->Macro("Li_cut.C");
	gROOT->Macro("Be11_cut.C");
	gROOT->Macro("Be10_gagg_cut.C");
	gROOT->Macro("Be11_gagg_cut.C");
	
	TCutG *d_cut = (TCutG *)gROOT->FindObject("d_cut_newpar");
	TCutG *Be11_cut = (TCutG *)gROOT->FindObject("Be11_cut");
	
	TCutG *Be11_gagg = (TCutG *)gROOT->FindObject("Be11_gagg_cut");
	TCutG *Be10_gagg = (TCutG * )gROOT->FindObject("Be10_gagg_cut");

	TGraph *dedx_Be11_C3D8 = read_dedx_from_lise("../elosslise/dedx_Be11_C3D8_466torr.txt", 11);
	TH1D *ex_Be11_dd = new TH1D("ex_Be11_dd", "ex_Be11_dd", 100, -5, 15);
	TH1D *e_x_Be11_Be10 = new TH1D("ex_Be11_Be10", "ex_Be11_Be10", 60, -5, 15);

	TH2D *ex_energy = new TH2D("ex_energy", "ex_energy", 10, 24, 28, 60, -5, 15);
	Long64_t nentries = fc->GetEntries();
	cout<<"entry number : "<<nentries<<endl;
	for(Long64_t jentry = 0; jentry < nentries; jentry ++){
		fc->GetEntry(jentry);
		if(punch_through[0] == 0 && Be11_cut->IsInside(SiFe2 , SiFe1)&& d_cut->IsInside(range[0], dedx[0]) &&  SiFn1 == 1 && SiFn2 == 1){
			double beam_range = 1000 - first_z[0];
			double beam_energy = 26.2*11;
			//double beam_e = 0;
		//	double e_x_dd = 0;
			/*for(int i = 0; i < 10 ; i++){
				beam_energy = (24.0 + 4.0*i/10);
				beam_e = calib_beam_energy(dedx_Be11_C3D8, beam_energy*11.0, beam_range);
				e_x_dd = ExEne(MBe11, Mdeuteron, Mdeuteron, MBe11, beam_e , Ek[0] , angle[0]);
				ex_energy->Fill(beam_energy, e_x_dd);
			}*/
			
			double beam_e = calib_beam_energy(dedx_Be11_C3D8, beam_energy, beam_range);
			double e_x_dd = ExEne(MBe11, Mdeuteron, Mdeuteron, MBe11, beam_e , Ek[0] , angle[0]);
			if(Be11_gagg->IsInside(e_g1[0], SiFe2))ex_Be11_dd->Fill(e_x_dd);
			else if(Be10_gagg->IsInside(e_g1[0], SiFe2))e_x_Be11_Be10->Fill(e_x_dd);
			else{
				//ex_Be11_dd->Fill(e_x_dd);
			}
		}

		if(jentry % 10000 == 1)cout<<"processing : "<<jentry<<endl;
	}

	ex_Be11_dd->Draw();
	c1->Draw();
	TCanvas *c2 = new TCanvas();
	c2->cd();
	e_x_Be11_Be10->SetLineColor(kRed);
	e_x_Be11_Be10->Draw();
	c2->Draw();
	
//	ex_energy->Draw("colz");
//	c1->Draw();

	
}
