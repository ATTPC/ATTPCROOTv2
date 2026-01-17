{


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
};

const   Double_t MC15 = 931.478*15.0 + 9.8731;
const   Double_t Mproton = 931.478*1.0 + 7.289;
const   Double_t MC13 = 931.478*13.0 + 3.1250;
const   Double_t Mtriton = 931.478*3.0 + 14.9498;
const   Double_t MC12 = 931.478*12.0;
const   Double_t Mneuteron = 931.478*1.0 + 8.071;
    //Double_t MBe9 = 931.478*9.0 + 11.3485;
const   Double_t MHe4 = 931.478*4.0 + 2.4249;
const   Double_t MBe8 = 931.478*8.0 + 4.9417;
const   Double_t MBe10 = 931.478*10.0 + 12.6075;
const   Double_t MBe11 = 931.478*11.0 + 20.1772;
const   Double_t Mdeuteron = 931.478*2.0 + 13.1357;
const   Double_t MHe3 = 931.478*3.0 + 14.9312;
const   Double_t MLi11 = 931.478*11.0 + 40.7283;
const   Double_t MLi10 = 931.478*10.0 + 33.0526;
const   Double_t MLi9 = 931.478*9.0 + 24.9549;
const Double_t BM = 11.021661; //u;
const Double_t TM = 2.01410177785; //u;
const Double_t he3_mass =3.01602932197 *931.494; //u;
const Double_t RM =3.01602932197; //u;
const Double_t SM = 10.0354834; //u;



	TChain *fc = new TChain("tree_en");
	TChain *fc_get = new TChain("tree_get");
    TChain *fc_frib = new TChain("tree_frib");
	
	vector<int>not_run = {4046, 4047, 4048, 4049, 4050, 4059, 4060, 4061, 4062};
    TString ss;

	for(int run_num = 4063; run_num <= 4063; run_num ++){
		bool good_run = 1;
		for(int i = 0; i < int(not_run.size()); i++){
			if(run_num == not_run[i])good_run = 0;
		}
		if(good_run){
			ss.Form("/home/attpc/fair_install/ATTPCROOTv2-RCNPAnalysisBranch/macro/Unpack_HDF5/E581_RCNP/ghy/merge/merge_data/merge_%d.root",run_num);
			fc->Add(ss.Data());
            fc_get->Add(ss.Data());
            fc_frib->Add(ss.Data());
		}
	}
	fc->AddFriend(fc_get);
    fc->AddFriend(fc_frib);


    Double_t ppac_pos_cal[4][2], ppac_ang[2][2], vertex_z, vertex_x;
    fc->SetBranchAddress("ppac_pos_cal",ppac_pos_cal);
    fc->SetBranchAddress("ppac_ang", ppac_ang);
    fc->SetBranchAddress("vertex_z", &vertex_z);
    fc->SetBranchAddress("vertex_x", &vertex_x);

    TH2D *mean_diff = new TH2D("mean_diff","mean_diff", 100, 250, 2750, 100, -50, 50);

    Long64_t nentries = fc->GetEntries();
    for(Long64_t jentry = 0; jentry < nentries; jentry++){
        fc->GetEntry(jentry);
        
        for(int i = 0; i < 100; i ++){
            double length = 1250 + 5*i;//1557, 1250, 1750
            double mean_diff_x = ppac_pos_cal[3][0] + ppac_ang[1][0]*(length + vertex_z) - vertex_x;
            cout<<mean_diff_x<<" ";
            mean_diff->Fill(length, mean_diff_x);
            
        }
        if(jentry % 1000 == 0)cout<<jentry<<endl;
    }
    TCanvas *c1 = new TCanvas();
    mean_diff->Draw("colz");
    c1->Draw();

}
