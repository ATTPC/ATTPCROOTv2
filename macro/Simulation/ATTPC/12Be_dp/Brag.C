#include "catima/catima.h"
#include "mystyle.h"

double ExEne(double IM, double TM, double AS, double AR, double K1,double K3, double theta);
double Range(double posx_1st, double posy_1st, double posz_1st, double posx_last, double posy_last, double posz_last);
double Projection(double x_mea, double y_mea, double p0, double p1, double p2, double x_limit, double step);

void Brag()
{
	mystyle();
	const Double_t rad = 57.2958;
	//const char *filepng = "13Be_p_0_22";
	const char *filepng = "total_brag";
	const double TMD    = 2.014101;
	const double IMBe   = 12.026922;
	const double ASD    = 1.007825;
	const double ARBe   = 13.036134;
	const double Kinematics2   = 0;
	//const double Beam   = 30.30799;
	const double Beam_ene   = 18 * IMBe;
	const double ZBe   = 4;
	const double density_450  = 0.0012836;//g/cm3

   TCutG *cutg_error = new TCutG("cutg_error",8);
   cutg_error->SetVarX("");
   cutg_error->SetVarY("");
   cutg_error->SetTitle("Graph");
   cutg_error->SetFillStyle(1000);
   cutg_error->SetPoint(0,286.17,5287.18);
   cutg_error->SetPoint(1,224.218,3306.64);
   cutg_error->SetPoint(2,311.827,1304.57);
   cutg_error->SetPoint(3,490.801,2531.65);
   cutg_error->SetPoint(4,443.867,6492.72);
   cutg_error->SetPoint(5,325.594,6191.34);
   cutg_error->SetPoint(6,325.594,6191.34);
   cutg_error->SetPoint(7,286.17,5287.18);
   cutg_error->Draw("d");

   //cut
   TCutG *cutg_13Beq = new TCutG("cutg_13Beq",8);
   cutg_13Beq->SetVarX("h2d_kine");
   cutg_13Beq->SetVarY("");
   cutg_13Beq->SetTitle("Graph");
   cutg_13Beq->SetFillStyle(1000);
   cutg_13Beq->SetPoint(0,105.894,3434.47);
   cutg_13Beq->SetPoint(1,122.695,1969.71);
   cutg_13Beq->SetPoint(2,139.496,937.716);
   cutg_13Beq->SetPoint(3,153.98,1536.94);
   cutg_13Beq->SetPoint(4,133.413,3734.08);
   cutg_13Beq->SetPoint(5,116.033,4932.53);
   cutg_13Beq->SetPoint(6,116.033,4932.53);
   cutg_13Beq->SetPoint(7,105.894,3434.47);
   cutg_13Beq->Draw("d");

   TCutG *cutg_13Bee = new TCutG("cutg_13Bee",9);
   cutg_13Bee->SetVarX("h2d_energy");
   cutg_13Bee->SetVarY("");
   cutg_13Bee->SetTitle("Graph");
   cutg_13Bee->SetFillStyle(1000);
   cutg_13Bee->SetPoint(0,110.819,4.13357);
   cutg_13Bee->SetPoint(1,107.053,2.96841);
   cutg_13Bee->SetPoint(2,126.461,1.76997);
   cutg_13Bee->SetPoint(3,138.627,1.03759);
   cutg_13Bee->SetPoint(4,150.504,1.37049);
   cutg_13Bee->SetPoint(5,146.449,2.73538);
   cutg_13Bee->SetPoint(6,119.219,4.49976);
   cutg_13Bee->SetPoint(7,119.219,4.49976);
   cutg_13Bee->SetPoint(8,110.819,4.13357);
   cutg_13Bee->Draw("d");
   
   TCutG *cutg_13Ber = new TCutG("cutg_13Ber",11);
   cutg_13Ber->SetVarX("h2d_pid");
   cutg_13Ber->SetVarY("");
   cutg_13Ber->SetTitle("Graph");
   cutg_13Ber->SetFillStyle(1000);
   cutg_13Ber->SetPoint(0,104.156,145.784);
   cutg_13Ber->SetPoint(1,128.199,27.6042);
   cutg_13Ber->SetPoint(2,160.353,25.1074);
   cutg_13Ber->SetPoint(3,168.753,77.5393);
   cutg_13Ber->SetPoint(4,157.746,124.145);
   cutg_13Ber->SetPoint(5,134.572,195.719);
   cutg_13Ber->SetPoint(6,121.826,278.112);
   cutg_13Ber->SetPoint(7,108.501,225.68);
   cutg_13Ber->SetPoint(8,103.287,170.752);
   cutg_13Ber->SetPoint(9,103.287,170.752);
   cutg_13Ber->SetPoint(10,104.156,145.784);
   cutg_13Ber->Draw("d");

  TF1 *fexp = new TF1("fexp","[0]*exp([1]*x)+[2]");
  TF1 *fx_1 = new TF1("fx_1","[0]/(x+[1])+[2]");
  TF1 *fx = new TF1("fx","x",0,300);
  TF1 *fx2 = new TF1("fx2","[0]*x^2+[1]*x+[2]");
  TF1 *fx3 = new TF1("fx3","[0]*x^3+[1]*x^2+[2]*x+[3]");
  TH1D *h1d_proq = new TH1D("h1d_proq","projection",100,-50,50);
  TH1D *h1d_proe = new TH1D("h1d_proe","projection",120,-5,5);
  TH1D *h1d_ex = new TH1D("h1d_ex","Excitaion energy",200,-10,10);
  TH1D *h1d_range = new TH1D("h1d_range","Projection",100,-50,50);
  TH1D *h1d_beam_ene = new TH1D("h1d_beam_ene","Position z",100,0,1000);
  TH1D *h1d_eloss = new TH1D("h1d_eloss","",100,1,100);
  TH2D *h2d_cut = new TH2D("h2d_cut","energy_angle",2000,0,200,200,0,10); 
  TH2D *h2d_cutq = new TH2D("h2d_cutq","charge_angle",800,0,200,1000,0,10000); 
  TH2D *h2d_pid = new TH2D("h2d_pid","h2d_pid",400,0,200,250,0,500); 
  TH2D *h2d_pid_brag = new TH2D("h2d_pid_brag","h2d_pid",400,0,200,250,0,500); 
  TH2D *h2d_range = new TH2D("h2d_range","h2d_range",400,0,200,250,0,500); 

  TH2D *h2d_range_q = new TH2D("h2d_range_q","",250,0,500,500,0,20000); 
  TH2D *h2d_range_brag_q = new TH2D("h2d_range_brag_q","",250,0,500,500,0,20000); 

  TH2D *h2d_range_range_brag = new TH2D("h2d_range_range_brag","",300,0,300,300,0,300); 
  TH2D *h2d_minus = new TH2D("h2d_minus","",300,0,300,110,-10,100); 
  
  TH2D *h2d_eloss = new TH2D("h2d_eloss","",300,0,300,200,0,50);
  TH2D *h2d_Brag = new TH2D("h2d_Brag","h2d_Brag",300,0,300,100,0,100); 

  TGraph *g_line = new TGraph(); 
   
  FairRunAna *run = new FairRunAna();
  
  std::ifstream infile("Be12_C3D8.txt");
        std::string line;
		int i = 0;
	    // 读取每一行数据
	    double x,y,ang,ene;
	    while (std::getline(infile, line)) {
	        std::istringstream iss(line);
	        infile >> x >> y/* >> ang >> ene*/;
		    g_line->SetPoint(i++,x,y);
	    }
   TH2D *h2d_kine = new TH2D("h2d_kine","h2d_kine",800,0,200,1000,0,20000);
   h2d_kine->GetXaxis()->SetTitle("Angle (Lab)");
   h2d_kine->GetYaxis()->SetTitle("Charge");
   h2d_kine->GetXaxis()->CenterTitle();
   h2d_kine->GetYaxis()->CenterTitle();
   TH2D *h2d_energy = new TH2D("h2d_energy","h2d_energy",1600,0,200,1000,0,20);
   h2d_energy->GetXaxis()->SetTitle("Angle (Lab)");
   h2d_energy->GetYaxis()->SetTitle("Energy (MeV)");
   h2d_energy->GetXaxis()->CenterTitle();
   h2d_energy->GetYaxis()->CenterTitle();
   
   TGraph *g_kine = new TGraph();
   TGraph *g_ene = new TGraph();
   

   std::ofstream hitsFile;
   hitsFile.open("hits.txt");

   //TString infiledir = "./Output/";
   TString infiledir = "./";
   //TString digifile = infiledir + TString::Format("output_digi_rcnp_13Be_p_0.0_22.0_hole.root");
   TString digifile = infiledir + TString::Format("output_digi_rcnp_13Be_p_18.0_18.0_hole.root");
   TFile *file = new TFile(digifile.Data(), "READ");
   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   //std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader Reader("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventHArray(Reader, "AtEventH");
   TTreeReaderValue<TClonesArray> patternArray(Reader, "AtPatternEvent");

   for (Int_t ievent = 0; ievent < 10000/*nEvents*/; ievent++) {
      if(ievent%(nEvents/10) == 0){
		printf("\n Complete %.0f%% (evt %d) ...\n", ievent * 100./nEvents, ievent);
      }
      Reader.Next();

      AtEvent *event = (AtEvent *)eventHArray->At(0);
      AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);

      if (event && patternEvent) {

         auto &hitArray = event->GetHits();
         auto &tracks = patternEvent->GetTrackCand();
         //std::cout << " Number of hits : " << hitArray.size() << std::endl;
         //std::cout << " Number of tracks : " << tracks.size() << std::endl;

   	Double_t theta = 0;
    Double_t phi = 0;
    Double_t energy = 0;
   	Int_t track_num = 0;
	Double_t range_brag = 0;
	Double_t beam_ene_remain = 216;
         for (auto &track : tracks) {
            auto &points = track.GetHitArray();
	    theta = track.GetGeoTheta();
	    if(theta < 1.570795){
	    	theta = TMath::Pi() - theta;
	    }
	    phi = track.GetGeoPhi();
	    track_num = track.GetTrackID();
            //std::cout << "Track ID: " << track.GetTrackID() << std::endl;
   	    
            Int_t beam_i = 0; 
	    Double_t Q_total = 0;
	    Double_t posx[1000] = {0};
	    Double_t posy[1000] = {0};
	    Double_t posz[1000] = {0};
	    Double_t charge_step[1000] = {0};
	    Double_t posbeamz_1st = 0;
	    Double_t beam_range = 0;
	    Int_t cts = 0;
	    for (auto &point : points){
               auto pos = point->GetPosition();
               auto charge = point->GetCharge();
               auto time = point->GetTimeStamp();
	    	//cout<<" Event = "<<ievent<<", track = "<<track_num<<", charge = "<<charge<<endl;
	       
	           charge_step[beam_i] = charge;	
	           posx[beam_i] = pos.X();
	           posy[beam_i] = pos.Y();	
	           posz[beam_i] = pos.Z();
	           beam_i += 1;
	       	   Q_total +=charge;
	           energy = Q_total * 0.0009815 + 0.0959; 
            }
	    Double_t range = Range(posx[0],posy[0],posz[0],posx[beam_i-1],posy[beam_i-1],posz[beam_i-1]);	//range of last point.

	    Double_t charge = charge_step[0];
	    Double_t charge_tot = 0;


        Double_t max_slope = 0;  //最大斜率
        Int_t end_index = 0;

        Int_t window_size = 5;
        Double_t smoothed_slope[beam_i];

	    for(Int_t i = 1; i< beam_i; i++){

		    charge = charge_step[i];
		    charge_tot += charge_step[i];

            //Double_t delta_charge = charge_step[i] - charge_step[i-1];
            Double_t delta_charge = charge_step[i];
            Double_t delta_range = Range(posx[i], posy[i], posz[i], posx[i - 1], posy[i - 1], posz[i - 1]);
            Double_t range_range = Range(posx[i], posy[i], posz[i], posx[0], posy[0], posz[0]);

            smoothed_slope[i] = delta_charge / delta_range;
            h1d_eloss->Fill( delta_charge / delta_range );
            Double_t slope = delta_charge / delta_range;

	    	if(/*ievent == 5 && */track_num == 2){
			    h2d_eloss->Fill(range_range, charge_tot/range_range);
			    //h2d_Brag->Fill(range_range, delta_charge/delta_range);
			    h2d_Brag->Fill(range_range, delta_charge/(posz[i]-posz[i-1]));
                //cout<<"range_range = "<<range_range<<", delta_charge = "<<delta_charge/delta_range<<endl;
	    	}
            else continue;
	    }

        for (Int_t i = 1; i < beam_i; i++) {
	    	//cout<<" Event = "<<ievent<<", track_num = "<<track_num<<", step_num = "<<i<<endl;
            Double_t delta_charge = charge_step[i] - charge_step[i - 1];
            Double_t delta_range = Range(posx[i], posy[i], posz[i], posx[i - 1], posy[i - 1], posz[i - 1]);
            smoothed_slope[i] = delta_charge / delta_range;
        }

        for (Int_t i = window_size; i < beam_i - window_size; i++) {
            if (smoothed_slope[i] > max_slope) {
                  max_slope = smoothed_slope[i];
                  end_index = i;
             }
        }
        
        Double_t range_at_max_slope = Range(posx[0], posy[0], posz[0], posx[end_index], posy[end_index], posz[end_index]);

	    
	    if(ievent%2 == 0){
	    	beam_range = 1000 - range; //range of beam(cm)
		Int_t beam_range_int = static_cast<int>(beam_range);
		for(int i = 0; i<= beam_range_int; i++){
			beam_ene_remain -= 10.8722/(beam_ene_remain + 7.61098) + 0.00601254;
		    }
		    h1d_beam_ene->Fill(posz[0]);
	   }
	    if(/*ievent%2 == 1 &&*/ track_num == 2){
	    	cout<<" Event = "<<ievent<<", track_num = "<<track_num<<", Range = "<<range<<", Range_brag = "<<range_at_max_slope<<endl;
            h2d_range_range_brag->Fill(range,range_at_max_slope);
	    	h2d_range_q->Fill(range,Q_total);
            h2d_range_brag_q->Fill(range_at_max_slope,Q_total);
            h2d_minus->Fill(range,range-range_at_max_slope);

            //cout<<"Range = "<<range<<", Range_Brag = "<<range_at_max_slope<<endl;
	    	h2d_pid->Fill(theta * rad, range);
	    	h2d_pid_brag->Fill(theta * rad, range_brag);
	    	if(cutg_13Ber->IsInside(theta * rad, range)){
			h2d_range->Fill(theta * rad, range);
			Double_t Dis_R = Projection(theta * rad,range, 0.117522, -34.8712, 2632.36, 180., 0.01);
			h1d_range->Fill(Dis_R);
		    }
	    }

	    	h2d_kine->Fill(theta * rad, Q_total);
	    	h2d_energy->Fill(theta * rad, energy);
	    	g_kine->SetPoint(ievent, theta * rad, Q_total);
	    	g_ene->SetPoint(ievent, theta * rad, energy);

	    if(cutg_13Bee->IsInside(theta * rad,energy)){
	        h2d_cut->Fill(theta * rad,energy);
		    Double_t Ex = ExEne(IMBe, TMD, ASD, ARBe, beam_ene_remain, energy, theta * rad); 
		    h1d_ex->Fill(Ex);
		
		    Double_t Dis_E = Projection(theta * rad,energy,0.0005698,-0.225241,22.1936,180,0.01);
		    h1d_proe->Fill(Dis_E);
	    }
	    if(cutg_13Beq->IsInside(theta * rad,Q_total)){
	    	h2d_cutq->Fill(theta * rad,Q_total);

		    Double_t Dis_Q = Projection(theta * rad,Q_total,0.513122,-211.838,21365.6,180,0.01);
		    h1d_proq->Fill(Dis_Q);
	        }
         }
      }
   }
   
   
   //Draw plot	
   TCanvas *c0 = new TCanvas("c0","c0",1800,800);
   TCanvas *c1 = new TCanvas("c1","c1",1800,800);
   TCanvas *c2 = new TCanvas("c2","c2",1800,800);
   TCanvas *c3 = new TCanvas("c3","c3",1800,800);
   TCanvas *c4 = new TCanvas("c4","c4",1800,800);
   TCanvas *c5 = new TCanvas("c5","c5",1800,1200);
   TCanvas *c6 = new TCanvas("c6","c6",1800,800);
   TCanvas *c7 = new TCanvas("c7","c7",1800,800);
   TCanvas *c8 = new TCanvas("c8","c8",1800,800);

   c0->Divide(2,1);
   c1->Divide(2,1);
   c2->Divide(2,1);
   c3->Divide(2,1);
   c4->Divide(2,1);
   c5->Divide(2,2);
   c6->Divide(2,1);
   c7->Divide(2,1);
   c8->Divide(2,1);

   c0->cd(1);
   g_line->GetXaxis()->SetTitle("Energy (MeV)");
   g_line->GetYaxis()->SetTitle("dE/dx (MeV/mm)");
   g_line->GetXaxis()->CenterTitle();
   g_line->GetYaxis()->CenterTitle();
   g_line->Draw("APL");
   g_line->Fit("fx_1");
   c0->cd(2);
   h1d_beam_ene->Draw();
   h1d_beam_ene->GetXaxis()->SetTitle("Reaction positionz(mm)");
   h1d_beam_ene->GetYaxis()->SetTitle("Counts");
   h1d_beam_ene->GetXaxis()->CenterTitle();
   h1d_beam_ene->GetYaxis()->CenterTitle();
   c0->SaveAs(Form("beam_ene_%s.png",filepng));
   c1->cd(1);
   h2d_kine->Draw("colz");
   c1->cd(2);
   g_kine->SetMarkerSize(0.5);
   g_kine->SetMarkerStyle(20);
   g_kine->SetMarkerColor(kGreen);
   g_kine->GetXaxis()->SetTitle("Angle (Lab)");
   g_kine->GetYaxis()->SetTitle("Charge");
   g_kine->GetXaxis()->CenterTitle();
   g_kine->GetYaxis()->CenterTitle();
   g_kine->Draw("AP");
   c1->Draw();
   c1->SaveAs(Form("charge_angle_%s.png",filepng));
   c2->cd(1);
   h2d_energy->Draw("colz");
   c2->cd(2);
   g_ene->SetMarkerSize(0.5);
   g_ene->SetMarkerStyle(20);
   g_ene->SetMarkerColor(kGreen);
   g_ene->GetXaxis()->SetTitle("Angle (Lab)");
   g_ene->GetYaxis()->SetTitle("Energy (MeV)");
   g_ene->GetXaxis()->CenterTitle();
   g_ene->GetYaxis()->CenterTitle();
   g_ene->Draw("AP");
   c2->SaveAs(Form("energy_angle_%s.png",filepng));
   c3->cd(1);		
   h2d_eloss->GetXaxis()->SetTitle("Range (mm)");
   h2d_eloss->GetYaxis()->SetTitle("Charge/Range");
   h2d_eloss->GetXaxis()->CenterTitle();
   h2d_eloss->GetYaxis()->CenterTitle();
   h2d_eloss->Draw("colz");
   h2d_eloss->GetXaxis()->SetTitle("Range (mm)");
   c3->cd(2);		
   h2d_Brag->Draw("colz");
   c3->Draw();
   c3->SaveAs(Form("excitation_%s.png",filepng));

   c4->cd(1);
   h2d_pid->GetXaxis()->SetTitle("Angle (Lab)");
   h2d_pid->GetYaxis()->SetTitle("Range (mm)");
   h2d_pid->GetXaxis()->CenterTitle();
   h2d_pid->GetYaxis()->CenterTitle();
   h2d_pid->Draw("colz");
   c4->cd(2);
   h2d_pid_brag->GetXaxis()->SetTitle("Angle (Lab)");
   h2d_pid_brag->GetYaxis()->SetTitle("Range (mm)");
   h2d_pid_brag->GetXaxis()->CenterTitle();
   h2d_pid_brag->GetYaxis()->CenterTitle();
   h2d_pid_brag->Draw("colz");

   c4->SaveAs(Form("range_angle_%s.png",filepng));

   c5->cd(1);
   h2d_cutq->Draw("colz");
   h2d_cutq->GetXaxis()->SetTitle("Angle (Lab)");
   h2d_cutq->GetYaxis()->SetTitle("Charge");
   h2d_cutq->GetXaxis()->CenterTitle();
   h2d_cutq->GetYaxis()->CenterTitle();
   cutg_13Beq->Draw("same");
   //h2d_cut->Fit("fx");
   h2d_cutq->Fit("fx2");
   c5->cd(2);
   h1d_proq->Draw();
   h1d_proq->GetXaxis()->SetTitle("Deviation (Charge)");
   h1d_proq->GetYaxis()->SetTitle("Counts");
   h1d_proq->GetXaxis()->CenterTitle();
   h1d_proq->GetYaxis()->CenterTitle();
   h1d_proq->Fit("gaus");
   c5->cd(3);
   h2d_cut->Draw("colz");
   h2d_cut->GetXaxis()->SetTitle("Angle (Lab)");
   h2d_cut->GetYaxis()->SetTitle("Energy (MeV)");
   h2d_cut->GetXaxis()->CenterTitle();
   h2d_cut->GetYaxis()->CenterTitle();
   cutg_13Bee->Draw("same");
   //h2d_cut->Fit("fx");
   h2d_cut->Fit("fx2");
   c5->cd(4);
   h1d_proe->GetXaxis()->SetTitle("Deviation (Energy)");
   h1d_proe->GetYaxis()->SetTitle("Counts");
   h1d_proe->GetXaxis()->CenterTitle();
   h1d_proe->GetYaxis()->CenterTitle();
   h1d_proe->Draw();
   h1d_proe->Fit("gaus");
   c5->SaveAs(Form("projection_%s.png",filepng));

   c6->cd(1);
   h2d_range_q->GetXaxis()->SetTitle("Range (mm)");
   h2d_range_q->GetYaxis()->SetTitle("Charge");
   h2d_range_q->GetXaxis()->CenterTitle();
   h2d_range_q->GetYaxis()->CenterTitle();
   h2d_range_q->Draw("colz");
   h2d_range_q->Fit("fx2");
   c6->cd(2);
   h2d_range_brag_q->GetXaxis()->SetTitle("Range (mm)");
   h2d_range_brag_q->GetYaxis()->SetTitle("Charge");
   h2d_range_brag_q->GetXaxis()->CenterTitle();
   h2d_range_brag_q->GetYaxis()->CenterTitle();
   h2d_range_brag_q->Draw("colz");
   h2d_range_brag_q->Fit("fx2");
   c6->SaveAs(Form("range_q_%s.png",filepng));
   c7->cd(1);
   h1d_eloss->Draw();
   c8->cd(1);
   h2d_range_range_brag->Draw("colz");
   h2d_range_range_brag->GetXaxis()->SetTitle("Range (mm)");
   h2d_range_range_brag->GetYaxis()->SetTitle("Range_Brag (mm)");
   h2d_range_range_brag->GetXaxis()->CenterTitle();
   h2d_range_range_brag->GetYaxis()->CenterTitle();
   fx->Draw("same");
   c8->cd(2);
   h2d_minus->Draw("colz");
   h2d_minus->GetXaxis()->SetTitle("Range (mm)");
   h2d_minus->GetYaxis()->SetTitle("Range-Range_brag (mm)");
   h2d_minus->GetXaxis()->CenterTitle();
   h2d_minus->GetYaxis()->CenterTitle();
   c8->SaveAs(Form("range_minus_%s.png",filepng));

   hitsFile.close();
   file->Close();
   //Write in root file
   TString outfile = "./ground.root";
   TFile *rootfile = new TFile(outfile,"recreate");
   h1d_ex->Write();
}


double ExEne(double IM, double TM, double AS, double AR, double K1, double K3, double theta){
	const double unit = 931.494102;
	
	double m1 = IM * unit;
	double m2 = TM * unit;
	double m3 = AS * unit;
	double m4 = AR * unit;
	
	double E1 = m1 + K1;
	double E3 = m3 + K3;

	double p1 = sqrt(pow(E1,2) - pow(m1,2));
	double p3 = sqrt(pow(E3,2) - pow(m3,2));

	double thetalab = (TMath::Pi() * theta)/180;
	
	double s = pow(m1,2) + pow(m2,2) + 2*m2*E1;
	double t = pow(m1,2) + pow(m3,2) + 2*(p1*p3*cos(thetalab) - E1*E3);
	double u = pow(m2,2) + pow(m3,2) - 2*m2*E3;

	double Ex = sqrt(s + t + u - pow(m1,2) - pow(m2,2) - pow(m3,2)) - m4;
	
	return Ex;
}

double Range(double posx_1st, double posy_1st, double posz_1st, double posx_last, double posy_last, double posz_last){

	double posx2 = pow((posx_1st-posx_last),2);
	double posy2 = pow((posy_1st-posy_last),2);
   	double posz2 = pow((posz_1st-posz_last),2);
	double range = sqrt(posx2 + posy2 + posz2);

	return range;
}

double Distance3D_pu(double Reac_p[3], double Proj[3]){
	double distance = pow( (Reac_p[0] - Proj[0]), 2 ) + pow( (Reac_p[1] - Proj[1]), 2 ) + pow( (Reac_p[2] - Proj[2]), 2 );
	return distance;
}

double Projection(double x_mea, double y_mea, double p0, double p1, double p2, double x_limit, double step){
	double x = 0;
	double y = 0;
	double x_0 = 0;
	double y_0 = 0;
	double dis_tmp = 0;
	double dis = std::numeric_limits<double>::max();;
	while(x < x_limit){
		y = p0 * pow(x,2) + p1 * x + p2;
		dis_tmp = sqrt(pow((y_mea-y),2) + pow((x_mea - x),2));
		if(dis >= dis_tmp){
			dis = dis_tmp;
			x_0 = x;
			y_0 = y;
		}
		x += step;
	}
	double distance = (y_0 >= y_mea) ? dis : -dis;

	return distance;
}
