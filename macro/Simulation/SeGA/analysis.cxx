// Generic analysis for HELIOS MC simulation
// Y. Ayyad ayyadlim@frib.msu.edu 12/30/2020

void sim_analysis(Int_t num_ev=10000)
{

    TString mcFileNameHead = "./data/SeGA";
    TString mcFileNameTail = ".root";
    TString mcFileName     = mcFileNameHead + mcFileNameTail;
    TString outFileNameHead = "SeGAana";
    TString outFileNameTail = ".root";
    TString outFileName     = outFileNameHead + outFileNameTail;

    AtSeGAPoint* point = new AtSeGAPoint();
    TClonesArray *pointArray=0;

    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");


    tree = (TTree*) file -> Get("cbmsim");
    tree -> SetBranchAddress("AtSeGAPoint", &pointArray);
    Int_t nEvents = tree -> GetEntriesFast();

    if(nEvents>num_ev) nEvents=num_ev;


    

    TCanvas *c1 = new TCanvas();
    c1->Divide(1,1);
    c1->Draw();
    double energy{20};
    Int_t points{20};

    for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {

	tree->GetEvent(iEvent);
        // tree -> GetEntry(iEvent);
        Int_t n = pointArray -> GetEntries();
        std::cout<<" ---- Event Number : "<<iEvent<<std::endl;
        double zpos_sca = 0.0;
        double energyLoss_sca = 0.0;
        double energyLossTarget_sca = 0.0;
        double zpos_rec = 0.0;
        double xpos_rec = 0.0;
        double ypos_rec = 0.0;
        double energyLoss_rec = 0.0;
	double angle_sca = 0.0;
	double energy_sca = 0.0;

           for(Int_t =0; i<n; i++) {

            point = (AtSeGAPoint*) pointArray -> At(i);
            auto VolName="germaniumTarget";
            auto trackID = point -> GetTrackID();
            //std::cout<<" Volume Name : "<<VolName<<std::endl;
            //std::cout<<" Track ID : "<<trackID<<std::endl;

             if(trackID==1) //Scattered proton
	     {
              if(VolName!="germaniumTarget"){
	       std::cout<<" Volume Name : "<<VolName<<std::endl;
               std::cout<<" Track ID : "<<trackID<<std::endl;
               std::cout<<" Point number : "<<i<<std::endl;
	       energyLoss_sca+=( point -> GetEnergyLoss() )*1000;//MeV
	       zpos_sca=point->GetZ()*10;
               //angle_sca=point->GetAIni();
	       std::cout<<" Energy Loss "<<energyLoss_sca<<"  Z pos "<<zpos_sca<<" Point ID : "<<i<<"\n";
               }else if(VolName=="germaniumTarget"){
                 
                 energyLossTarget_sca+=( point -> GetEnergyLoss() )*1000;//MeV;
               }

	     }

             if(trackID==0) //Heavy recoil
	     {
	      if(VolName!="germaniumTarget"){
	       std::cout<<" Volume Name : "<<VolName<<std::endl;
               std::cout<<" Track ID : "<<trackID<<std::endl;
               std::cout<<" Point number : "<<i<<std::endl;
	       energyLoss_rec+=( point -> GetEnergyLoss() )*1000;//MeV
	       zpos_rec=point->GetZ()*10;
               xpos_rec=point->GetX()*10;
               ypos_rec=point->GetY()*10;
	       //std::cout<<" Energy Loss "<<energyLoss_rec<<"  Z pos "<<zpos_rec<<" Point ID : "<<i<<"\n";
              }
	     }
				int count = i;

           }

	//Eloss_vs_Z->Fill(zpos_sca,energyLoss_sca);
        //Recoil_XY->Fill(xpos_rec,ypos_rec);
       // ThetaLabSim_vs_Z->Fill(zpos_sca,angle_sca);

       points[iEvent] = n;
       energy[iEvent] = energyLoss_rec;

    }
    
   // c1->cd(1);
  //  Eloss_vs_Z->Draw("zcol");
   // c1->cd(2);
  //  Recoil_XY->Draw("zcol");
  auto g = new TGraph(20,energy,points);
   
    c1->cd(1);
    g->Draw("AC*");

}
