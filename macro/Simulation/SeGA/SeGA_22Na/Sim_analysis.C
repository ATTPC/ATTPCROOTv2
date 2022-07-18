// Generic analysis for HELIOS MC simulation
// Y. Ayyad ayyadlim@frib.msu.edu 12/30/2020

void Sim_analysis(Int_t num_ev=100000)
{

    TString mcFileNameHead = "./SeGA";
    TString mcFileNameTail = ".root";
    TString mcFileName     = mcFileNameHead + mcFileNameTail;
    TString outFileNameHead = "./SeGAana";
    TString outFileNameTail = ".root";
    TString outFileName     = outFileNameHead + outFileNameTail;

    AtMCPoint* point = new AtMCPoint();
    TClonesArray *pointArray=0;

    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");


    tree = (TTree*) file -> Get("cbmsim");
    tree -> SetBranchAddress("AtMCPoint", &pointArray);
    Int_t nEvents = tree -> GetEntriesFast();

    if(nEvents>num_ev) nEvents=num_ev;

    //Histograms
    TH1D *Energy_loss= new TH1D("Energy_loss","Energy_loss",1000,0,10);
    

      TCanvas *c1 = new TCanvas();
    
    c1->Draw();
Double_t Count =0.0;

    for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {

	tree->GetEvent(iEvent);
        // tree -> GetEntry(iEvent);
        Int_t n = pointArray -> GetEntries();
       // std::cout<<" ---- Event Number : "<<iEvent<<std::endl;
        
        double energyLoss = 0.0;
        
           for(Int_t i=0; i<n; i++) {

            point = (AtMCPoint*) pointArray -> At(i);
            auto VolName=point->GetVolName();

            auto trackID = point -> GetTrackID();
           // std::cout<<" Volume Name : "<<VolName<<std::endl;
            //std::cout<<" Track ID : "<<trackID<<std::endl;

            
	       /*std::cout<<" Volume Name : "<<VolName<<std::endl;
               std::cout<<" Track ID : "<<trackID<<std::endl;
               std::cout<<" Point number : "<<i<<std::endl;*/
	       energyLoss+=( point -> GetEnergyLoss() )*1000;//MeV
		Count+=1;

	

	       
	     }
				

         if (energyLoss != 0.0){  

	Energy_loss->Fill(energyLoss);};
	

    }
// Photo Peak efficeny
Double_t momentum = 0.005;
Double_t photopeakcount =0.0;
for(Int_t i=0;i<1000;i++){
	Double_t cur = Energy_loss->GetBinContent(i);
	if(cur > photopeakcount){		
		photopeakcount = cur;
}
};
Double_t eff = (photopeakcount*100/Count);
std::cout<<"Photo peak efficency: "<<eff<<std::endl;

   // std::sort(x,x+20);
    c1->cd(1);
	
    Energy_loss->Draw();


}
