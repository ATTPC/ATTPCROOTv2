void patternTest()
{   

   FairRunAna* run = new FairRunAna(); //Forcing a dummy run
   TString FileName = "/mnt/simulations/attpcroot/fair_install_2020/yassid/ATTPCROOTv2/macro/Simulation/ATTPC/12Be_pp/output_digi_PRA.root";
   std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
   TFile* file = new TFile(FileName.Data(),"READ");
  
   TTree* tree = (TTree*) file -> Get("cbmsim");
   Int_t nEvents = tree -> GetEntries();
   std::cout<<" Number of events : "<<nEvents<<std::endl;

   TTreeReader Reader1("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATPatternEvent");
   //TTreeReaderValue<std::vector<genfit::Track>> fitterVector(Reader1, "ATTPC");

   
   //Histograms
   TH1F* angle = new TH1F("angle","angle",720,0,179);
   TH1F* momentum = new TH1F("momentum","momentum",1000,0,2.0);//GeV
   TH2F* angle_vs_momentum = new TH2F("angle_vs_momentum","angle_vs_momentum",720,0,179,1000,0,2.0);
   TH2F* pos_vs_momentum = new TH2F("pos_vs_momentum","pos_vs_momentum",200,0,200,1000,0,2.0);
   TH2F* length_vs_momentum = new TH2F("length_vs_momentum","length_vs_momentum",200,0,200,1000,0,2.0);
   TH2F* hits_vs_momentum = new TH2F("hits_vs_momentum","hits_vs_momentum",200,0,200,1000,0,2.0);

   //event display
   //genfit::EventDisplay* display = genfit::EventDisplay::getInstance();

   for(Int_t i=0;i<2;i++){
         
              std::cout<<" Event Number : "<<i<<"\n";

              Reader1.Next();
              ATPatternEvent* patternEvent = (ATPatternEvent*) eventArray->At(0);
              
           
              

              
	      std::vector<ATTrack>& patternTrackCand =  patternEvent->GetTrackCand();
              std::cout<<" Number of pattern tracks "<<patternTrackCand.size()<<"\n";
		for(auto track : patternTrackCand)
		{
                            std::cout<<" -  Track - "<<"\n";
		            std::vector<ATHit>* hitArray =  track.GetHitArray();
                            for(auto hit : *hitArray)
			    {
                              TVector3 pos = hit.GetPosition();
                              int TB = hit.GetTimeStamp();
                              std::cout<<" Pos : "<<pos.X()<<"	"<<pos.Y()<<"	"<<pos.Z()<<" "<<TB<<"\n";
			    }
		}
              
    }

    TCanvas *c1 = new TCanvas();
    c1->Divide(2,2);
    c1->Draw();
    c1->cd(1);
    angle->Draw();
    c1->cd(2);
    momentum->Draw();
    c1->cd(3);
    angle_vs_momentum->Draw(); 
    c1->cd(4);
    pos_vs_momentum->Draw(); 

    TCanvas *c2 = new TCanvas();
    c2->Divide(2,2);
    c2->Draw();
    c2->cd(1);   
    length_vs_momentum->Draw();
    c2->cd(2);
    hits_vs_momentum->Draw();
  

   

}
