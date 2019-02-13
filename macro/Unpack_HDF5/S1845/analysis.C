void analysis()
{

	FairRunAna* run = new FairRunAna(); //Forcing a dummy run

	TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "output_proto";
    TString FilePath = workdir + "/macro/Unpack_HDF5/S1845/";
    TString FileNameTail = ".root";
    TString FileName     = FilePath + FileNameHead + FileNameTail;

    std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> raweventArray(Reader1, "ATRawEvent");
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    TTreeReaderValue<TClonesArray> patterneventArray(Reader1, "ATPatternEvent");

    TString scriptfile = "LookupProto20181201v2.xml";
    TString protomapfile = "proto20181201.map";
    TString scriptdir = workdir + "/scripts/"+ scriptfile;
    TString protomapdir = workdir + "/scripts/"+ protomapfile;
    TString geo = "proto20181201_geo_hires.root";

    AtTpcMap *fAtMapPtr = new AtTpcProtoMap();
    fAtMapPtr->ParseXMLMap(scriptdir.Data());
    fAtMapPtr->SetGeoFile(geo);
    fAtMapPtr->SetProtoMap(protomapdir.Data());

    TH2Poly* fPadPlane = fAtMapPtr->GetATTPCPlane("ATTPC_Proto");

    // Histograms

    TH2F* range_vs_Q = new TH2F("range_vs_Q","range_vs_Q",100,0,200,1000,0,1000000);


     for(Int_t i=0;i<nEvents;i++){
          //while (Reader1.Next()) {

              Reader1.Next();

              ATEvent* event = (ATEvent*) eventArray->At(0);

              if(event!=NULL)
              {
            
	              Int_t nHits = event->GetNumHits();
	              std::vector<ATHit>* hitArray = event->GetHitArray();
	              event->GetHitArrayObj();
	              std::cout<<" Event Q : "<<event->GetEventCharge()<<std::endl;
	              //std::cout<<hitArray->size()<<"\n";
	              ATPatternEvent* patternEvent = (ATPatternEvent*) patterneventArray->At(0);
	              std::vector<ATTrack>& tracks = patternEvent->GetTrackCand();
	              std::cout<<" Found tracks "<<tracks.size()<<"\n";

	              bool isValid = true;

	              for(auto  track : tracks)
	              {
	              	//std::cout<<" - Number of hits : "<<track.GetHitArray()->size()<<"	- Is noise? : "<<track.GetIsNoise()<<"\n";

	              	if( track.GetHitArray()->size()>10 && track.GetIsNoise()==0 && tracks.size()==2)
	              	{
	              		track.SortHitArrayTime();
	              		//std::cout<<" - Number of hits : "<<track.GetHitArray()->size()<<"	- Is noise? : "<<track.GetIsNoise()<<"\n";
	              		std::cout<<" **** Track ID : "<<track.GetTrackID()<<"\n";
	              		std::vector<ATHit>*  hits = track.GetHitArray();

	              		for(auto hit : *hits)
	              		{

	              			TVector3 pos = hit.GetPosition();
	              			//std::cout<<pos.X()<<"	"<<pos.Y()<<"	"<<pos.Z()<<"	"<<hit.GetCharge()<<"\n";
	              			if( TMath::Sqrt(pos.X()*pos.X() + pos.Y()*pos.Y())>110 )
	              			{
	              				isValid = false;
	              				break;
	              			}else{
	              				fPadPlane->Fill(pos.X(), pos.Y(), hit.GetCharge());
	              			}


	              		}

	              		std::cout<<" Range : "<<track.GetLinearRange()<<"\n";

	              	    if(isValid)range_vs_Q->Fill(track.GetLinearRange(),event->GetEventCharge()); //For the moment only events where the clustering works 1 track + noise

	              	}



	              }



	          }   

     }//for loop

     TCanvas *c1 = new TCanvas();
     c1->Divide(1,2);
     c1->cd(1);
     range_vs_Q->Draw("zcol"); 
     c1->cd(2);
     fPadPlane -> Draw("COL L");
     fPadPlane -> SetMinimum(1.0);
     gStyle->SetOptStat(0);
     gStyle->SetPalette(103);
     gPad ->Update();        

}