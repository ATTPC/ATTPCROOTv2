void ana_digi()
{

	FairRunAna* run = new FairRunAna(); //Forcing a dummy run

	std::string digiFileName = "../output_digi_4.root";

	TFile* file = new TFile(digiFileName.c_str(),"READ");

	TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    TTreeReaderValue<TClonesArray> houghArray(Reader1, "ATHough");

    TH1F* scatteringAngle = new TH1F("scatteringAngle","scatteringAngle",1000,0,200);


    for(Int_t i=0;i<nEvents;i++){
         

              Reader1.Next();
              ATHoughSpaceCircle* fHoughSpaceCircle  = dynamic_cast<ATHoughSpaceCircle*> (houghArray->At(0));


              if(fHoughSpaceCircle==NULL) std::cout<<" Null pointer "<<"\n";

              else{

              	if(i%2!=0){
              	  std::cout<<" >>>> Event "<<i<<"\n";

              	  const std::vector<double> parameter = fHoughSpaceCircle->GetInitialParametersVector();	              

	              	std::cout<<" 	X ini "<<parameter[0]<<"\n";
	              	std::cout<<" 	Y ini "<<parameter[1]<<"\n";
	              	std::cout<<" 	Z ini "<<parameter[2]<<"\n";
	              	std::cout<<" 	Ini TB "<<parameter[3]<<"\n";
	              	std::cout<<" 	Phi ini "<<parameter[4]<<"\n";
	              	std::cout<<" 	Radius ini "<<parameter[5]<<"\n";
	              	std::cout<<" 	Theta ini "<<parameter[6]*180.0/TMath::Pi()<<"\n";
	              	std::cout<<" 	Hit ID ini "<<parameter[7]<<"\n";

	              	scatteringAngle->Fill(parameter[6]*180.0/TMath::Pi());

	            }

	              


              }

             



     }// Event loop

     TCanvas *c1 = new TCanvas("c1","c1",400,400);
	 scatteringAngle->Draw();             



}