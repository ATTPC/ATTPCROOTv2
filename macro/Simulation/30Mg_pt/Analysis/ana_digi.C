void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E);

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

    TH1F* energy = new TH1F("enegy","energy",100,0,40);

    TH2F* ang_vs_energy = new TH2F("ang_vs_energy","ang_vs_energy,",100,0,200,100,0,40);

    double _B = 2.0; //Magnetic field


    for(Int_t i=0;i<22;i++){
         

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

	              	double bro = _B*parameter[5]/TMath::Sin(parameter[6])/1000.0;
	              	std::cout<<" Bro "<<bro<<"\n";
	              	double ener = 0;
  					GetEnergy(1.0,1.0,bro,ener);
  					std::cout<<" Energy "<<ener<<"\n";

	              	scatteringAngle->Fill(parameter[6]*180.0/TMath::Pi());
	              	energy->Fill(ener);

	              	ang_vs_energy->Fill(parameter[6]*180.0/TMath::Pi(),ener);

	            }

	              


              }

             



     }// Event loop

     TCanvas *c1 = new TCanvas("c1","c1",400,400);
	 scatteringAngle->Draw();  

	 TCanvas *c2 = new TCanvas("c2","c2",400,400);
	 energy->Draw();

	 TCanvas *c3 = new TCanvas("c3","c3",400,400);   
	 ang_vs_energy->Draw("col");      



}

void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E){

  //Energy per nucleon
  Float_t  AM=931.5;
  Float_t X=BRO/0.1439*IZ/M;
  X=pow(X,2);
  X=2.*AM*X;
  X=X+pow(AM,2);
  E=TMath::Sqrt(X)-AM;

}