// Check the coincidence between ENDAQ and GET
// 2025.11.13

void checkClock()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   // Set the root file path
   Int_t runNum = 0;
   cout<<"=> Please input AT-TPC run number: "<<endl;
   cin>>runNum;

   Int_t runNumENCourse = 0;
   cout<<"=> Please input EN course run number: "<<endl;
   cin>>runNumENCourse;

   TString dataDirEN  = "/data/enData/E510/";
   TString dataDirGET = "/data/ATTPCROOTv2_results/E510/UnpackerOutput/";
   TString runNumEN  = TString::Format("%04d.root", runNumENCourse);
   TString runNumGET = TString::Format("run_%04d_reUnpack_Theshold30.root", runNum);

   TString runFileEN  = dataDirEN  + runNumEN;
   TString runFileGET = dataDirGET + runNumGET;

   // Open the root files and get the TTree.
   TFile *fileEN  = new TFile(runFileEN,  "READ");
   TFile *fileGET = new TFile(runFileGET, "READ");
   TTree *trEN  = (TTree*) fileEN->Get("tree");
   TTree *trGET = (TTree*) fileGET->Get("cbmsim");

   TTreeReader readerEN(trEN);
   TTreeReaderValue<ULong64_t> clockEN(readerEN, "madc.counter");

   TTreeReader readerGET(trGET);
   TTreeReaderValue<TClonesArray> eventArray(readerGET, "AtEventH");

   // Plot definition
   TCanvas *c1 = new TCanvas("c1", "", 1600, 1200);
   c1->Divide(2,3);

   TGraph *gr_GET = new TGraph();
   TGraph *gr_EN = new TGraph();
   gr_GET->SetTitle(TString::Format("GET time stamp: run %04d", runNum));
   gr_EN->SetTitle(TString::Format("EN time stamp: run %04d", runNumENCourse));
   gr_GET->GetXaxis()->SetTitle("Number of Events"); gr_GET->GetYaxis()->SetTitle("Clock counts");
   gr_EN->GetXaxis()->SetTitle("Number of Events"); gr_EN->GetYaxis()->SetTitle("Clock counts");
   gr_GET->SetMarkerColor(kRed);

   const int xmax = 8e4;
   TH1F *h1_GET  = new TH1F("h1_GET",  "GET clock", xmax/10, 0, xmax);
   TH1F *h1_EN   = new TH1F("h1_EN",   "EN clock",  xmax/10, 0, xmax);
   TH1F *h1_diff = new TH1F("h1_diff", "Clock diff (EN-GET)", 100, 0, 50);
   h1_GET->GetXaxis()->SetTitle("Number of Events"); h1_GET->GetYaxis()->SetTitle("Record speed");
   h1_EN->GetXaxis()->SetTitle("Number of Events"); h1_EN->GetYaxis()->SetTitle("Record speed");
   h1_diff->GetXaxis()->SetTitle("#DeltaT (us)"); h1_diff->GetYaxis()->SetTitle("Counts");
   h1_GET->SetLineColor(kRed);

   TGraph *gr_ratio = new TGraph();
   gr_ratio->GetXaxis()->SetTitle("Number of Events"); gr_ratio->GetYaxis()->SetTitle("GET / EN");
   gr_ratio->SetTitle("Time stamp ratio");
   gr_ratio->SetMarkerColor(kBlue);


   int nEventsEN  = trEN->GetEntries();
   int nEventsGET = trGET->GetEntries();
   std::cout << " runNameEN: " << runNumEN << ", nEvents: " << nEventsEN << std::endl;
   std::cout << " runNameGET: " << runNumGET << ", nEvents: " << nEventsGET << std::endl;

   const int freEN = 1e6;	//> fre.: 1 MHz
   const int freGET = 1e8;	//> fre.: 100 MHz
   ULong64_t tsEN = 0, tmpEN = 0;
   ULong64_t tsGET = 0, tmpGET = 0;
   ULong64_t deltaEN = 0, deltaGET = 0;

   // Events loop
   for(int ievt=0; ievt<nEventsGET/1; ievt++) {
	   if(ievt%100 == 0)
	   std::cout<<" Process: "<<ievt<<" evts, "<<ievt*100./nEventsGET<<"%"<<std::endl;

	   //- GET loop
	   readerGET.Next();

	   //AtRawEvent *rawEvent = (AtRawEvent*) rawArray->At(0);
	   AtEvent *event = (AtEvent*) eventArray->At(0);
	   if (!event) continue;

	   tmpGET = tsGET;
	   tsGET = event->GetTimestamp();
	   if(tmpGET>0 && tsGET>0) deltaGET = tsGET - tmpGET;

	   //- EN loop
	   readerEN.Next();

	   tmpEN = tsEN;
	   tsEN = *clockEN;
	   if(tmpEN>0 && tsEN>0) deltaEN = tsEN - tmpEN;


	   //- check
	   gr_GET->SetPoint(ievt, ievt, tsGET);
	   gr_EN->SetPoint(ievt, ievt, tsEN);
	   h1_GET->Fill(ievt, deltaGET);
	   h1_EN->Fill(ievt, deltaEN);
	   h1_diff->Fill(tsEN/freEN - tsGET/freGET);

	   if(deltaEN>0)
	   gr_ratio->SetPoint(ievt, ievt, deltaGET/deltaEN);
   }

   c1->cd(1); gr_GET->Draw("ap"); 
   	gPad->SetGrid(1,1);
   c1->cd(2); h1_GET->Draw("hist"); 
   c1->cd(3); gr_EN->Draw("ap");
   	gPad->SetGrid(1,1);
   c1->cd(4); h1_EN->Draw("hist");
   c1->cd(5); gr_ratio->Draw("ap");
   c1->cd(6); h1_diff->Draw();

   c1->SaveAs(Form("output/run%d.png", runNum));
}
