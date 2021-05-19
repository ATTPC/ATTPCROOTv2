// Macro to link events between ATTPC and NSCL
// Takes the min |dTS| as the offset
// NSCL: 310 TPC: 118
// NSCL: 407 TPC: 200


void linkRuns(int tpcRunNum = 118, int nsclRunNum = 310)
{
  TChain trNscl("E12014");
  trNscl.Add(TString::Format("/mnt/analysis/e12014/HiRAEVT/unpacked/run-%04d-00.root", nsclRunNum));
  TChain trTpc("cbmsim");
  trTpc.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpcRunNum));

  //
  TTreeReader nsclTr(&trNscl);
  TTreeReader tpcTr(&trTpc);

  TTreeReaderArray<unsigned long long> nsclTS(nsclTr, "tstamp");
  TTreeReaderArray<Short_t> adc1(nsclTr, "adc1");

  TTreeReaderValue<TClonesArray> tpcArray(tpcTr, "ATRawEvent");

  //This is basically the start of FindOffsetFunction
  //Load in the first timestamps. Skip the first event.
  nsclTr.Next();  nsclTr.Next();
  tpcTr.Next();  tpcTr.Next();
  auto event = (ATRawEvent*) tpcArray->At(0);

  //Set what timestamp we're using to match
  int tsIndex = 0;
  
  //auto tpcOffset = event->GetTimestamp()/10;
  auto tpcOffset = event->GetTimestamp(tsIndex);

  auto nsclTSOld = nsclTS[1];
  unsigned long long tpcTSOld = 0;
  unsigned long long tpcTSOldSis = 0;

  //TH1F *histInterval = new TH1F("histInt", "Interval difference", 1000, -100, 400);
  double mean = 8.99925;
  double radius = 0.00275;

  TH1F *histIntervalSis = new TH1F("histIntSis", "Interval difference Sis", 200, -100, 100);
  TH1F *histInterval = new TH1F("histInt", "Interval difference", 1000, mean - radius, mean + radius);
  TH1D *histOffset = new TH1D("histOff", "Offset", 1000, -100000, 1000);

  //
  TGraph *grOffset, *grTPC, *grNSCL, *grInterval, *grDiff;

  //IC comparison plots
  TH2F *hIC = new TH2F("hIC", "NSCL IC vs TPC IC", 100, 2000, 3000, 100, 700, 1100);
  TH1F *hICRatio = new TH1F("hIcRatio", "NSCL/TPC", 100, 2, 8);
  TGraph *grICRatio;
  
  std::cout << "NSCL entries: " << nsclTr.GetEntries(false) << std::endl
	    << "TPC entries:  " << tpcTr.GetEntries(false) << std::endl << std::endl;

  //int numEntries = tpcTr.GetEntries(false);
  int numEntries = 1e4;
  //numEntries = numEntries < nsclTr.GetEntries(false) ? numEntries : nsclTr.GetEntries(false);
  //numEntries = numEntries < tpcTr.GetEntries(false) ? numEntries : tpcTr.GetEntries(false);

  grOffset = new TGraph(numEntries);   //TPC TS - NSCL TS
  grTPC = new TGraph(numEntries);      //TPC TS
  grNSCL = new TGraph(numEntries);     //NSCL TS
  grInterval = new TGraph(numEntries); //interval/nsclInterval
  grDiff = new TGraph(numEntries);
  grICRatio = new TGraph(numEntries);  //adc/waveform max

  while(tpcTr.GetCurrentEntry() < numEntries - 1)
  {
    //Load the next entry
    if( !(nsclTr.Next() && tpcTr.Next()) )
      break;
    event = (ATRawEvent*) tpcArray->At(0);
    if (event == nullptr)
      break;
    auto tpcTS = event->GetTimestamp(tsIndex) - tpcOffset;

/*    //Print out the timestamps
    for(auto &&ts : *event->GetTimestamps())
      std::cout << tpcTr.GetCurrentEntry() << " " << ts << std::endl;
    std::cout << std::endl;
*/  
    //Get the TS intervals
    auto tpcInterval  = tpcTS - tpcTSOld;
    auto nsclInterval = nsclTS[1] - nsclTSOld;
    
    //Get the abs difference in interval |NSCL - TPC|
    int dInterval = 0;
    if (nsclInterval > tpcInterval)
      dInterval = nsclInterval - tpcInterval;
    else
      dInterval = tpcInterval - nsclInterval;


    //Scale the difference in the intervals
    float fInterval = (float)dInterval/nsclInterval;

    //Get the interval from Sis timestamp
    float tpcSisInterval = (float)event->GetTimestamp(1) - tpcTSOldSis;
    float fSisInterval = tpcSisInterval - (float)nsclInterval;
    histIntervalSis->Fill(fSisInterval);    
    
    //Check to see if this is within radius of mean
    if (fInterval > mean + radius || fInterval < mean - radius)
    {
      auto tpcEntry = tpcTr.GetCurrentEntry();
      auto nsclEntry = nsclTr.GetCurrentEntry();
      std::cout << "Missed an event, fInterval is " << fInterval << std::endl
		<< "TPC event number:  " << tpcEntry << std::endl
		<< "NSCL event number: " << nsclEntry << std::endl;

      //Look one tpc entry ahead
      tpcTr.Next();
      event = (ATRawEvent*) tpcArray->At(0);
      if (event == nullptr)
	break;
      auto nextTpcTS = event->GetTimestamp(tsIndex) - tpcOffset;
      auto nextTpcInterval = nextTpcTS - tpcTS;
      float fNextTpcInterval = (nsclInterval > nextTpcInterval) ?
	nsclInterval - nextTpcInterval : nextTpcInterval - nsclInterval;
      fNextTpcInterval /= nsclInterval;

      //std::cout << "skipping TPC gives interval " << fNextTpcInterval << std::endl;
      
      //Look one nscl entry ahead
      nsclTr.Next();
      auto nextNsclInterval = nsclTS[1] - nsclTSOld;
      float fNextNsclInterval = (nextNsclInterval > tpcInterval) ?
	nextNsclInterval - tpcInterval : tpcInterval - nextNsclInterval;
      fNextNsclInterval /= nextNsclInterval;

  
      //Check to see if skipping TPC entry fixed problem
      if (fNextTpcInterval < mean + radius && fNextTpcInterval > mean - radius)
      {
	tpcEntry++;
	fInterval = fNextTpcInterval;
	tpcTS = nextTpcTS;

        //We fixed the problem by skipping a TPC entry
	std::cout << "Fixed event by skipping TPC event, fInterval is " << fInterval << std::endl
		  << "TPC event number:  " << tpcEntry << std::endl
		  << "NSCL event number: " << nsclEntry << std::endl << std::endl;
      }
      else if (fNextNsclInterval < mean + radius && fNextNsclInterval > mean - radius)
      {
	nsclEntry++;
	fInterval = fNextNsclInterval;
		
        //We fixed the problem by skipping a NSCL entry
	std::cout << "Fixed event by skipping NSCL event, fInterval is " << fInterval << std::endl
		  << "TPC event number:  " << tpcEntry << std::endl
		  << "NSCL event number: " << nsclEntry << std::endl << std::endl;
      }
      else
      {
	std::cout << "Failed to fix problem: quiting" << std::endl;
	return;
      }
      
      //Reset tree readers to correct values
      tpcTr.SetEntry(tpcEntry);
      nsclTr.SetEntry(nsclEntry);
      
    }// End correction. Entries should now be aligned
    event = (ATRawEvent*) tpcArray->At(0);
    
    //Fill histogram with interval
    histInterval->Fill(fInterval);
    grInterval->SetPoint(tpcTr.GetCurrentEntry(),tpcTr.GetCurrentEntry(), fInterval);    

    //Get the double equivalent of the tpc and nscl timestamps and fill the histogram
    double fTPC = tpcTS;
    double fNSCL = nsclTS[1];
    histOffset->Fill(fTPC - fNSCL);

    //Fill the graphs
    grOffset->SetPoint(tpcTr.GetCurrentEntry(),tpcTr.GetCurrentEntry(), event->GetTimestamp(1) - fNSCL);
    grNSCL->SetPoint(tpcTr.GetCurrentEntry(),tpcTr.GetCurrentEntry(), fNSCL);
    grTPC->SetPoint(tpcTr.GetCurrentEntry(),tpcTr.GetCurrentEntry(), fTPC);
    
    //Save old values
    nsclTSOld = nsclTS[1];
    tpcTSOld  = tpcTS;
    tpcTSOldSis = event->GetTimestamp(1);

    // Now get the IC values and fill hIC(nscl, tpc)
    // event is type ATRawEvent
    // Only way to find aux pads is to loop through all pads, so loop and look for IC
    for( auto&& pad : *(event->GetPads()))
      if(pad.IsAux() && pad.GetAuxName().compare("IC") == 0)
      {
	
	//Get baseline
	double baseline = 0;
	for(int i = 0; i < 50; ++i)
	  baseline += pad.GetRawADC(i);
	baseline /= 50.;
	
	
	//Search for max between TB 60 and 65 (inclusive)
	double max = 0;
	int maxLoc = 0;
	for(int i = 70; i <= 75; ++i)
	  if( max < pad.GetRawADC(i) )
	  {
	    max = pad.GetRawADC(i);
	    maxLoc = i;
	  }
	
	//std::cout << max << " " << pad.GetRawADC(maxLoc-2);
	//max -= pad.GetRawADC(maxLoc-2);
	max -= baseline;
	//std::cout << " " << max << std::endl << std::endl;

	//Fill the histogram
	hIC->Fill(adc1[2], max);
	hICRatio->Fill(adc1[2]/max);
	grICRatio->SetPoint(tpcTr.GetCurrentEntry(),tpcTr.GetCurrentEntry(), adc1[2]/max);
      }
      
    
  }//End loop over tree entries

  //Try to fit with a Poisson distobution
  // "xmin" = 0, "xmax" = 10
  TF1 *f1 =
    //new TF1("f1","[0]*TMath::Power(([1]/[2]),(x/[2]))*(TMath::Exp(-([1]/[2])))/TMath::Gamma((x/[2])+1.)", 0, 10);
    //new TF1("f1","[0] * TMath::Power([1], x) * TMath::Exp(-[1]) / TMath::Gamma(x + 1.)", 0, 50);
    new TF1("f1","[0] * x * TMath::Exp(-[1] * x)", 0, 30); 
  f1->SetParameters(1, 1); // you MUST set non-zero initial values for parameters
  //histInterval->Fit("f1", "R"); // "R" = fit between "xmin" and "xmax" of the "f1"

  // Draw the offset histograms and the interval histograms
  //histInterval->Draw();
  //histOffset->Draw();
  TCanvas *c1 = new TCanvas("c1", "Offset");
  grOffset->Draw("AC*");
  TCanvas *c2 = new TCanvas("c2", "TPC");
  grTPC->Draw("AC*");
  TCanvas *c3 = new TCanvas("c3", "NSCL");
  grNSCL->Draw("AC*");
  TCanvas *c4 = new TCanvas("c4", "Interval");
  grInterval->Draw("AC*");
  TCanvas *c5 = new TCanvas("c5", "Interval");
  histInterval->Draw();
  TCanvas *c6 = new TCanvas("c6", "IC");
  hIC->Draw("colz");
  TCanvas *c7 = new TCanvas("c7", "IC Ratio");
  hICRatio->Draw("colz");
  TCanvas *c8 = new TCanvas("c8", "IC Ratio");
  grICRatio->Draw("AC*");
  TCanvas *c9 = new TCanvas("c9", "Sis Interval");
  histIntervalSis->Draw();
  

  
  // std::cout << "Scaling: " << f1->GetParameter(0) << std::endl
//	    << "Mean: " << f1->GetParameter(1) << std::endl;
//	    << "Mean scaling: " << f1->GetParameter(2) << std::endl;
						       
}

