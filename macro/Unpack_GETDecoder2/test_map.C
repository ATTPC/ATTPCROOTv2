void test_map()
{




  TStopwatch timer;
  timer.Start();

  AtTpcMap* fAtMapPtr = new AtTpcMap();
  fAtMapPtr->GenerateATTPC();
  TH2Poly *fPadPlane = fAtMapPtr->GetATTPCPlane();
  fAtMapPtr->Dump();

  Float_t x=0;
  Float_t y=0;
  Float_t z=0;

  gSystem->Load("libATTPCReco.so");
 
  FairRunAna* run = new FairRunAna(); //Forcing a dummy run
 

    TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "output";
    TString FilePath = workdir + "/macro/Unpack_HDF5/";
    TString FileNameTail = ".root";
    TString FileName     = FilePath + FileNameHead + FileNameTail;

   /* std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    

    for(Int_t i=0;i<nEvents;i++){
          //while (Reader1.Next()) {

              std::cout<<" Event "<<i<<"\n";

              Reader1.Next();

              ATEvent* event = (ATEvent*) eventArray->At(0);
              Int_t nHits = event->GetNumHits();
              std::vector<ATHit>* hitArray = event->GetHitArray();
              event->GetHitArrayObj();
              hitArray->size();

            for(Int_t iHit=0; iHit<nHits; iHit++){
              ATHit hit = event->GetHit(iHit);
              TVector3 hitPos = hit.GetPosition();
              Int_t bin=  fPadPlane->Fill(hitPos.X(),hitPos.Y(),hit.GetCharge());
             } 

    }*/

            for(Int_t i=0;i<100000;i++)
            {

                  x = x + 0.001;
                  y = y + 0.001;
                  z = z + 0.001;
                  Int_t bin=  fPadPlane->Fill(x,y,z);


            }

            fPadPlane->Draw("COL L0");
            fPadPlane -> SetMinimum(1.0);
            gStyle->SetOptStat(0);
            gStyle->SetPalette(103);
            gPad ->Update();


            std::cout << std::endl << std::endl;
            std::cout << "Macro finished succesfully."  << std::endl << std::endl;
            // -----   Finish   -------------------------------------------------------
            timer.Stop();
            Double_t rtime = timer.RealTime();
            Double_t ctime = timer.CpuTime();
            cout << endl << endl;
            cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
            cout << endl;

}
