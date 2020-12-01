void test_map3()
{




  TStopwatch timer;
  timer.Start();

  TString scriptfile = "e12014_pad_map_size.xml";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;


  AtTpcMap* fAtMapPtr = new AtTpcMap();
  fAtMapPtr->GenerateATTPC();
  TH2Poly *fPadPlane = fAtMapPtr->GetATTPCPlane();
  fAtMapPtr->Dump();
  Bool_t MapIn = fAtMapPtr->ParseXMLMap(scriptdir);





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
            std::vector<Float_t> PadCenterCoord;

            for(Int_t i=0;i<10240;i++)
            {
                PadCenterCoord = fAtMapPtr->CalcPadCenter(i);
                  x = PadCenterCoord[0];
                  y = PadCenterCoord[1];
                  int sizeval = fAtMapPtr->GetPadSize(i);
                  if(sizeval==0) z = 1;
                  else if(sizeval==1) z = 10;
                  else cout<<i<<"  "<<sizeval<<endl;
                  Int_t bin=  fPadPlane->Fill(x,y,z);
                  PadCenterCoord.clear();

            }

            fPadPlane->Draw("COL L0");
            fPadPlane -> SetMinimum(1.0);
            gStyle->SetOptStat(0);
            //gStyle->SetPalette(103);
            //gPad ->Update();


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
