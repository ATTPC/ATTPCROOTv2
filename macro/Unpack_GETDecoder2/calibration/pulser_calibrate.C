//*********************Gain Calibration*********************************
//*****                                                     ************
//*****  Input file: InputFile (root file)                  ************
//*****  Output file: OutputFile (.txt file)                ************
//*****                                                     ************
//*****  Output file format: pad number [tab] ratio         ************
//*****  Where ratio is = set amplitude/measured amplitude  ************
//*****                                                     ************
//*****  Takes one pulser event (evnt) and sets large &     ************
//*****  small pads to amplitude of your choosing           ************
//*****                                                     ************
//*****  Written by: Nathan Watwood (NSCL)                  ************
//**********************************************************************

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

#include <iostream>
#include <fstream>

void pulser_calibrate(TString InputFile = "dummy", TString gainOutput = "gainCalibration.txt",
                      TString jitterOutput = "jitterCalibration.txt"){


  Double_t bigPadAverage    = 1250;
  Double_t smallPadAverage  = 350;
  Double_t tbAverage        = 127;
  Double_t padSplit         = 800; //chooses big and small pads based off whether they are above
                                    //or below this amplitude

  TCanvas *c1 = new TCanvas();
  c1->Divide(1,2);

  TH2D *plot  = new TH2D("plot", "plot", 10240, 0,10240, 100, 0, 40);
  plot->SetMarkerStyle(20);
  plot->SetMarkerSize(.5);
  plot->SetXTitle("PadNum");
  plot->SetYTitle("Amplitude/100");

  TH2D *corrplot  = new TH2D("corrplot", "corrplot", 10240, 0,10240, 50, 0, 2);
  corrplot->SetMarkerStyle(20);
  corrplot->SetMarkerSize(.5);
  corrplot->SetXTitle("PadNum");
  corrplot->SetYTitle("Base/Amplitude");

  TString workdir = getenv("VMCWORKDIR");
  TString FilePath = workdir + "/macro/Unpack_GETDecoder2/";
  TString FileNameTail = ".root";
  TString FileName     = FilePath + InputFile + FileNameTail;
  std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
  TFile* file = new TFile(FileName.Data(),"READ");
  TTree* tree = (TTree*) file -> Get("cbmsim");
  Int_t nEvents = tree -> GetEntriesFast();
  std::cout<<" Number of events : "<<nEvents<<std::endl;

  TString outFilePath = workdir + "/macro/Unpack_GETDecoder2/calibration/";
  TString gainOutputFile = outFilePath + gainOutput;
  TString jitterOutputFile = outFilePath + jitterOutput;


  TTreeReader Reader1("cbmsim", file);
  TTreeReaderValue<TClonesArray> eventArray1(Reader1, "ATEventH");


  ofstream myfile, yourfile;
  myfile.open(gainOutputFile);
  yourfile.open(jitterOutputFile);

  Int_t evnt          = 0;
  Int_t nGPads        = 0;
  Int_t PadNum        = 0;
  Double_t charge     = 0;
  Double_t timestamp  = 0;
  Double_t dTime      = 0;
  Int_t intTime       = 0;
  std::vector<ATHit> goodHits;
  while (Reader1.Next()) {
    evnt++;
    ATEvent *event = (ATEvent*) eventArray1->At(0);
    if(evnt ==5){ //Change to the event you want to be the calibration event!!
    std::cout<<"Using event "<<cRED<<evnt<<cNORMAL<<" for calibration!"<<std::endl;
    goodHits       = event->GetHitArrayObj();
    nGPads         = goodHits.size();
      for(Int_t i = 0; i<nGPads; i++){
        charge = goodHits[i].GetCharge();
        PadNum = goodHits[i].GetHitPadNum();
        timestamp = goodHits[i].GetTimeStampCorr();
        dTime     = tbAverage-timestamp;
        intTime   = dTime;
        plot->Fill(PadNum, charge/100);
        yourfile << PadNum << " " << intTime << "\n";

        if (charge>padSplit){
          corrplot->Fill(PadNum, bigPadAverage/charge);
          myfile << PadNum << " " << bigPadAverage/charge << "\n";
        }
        else if ( charge < padSplit){
          corrplot->Fill(PadNum, smallPadAverage/charge);
          myfile << PadNum << " " << smallPadAverage/charge << "\n";
        }
      }
    }

    goodHits.clear();
  }


  //c1->cd(1);
  //plot->Draw();
  //c1->cd(2);
  //corrplot->Draw();

  myfile.close();
  yourfile.close();
}
