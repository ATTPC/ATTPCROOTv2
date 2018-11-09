#include "ATPulseTask.hh"
#include "ATHit.hh"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "ATVertexPropagator.h"
#include "ATPad.hh"
#include "ATSimulatedPoint.hh"

// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TRandom.h"
#include "TMath.h"
#include "TF1.h"
#include "TH1.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"


ATPulseTask::ATPulseTask():FairTask("ATPulseTask"),
			   fEventID(0)
{
  LOG(INFO) << "Constructor of ATPulseTask" << FairLogger::endl;
}

ATPulseTask::~ATPulseTask()
{
  LOG(INFO) << "Destructor of ATPulseTask" << FairLogger::endl;
}

void
ATPulseTask::SetParContainers()
{
  LOG(INFO) << "SetParContainers of ATPulseTask" << FairLogger::endl;
  FairRunAna* ana = FairRunAna::Instance();
  FairRuntimeDb* rtdb = ana->GetRuntimeDb();
  fPar = (ATDigiPar*) rtdb->getContainer("ATDigiPar");
}

InitStatus
ATPulseTask::Init()
{
  LOG(INFO) << "Initilization of ATPulseTask" << FairLogger::endl;
  FairRootManager* ioman = FairRootManager::Instance();
  
  fDriftedElectronArray = (TClonesArray *) ioman -> GetObject("ATSimulatedPoint");
  if (fDriftedElectronArray == 0) {
    LOG(INFO) << "ERROR: Cannot find fDriftedElectronArray array!" << FairLogger::endl;
    return kERROR;
  }
  
  fRawEventArray  = new TClonesArray("ATRawEvent", 1); //!< Raw Event array (only one)
  ioman -> Register("ATRawEvent", "cbmsim", fRawEventArray, fIsPersistent);
  
  fGain = fPar->GetGain();
  std::cout<<"  Gain in ATTPC gas: "<<fGain<<std::endl;
  
  gain = new TF1("gain", "4*(x/[0])*pow(2.718, -2*(x/[0]))", 0, fGain*5);//Polya distribution of gain
  gain->SetParameter(0, fGain);
  //gain->SetParameter(1, getGain);
  
  //TODO: THIS SHOULD BE TAKEN FROM A NEW PARAMETER AS THE GAS GAIN
  //GET gain: 120fC, 240fC, 1pC or 10pC in 4096 channels
  // 1.6e-19*4096/120e-15 = 5.4613e-03
  // 1.6e-19*4096/240e-15 = 2.731e-03
  // 1.6e-19*4096/1e-12 = 6.5536e-04
  // 1.6e-19*4096/10e-12 = 6.5536e-05
  fGETGain = 5.4613e-03; //electrones(carga)*4096/120 fC
  std::cout<<"  GET Gain: " << fGETGain << std::endl;
  
  fTBTime = fPar->GetTBTime(); //Assuming 80 ns bucket
  fNumTbs = fPar->GetNumTbs();
  
  // ***************Create ATTPC Pad Plane***************************
  TString scriptfile = "Lookup20150611.xml";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  
  fMap = new AtTpcMap();
  fMap->GenerateATTPC();
  Bool_t MapIn = fMap->ParseXMLMap(scriptdir);
  fPadPlane = fMap->GetATTPCPlane();
  
  fEventID = 0;
  fRawEvent = NULL;
  
  /*TF1 ePulse(Form("ePulse_%i",iEvents),PadResponse,0,100,3);
    ePulse.SetParameter(0,fGain);
    ePulse.SetParameter(1,eTime);
    ePulse.SetParameter(2,tau);*/
  
  char buff[100];
  eleAccumulated = new TH1F*[10241];
  for(Int_t padS=0;padS<10240;padS++){
    sprintf(buff,"%d",padS);
    eleAccumulated[padS] = new TH1F(buff,buff,fNumTbs,0,fTBTime*fNumTbs/1000); //max time in microseconds from Time bucket size
  }
  
  return kSUCCESS;
}

Double_t PadResponse(Double_t *x, Double_t *par){
  return par[0] * TMath::Exp(-3.0*(x[0]-par[1])/par[2] )  * TMath::Sin((x[0]-par[1])/par[2]) * TMath::Power((x[0]-par[1])/par[2],3);
}

void ATPulseTask::Exec(Option_t* option) {
  LOG(INFO) << "Exec of ATPulseTask" << FairLogger::endl;
  Double_t tau  = 1.0; //shaping time (us)
  
  for(Int_t padS=0;padS<10240;padS++)
    eleAccumulated[padS]->Reset();
  
  Int_t nMCPoints = fDriftedElectronArray->GetEntries();
  std::cout<<" ATPulseTask: Number of Points "<<nMCPoints<<std::endl;
  if(nMCPoints<10){
    LOG(INFO) << "Not enough hits for digitization! (<10)" << FairLogger::endl;
    return;
  }
  
  electronsMap.clear();
  
  fRawEventArray->Delete();
  fRawEvent = NULL;
  fRawEvent = (ATRawEvent*)fRawEventArray->ConstructedAt(0);
  fPadPlane->Reset(0);
  Int_t size = fRawEventArray -> GetEntriesFast(); //will be always 1
  
  //TFile output("test_output.root","recreate");
  
  //Distributing electron pulses among the pads
  for(Int_t iEvents = 0; iEvents<nMCPoints; iEvents++){//for every electron
    
    auto dElectron   = (ATSimulatedPoint*) fDriftedElectronArray->At(iEvents);
    auto coord       = dElectron->GetPosition();
    auto xElectron   = coord (0); //mm
    auto yElectron   = coord (1); //mm
    auto eTime       = coord (2); //us
    auto padNumber   = (int)fPadPlane->Fill(xElectron,yElectron) - 1;
    
    if(padNumber<0 || padNumber>10240) continue;
    
    //std::cout<<padNumber<<"  "<<coord(0)<<"  "<<coord(1)<<"  "<<coord(2)<<"\n";
    //std::cout << eTime << " " <<std::endl;
    
    std::map<Int_t,TH1F*>::iterator ite = electronsMap.find(padNumber);
    if(ite == electronsMap.end()){
      eleAccumulated[padNumber]->Fill(eTime);
      electronsMap[padNumber]=eleAccumulated[padNumber];
    }
    else{
      eleAccumulated[padNumber] = (ite->second);
      eleAccumulated[padNumber]->Fill(eTime);
      electronsMap[padNumber]=eleAccumulated[padNumber];
    }
  }
  //std::cout << "...End of collection of electrons in this event."<< std::endl;
  TAxis* axis = eleAccumulated[0]->GetXaxis();
  Double_t binWidth = axis->GetBinWidth(10);
  
  //output.cd();
  
  std::vector<Float_t> PadCenterCoord;
  std::map<Int_t,TH1F*>::iterator ite2 = electronsMap.begin();
  Int_t signal[fNumTbs];
  
  //Double_t *thePar = new Double_t[3];
  while(ite2!=electronsMap.end()) {
    for(Int_t kk=0;kk<fNumTbs;kk++) signal[kk]=0;
    Int_t thePadNumber = (ite2->first);
    eleAccumulated[thePadNumber] = (ite2->second);
    // Set Pad and add to event
    ATPad *pad = new ATPad();
    for(Int_t kk=0;kk<fNumTbs;kk++) {
      if(eleAccumulated[thePadNumber]->GetBinContent(kk)>0) {
	for(Int_t nn=kk;nn<fNumTbs;nn++) {
	  Double_t binCenter = axis->GetBinCenter(kk);
	  Double_t factor =(((((Double_t)nn)+0.5)*binWidth)-binCenter)/tau;
 	  signal[nn] += eleAccumulated[thePadNumber]->GetBinContent(kk)*pow(2.718,-3*factor)*sin(factor)*pow(factor,3);
	}
      }
    }
    
    pad->SetPad(thePadNumber);
    PadCenterCoord = fMap->CalcPadCenter(thePadNumber);
    pad->SetValidPad(kTRUE);
    pad->SetPadXCoord(PadCenterCoord[0]);
    pad->SetPadYCoord(PadCenterCoord[1]);
    pad->SetPedestalSubtracted(kTRUE);
    Double_t g = gain->GetRandom();
    for(Int_t bin = 0; bin<fNumTbs; bin++){
      pad->SetADC(bin,signal[bin]*g*fGETGain);
    }
    fRawEvent->SetPad(pad);
    fRawEvent->SetEventID(fEventID);
    delete pad;
    //std::cout <<" Event "<<aux<<" "<<electronsMap.size()<< "*"<< std::flush;
    ite2++;
  }
  fEventID++;
  return;
}

ClassImp(ATPulseTask);
