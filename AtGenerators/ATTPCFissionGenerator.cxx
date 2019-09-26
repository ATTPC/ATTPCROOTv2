#include "ATTPCFissionGenerator.h"

#include "FairPrimaryGenerator.h"
#include "FairRootManager.h"
#include "FairLogger.h"
#include "FairMCEventHeader.h"

#include "FairIon.h"
#include "FairParticle.h"
#include "FairRunSim.h"
#include "FairRunAna.h"

#include "TDatabasePDG.h"
#include "TParticlePDG.h"
#include "TObjArray.h"


#include "TRandom.h"
#include "TMath.h"
#include "TLorentzVector.h"
#include "TVector3.h"
#include "TGenPhaseSpace.h"
#include "TVirtualMC.h"
#include "TParticle.h"
#include "TClonesArray.h"
#include "TFile.h"
#include "TTree.h"
#include "TTreeReader.h"
#include "TTreePlayer.h"
#include "TTreeReaderValue.h"


#include "FairRunSim.h"
#include "FairIon.h"
#include <iostream>
#include "TParticle.h"

#include "AtStack.h"
#include "AtTpcPoint.h"
#include "ATVertexPropagator.h"
#include "ATEulerTransformation.h"

#include "TVector3.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ATTPCFissionGenerator::ATTPCFissionGenerator()
  :
    fP1x(0.), fP1y(0.), fP1z(0.),
    fP2x(0.), fP2y(0.), fP2z(0.),
    fVx(0.), fVy(0.), fVz(0.)
{
    //  cout << "-W- ATTPCIonGenerator: "
    //      << " Please do not use the default constructor! " << endl;
}

ATTPCFissionGenerator::ATTPCFissionGenerator(const char* name,TString simfile)
  :
    fP1x(0.), fP1y(0.), fP1z(0.),
    fP2x(0.), fP2y(0.), fP2z(0.),
    fVx(0.), fVy(0.), fVz(0.)
{

        fPDG=TDatabasePDG::Instance();
        // INCL+ABLA file
        //simfile ="240Cf.root";
        TString dir = getenv("VMCWORKDIR");
        TString simfilepath = dir + "/macro/Simulation/data/"+ simfile;
        TFile *f = new TFile(simfilepath.Data());
        if(f->IsZombie()){
        std::cout<<cRED<<" ATTPCFissionGenerator: No simulation file found! Check VMCWORKDIR variable. Exiting... "<<cNORMAL<<std::endl;
     		delete f;
        }else std::cout<<cGREEN<<" ATTPCFissionGenerator : Prototype geometry found in : "<<simfilepath.Data()<<cNORMAL<<std::endl;

        fTree = (TTree*) f-> Get("tree101");
        Int_t nEvents = fTree -> GetEntriesFast();
        std::cout<<" Number of events : "<<nEvents<<std::endl;
        fTree->SetBranchAddress("Evnt",&Evnt);
        fTree->SetBranchAddress("Ntrack",&Ntrack);
        fTree->SetBranchAddress("Aout",Aout);
        fTree->SetBranchAddress("Zout",Zout);
        fTree->SetBranchAddress("fOutPx",fOutPx);
        fTree->SetBranchAddress("fOutPy",fOutPy);
        fTree->SetBranchAddress("fOutPz",fOutPz);

        event=0;

      /*  fFileNamebase = dir+"/macro/Simulation/database/ionlist.txt";
        std::cout << " ATTPCFissionGenerator: Opening input file " << fFileNamebase << std::endl;
         // Open first the file to register all new ions.
        fInputFilebase = new std::ifstream(fFileNamebase);
        if ( ! fInputFilebase->is_open() )
             Fatal("ATTPCFissionGenerator","Cannot open input file.");

        std::cout << "ATTPCFissionGenerator: Looking for ions..." << std::endl;

        //Int_t nIons = RegisterIons();
        //std::cout <<cYELLOW<< "ATTPCFissionGenerator: " << nIons << " ions registered."<<cNORMAL<< std::endl;*/




}


ATTPCFissionGenerator::~ATTPCFissionGenerator()
{

}

Bool_t ATTPCFissionGenerator::ReadEvent(FairPrimaryGenerator* primGen) {

  fVx=0.,fVy=0.,fVz=0.;
  Double_t uma=931.494028,mp=938.272013, c=29.972458;
  Double_t px,py,pz;

  fTree->GetEntry(event);

  // Define event
/*  Int_t     I = 0;

  // Define track variables
  Int_t    iPid   = -1;
  Int_t    ia1      = 12;
  Int_t    iz1      = 6;
  Int_t pdgType=0;

  for(Int_t j=0;j<Ntrack;j++){

      ia1=Aout[j];
      iz1=Zout[j];

  if(ia1>2 && iz1>2){
   if ( iPid < 0 ) {
	  char ionName[20];
	  sprintf(ionName, "Ion_%d_%d", ia1, iz1);
	  TParticlePDG* part = fPDG->GetParticle(ionName);
	  if ( ! part ) {
	      std::cout << "ATTPCFissionGenerator::ReadEvent: Cannot find "
		  << ionName << " in database!" << std::endl;
	      //continue;
	  }
	  if(part) pdgType = part->PdgCode();
      }
      else pdgType = ia1;  // "normal" particle

     px=(double)fOutPx[j];
     py=(double)fOutPy[j];
     pz=(double)fOutPz[j];

     primGen->AddTrack(pdgType, px, py, pz, fVx, fVy, fVz);

   }

 }*/

  std::cout<<cRED<<" Fission event : "<<event<<cNORMAL<<std::endl;
  event++;
  return kTRUE;

}

/*Int_t ATTPCFissionGenerator::RegisterIons() {

  Int_t nIons = 0;
  Int_t eventId=1;
  Double_t pBeam,b;

  // Define track variables to be read from file
  Int_t    iPid   = -1;
  Int_t    A,Z,qq;


  fIonMap.clear();

  *fInputFilebase>> Z >> A;

  while ( ! fInputFilebase->eof()) {


  if ( fInputFilebase->eof() ) continue;

	char buffer[20];
	sprintf(buffer, "Ion_%d_%d", A, Z);
	TString ionName(buffer);

	  FairIon* ion = new FairIon(ionName,Z,A,qq);
	  fIonMap[ionName] = ion;
	  nIons++;

     *fInputFilebase>> Z >> A;

  }//!

  FairRunSim* run = FairRunSim::Instance();
  std::map<TString, FairIon*>::iterator mapIt;
  for (mapIt=fIonMap.begin(); mapIt!=fIonMap.end(); mapIt++) {
    FairIon* ion = (*mapIt).second;
    run->AddNewIon(ion);

  }

  return nIons;
}*/



ClassImp(ATTPCFissionGenerator)
