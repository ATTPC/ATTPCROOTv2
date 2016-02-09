// -------------------------------------------------------------------------
// -----            Based on FairIonGenerator source file              -----
// -----            Created 30/01/15  by Y. Ayyad                      -----
// -------------------------------------------------------------------------
#include "ATTPCIonGenerator.h"
#include "ATVertexPropagator.h"

#include "FairPrimaryGenerator.h"

#include "FairIon.h"
#include "FairRunSim.h"

#include "TDatabasePDG.h"
#include "TParticlePDG.h"
#include "TObjArray.h"
                    
#include "TRandom.h"
#include "TMath.h"
#include "TLorentzVector.h"
  
#include "FairRunSim.h"
#include "FairIon.h"
#include <iostream>
#include "TParticle.h"
using std::cout;
using std::endl;

// -----   Initialsisation of static variables   --------------------------
Int_t ATTPCIonGenerator::fgNIon = 0;
// ------------------------------------------------------------------------



// -----   Default constructor   ------------------------------------------
ATTPCIonGenerator::ATTPCIonGenerator()
  : fMult(0),          
    fPx(0.), fPy(0.), fPz(0.),
    fR(0.), fz(0.), fOffset(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(NULL),  fQ(0), fBeamSpotIsSet(kFALSE)
{
//  cout << "-W- ATTPCIonGenerator: "
//      << " Please do not use the default constructor! " << endl;
}
// ------------------------------------------------------------------------



ATTPCIonGenerator::ATTPCIonGenerator(const Char_t* ionName, Int_t mult,
				 Double_t px, Double_t py, Double_t pz)
  : fMult(0),          
    fPx(0.), fPy(0.), fPz(0.),
    fR(0.), fz(0.), fOffset(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(NULL),  fQ(0), fBeamSpotIsSet(kFALSE)
{

  FairRunSim *fRun=FairRunSim::Instance();
  TObjArray *UserIons=fRun->GetUserDefIons();
  TObjArray *UserParticles=fRun->GetUserDefParticles();
  FairParticle *part=0;
  fIon =(FairIon *) UserIons->FindObject(ionName);
  if(fIon){
     fgNIon++;
     fMult = mult;
     fPx   = Double_t(fIon->GetA()) * px;
     fPy   = Double_t(fIon->GetA()) * py;
     fPz   = Double_t(fIon->GetA()) * pz;
     //fVx   = vx; 
     //fVy   = vy; 
     //fVz   = vz;
       //}

  }else{
     part= (FairParticle *)UserParticles->FindObject(ionName);
     if(part){
        fgNIon++;
        TParticle *particle=part->GetParticle();
        fMult = mult;
        fPx   = Double_t(particle->GetMass()/0.92827231) * px;
        fPy   = Double_t(particle->GetMass()/0.92827231) * py;
        fPz   = Double_t(particle->GetMass()/0.92827231) * pz;
	//fVx   = vx;
	//fVy   = vy;
	//fVz   = vz;
     }
  }
  if(fIon==0 && part==0 ){
     cout << "-E- ATTPCIonGenerator: Ion or Particle is not defined !" << endl;
     Fatal("ATTPCIonGenerator", "No FairRun instantised!");      
  }
    
}
// ------------------------------------------------------------------------



// -----   Default constructor   ------------------------------------------
ATTPCIonGenerator::ATTPCIonGenerator(const char* name,Int_t z, Int_t a, Int_t q, Int_t mult,
				 Double_t px, Double_t py, Double_t pz, Double_t Ex, Double_t m, Double_t ener)
  : fMult(0),          
    fPx(0.), fPy(0.), fPz(0.),
    fR(0.), fz(0.), fOffset(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(NULL),  fQ(0), fBeamSpotIsSet(kFALSE), fNomEner(0.)
 {
  fgNIon++;
  fMult = mult;
  fPx   = Double_t(a) * px;
  fPy   = Double_t(a) * py;
  fPz   = Double_t(a) * pz;
  fNomEner = ener;
  //fVx   = vx; 
  //fVy   = vy; 
  //fVz   = vz; 
  char buffer[20];
  sprintf(buffer, "FairIon%d", fgNIon);
  fIon= new FairIon(buffer, z, a, q,Ex,m); 
  cout <<" Beam Ion mass : "<<fIon->GetMass()<<endl;
  gATVP->SetBeamMass(fIon->GetMass());
  gATVP->SetBeamNomE(ener);
  FairRunSim* run = FairRunSim::Instance();
  if ( ! run ) {
    cout << "-E- FairIonGenerator: No FairRun instantised!" << endl;
    Fatal("FairIonGenerator", "No FairRun instantised!");
  }
  run->AddNewIon(fIon);
}
//_________________________________________________________________________



ATTPCIonGenerator::ATTPCIonGenerator(const ATTPCIonGenerator& right)
  : fMult(right.fMult),          
    fPx(right.fPx), fPy(right.fPy), fPz(right.fPz),
    fR(right.fR), fz(right.fz), fOffset(right.fOffset),
    fVx(right.fVx), fVy(right.fVy), fVz(right.fVz),
    fIon(right.fIon), fQ(right.fQ), fBeamSpotIsSet(right.fBeamSpotIsSet)
{
}


// -----   Destructor   ---------------------------------------------------
ATTPCIonGenerator::~ATTPCIonGenerator()
{
 // if (fIon) delete fIon;
}
//_________________________________________________________________________



// -----   Public method SetExcitationEnergy   ----------------------------
void ATTPCIonGenerator::SetExcitationEnergy(Double_t eExc) {
  fIon->SetExcEnergy(eExc);
}
//_________________________________________________________________________



// -----   Public method SetMass   ----------------------------------------
void ATTPCIonGenerator::SetMass(Double_t mass) {
  fIon->SetMass(mass);
}
//_________________________________________________________________________



// -----   Public method ReadEvent   --------------------------------------
Bool_t ATTPCIonGenerator::ReadEvent(FairPrimaryGenerator* primGen) {

  Double_t Phi, SpotR;

 // if ( ! fIon ) {
 //   cout << "-W- FairIonGenerator: No ion defined! " << endl;
 //   return kFALSE;
 // }

  TParticlePDG* thisPart = 
    TDatabasePDG::Instance()->GetParticle(fIon->GetName());
  if ( ! thisPart ) {
    cout << "-W- FairIonGenerator: Ion " << fIon->GetName()
	 << " not found in database!" << endl;
    return kFALSE;
  }

  int pdgType = thisPart->PdgCode();

  //cout << "fR=" << fR << " fz=" << fz <<endl;

  Phi= gRandom->Uniform(0,360)*TMath::DegToRad();
  SpotR=gRandom->Uniform(0,fR);

  if(fBeamSpotIsSet) {
    fVx   = SpotR*cos(Phi); //gRandom->Uniform(-fx,fx);
    fVy   = fOffset + SpotR*sin(Phi); //gRandom->Uniform(-fy,fy);
    fVz   = fz;
  }else
    {
      fVx=0.0;
      fVy=0.0;
      fVz=0.0;
    }

  cout << "-I- FairIonGenerator: Generating " << fMult <<" with mass "<<thisPart->Mass() << " ions of type "
       << fIon->GetName() << " (PDG code " << pdgType << ")" << endl;
  cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz 
       << ") Gev from vertex (" << fVx << ", " << fVy
       << ", " << fVz << ") cm" << endl;

  gATVP->IncBeamEvtCnt(); 


       if(gATVP->GetBeamEvtCnt()%2!=0){
	  Double_t Er = gRandom->Uniform(0.,fNomEner);
  	  gATVP->SetRndELoss(Er);
           std::cout<<" Random Energy ATTPCIonGenerator : "<<Er<<std::endl;
	}

  for(Int_t i=0; i<fMult; i++)
    primGen->AddTrack(pdgType, fPx, fPy, fPz, fVx, fVy, fVz);

   
  
  return kTRUE;

}

//_____________________________________________________________________________


ClassImp(ATTPCIonGenerator)
