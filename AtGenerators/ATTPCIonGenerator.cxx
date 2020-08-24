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
fR(0.), fOcal(0.), fOcalUncert(0.),
fVx(0.), fVy(0.), fVz(0.),
fIon(NULL),  fQ(0), fBeamSpotIsSet(kFALSE),
fA(0), fEbeam(0), fwhmFocus(0), angularDiv(0), zFocus(0)
{
  //  cout << "-W- ATTPCIonGenerator: "
  //      << " Please do not use the default constructor! \n";
}
// ------------------------------------------------------------------------



ATTPCIonGenerator::ATTPCIonGenerator(const Char_t* ionName, Int_t mult,
  Double_t px, Double_t py, Double_t pz)
  : fMult(0),
  fPx(0.), fPy(0.), fPz(0.),
  fR(0.), fOcal(0.), fOcalUncert(0.),
  fVx(0.), fVy(0.), fVz(0.),
  fIon(NULL),  fQ(0), fBeamSpotIsSet(kFALSE),
  fA(0), fEbeam(0)
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
      cout << "-E- ATTPCIonGenerator: Ion or Particle is not defined !\n";
      Fatal("ATTPCIonGenerator", "No FairRun instantised!");
    }

  }
  // ------------------------------------------------------------------------



  // -----   Default constructor   ------------------------------------------
  ATTPCIonGenerator::ATTPCIonGenerator(const char* name,Int_t z, Int_t a, Int_t q, Int_t mult,
    Double_t px, Double_t py, Double_t pz, Double_t Ex, Double_t m, Double_t ener, Double_t ebeam, Double_t beam1, Double_t beam2
    , Double_t beam3)
    : fMult(0),
    fPx(0.), fPy(0.), fPz(0.),
    fR(0.), fOcal(0.), fOcalUncert(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(NULL),  fQ(0), fBeamSpotIsSet(kFALSE), fNomEner(0.),
    fA(0), fEbeam(0), fwhmFocus(0), angularDiv(0), zFocus(0)
    {
      fgNIon++;
      fMult = mult;
      //fPx   = Double_t(a) * px;
      //fPy   = Double_t(a) * py;
      //fPz   = Double_t(a) * pz;
      fA  = a;
      fNomEner = ener;
      fEbeam = ebeam;
      fwhmFocus = beam1;
      angularDiv = beam2;
      zFocus = beam3;
      //fVx   = vx;
      //fVy   = vy;
      //fVz   = vz;
      char buffer[20];
      sprintf(buffer, "FairIon%d", fgNIon);
      fIon= new FairIon(buffer, z, a, q,Ex,m);
      //  cout <<" Beam Ion mass : "<<fIon->GetMass()<<endl;
      gATVP->SetBeamMass(fIon->GetMass());
      gATVP->SetBeamNomE(ener);
      if(fwhmFocus>0 && angularDiv>0 && zFocus>0) {fBeamSpotIsSet = kTRUE;  }
      FairRunSim* run = FairRunSim::Instance();
      if ( ! run ) {
        cout << "-E- FairIonGenerator: No FairRun instantised!\n";
        Fatal("FairIonGenerator", "No FairRun instantised!");
      }
      run->AddNewIon(fIon);
    }
    //_________________________________________________________________________



    ATTPCIonGenerator::ATTPCIonGenerator(const ATTPCIonGenerator& right)
    : fMult(right.fMult),
    fPx(right.fPx), fPy(right.fPy), fPz(right.fPz),
    fR(right.fR), fOcal(right.fOcal), fOcalUncert(right.fOcalUncert),
    fVx(right.fVx), fVy(right.fVy), fVz(right.fVz),
    fIon(right.fIon), fQ(right.fQ), fBeamSpotIsSet(right.fBeamSpotIsSet),
    fA(right.fA), fEbeam(right.fEbeam),
    fwhmFocus(right.fwhmFocus), angularDiv(right.angularDiv), zFocus(right.zFocus)
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


    inline Double_t sign(Double_t num) {
        if (num > 0) return 1;
        return (num == 0) ? 1 : -1;
    }
    //_________________________________________________________________________

    // -----   Public method ReadEvent   --------------------------------------
    Bool_t ATTPCIonGenerator::ReadEvent(FairPrimaryGenerator* primGen) {

      	Double_t xFocus, yFocus, x0, y0, theta, phi;
      	Double_t rHole=2.5;

      	TParticlePDG* thisPart =
      	TDatabasePDG::Instance()->GetParticle(fIon->GetName());
      	if ( ! thisPart ) {
        	cout << "-W- FairIonGenerator: Ion " << fIon->GetName()
        	<< " not found in database!\n";
        	return kFALSE;
     	}

      	int pdgType = thisPart->PdgCode();
      
 	Double_t fPtot=sqrt(pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2));
	
/*
	fwhmFocus = 1.; //cm, FWHM of Gaussian
	angularDiv = 10.*1E-3; //radians, use E-3 to convert from milirad
	zFocus = 50; //cm, distance from entrance
*/	
	//x0/y0 is coordinates of beam particle at ATTPC entrance, xFocus is coordinates at focus.
	xFocus = gRandom->Gaus(0,fwhmFocus / 2.355);
	yFocus = gRandom->Gaus(0,fwhmFocus / 2.355);
	do{
	theta = gRandom->Uniform(-angularDiv,angularDiv);
	phi = gRandom->Uniform(-angularDiv,angularDiv);
	x0 = xFocus-zFocus*tan(phi);
	y0 = yFocus-sqrt(pow(zFocus,2)+pow(xFocus-x0,2))*tan(theta);
	}
	while(sqrt(pow(x0,2)+pow(y0,2))>rHole && sqrt(pow(tan(theta),2)+pow(tan(phi),2))>tan(angularDiv));
	
	if(fBeamSpotIsSet){  
        	fVx   =x0 ;
        	fVy   =y0 ; 
        	fVz   =0. ;

		fPx=fPtot*cos(theta)*sin(phi);
		fPy=fPtot*sin(theta);
		fPz=sqrt(fPtot*fPtot - fPx*fPx - fPy*fPy);
/*
		cout << "x0: " << x0 <<", y0: " << y0 << ", xFocus: " << xFocus << ", yFocus: " << yFocus << ", zFocust:" << zFocus << std::endl;
		cout<< "Px: "<< fPx<<" Py: "<<fPy<<"fPz :"<<fPz<<endl;
		cout<<"theta: "<<theta<<"phi: "<<phi<<endl;
*/
	
      	}
      	else if(!fBeamSpotIsSet){
        	fVx=0.0;
        	fVy=0.0;
        	fVz=0.0;
	//	std::cout<<"THE BEAM SPOT WAS DETERMINED NOT TO BE SET \n"<<std::endl;
        	fPx   = 0.;
        	fPy   = 0.;
        	fPz   = sqrt(pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2) - fPx*fPx - fPy*fPy);
      }
      // cout << "-I- FairIonGenerator: Generating " << fMult <<" with mass "<<thisPart->Mass() << " ions of type "
      // << fIon->GetName() << " (PDG code " << pdgType << ")\n";
      // cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz
      // << ") Gev from vertex (" << fVx << ", " << fVy
      // << ", " << fVz << ") cm " <<endl;


      gATVP->IncBeamEvtCnt();
      gATVP->Setd2HeVtx(fVx,fVy,theta,phi);


      if(gATVP->GetBeamEvtCnt()%2!=0){
        Double_t Er = gRandom->Uniform(0.,fNomEner);
        gATVP->SetRndELoss(Er);
        //           std::cout<<" Random Energy ATTPCIonGenerator : "<<Er<<std::endl;
      }

      for(Int_t i=0; i<fMult; i++)
      primGen->AddTrack(pdgType, fPx, fPy, fPz, fVx, fVy, fVz);



      return kTRUE;

    }

    //_____________________________________________________________________________


    ClassImp(ATTPCIonGenerator)
