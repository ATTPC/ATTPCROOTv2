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
fA(0), fEbeam(0)
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
    Double_t px, Double_t py, Double_t pz, Double_t Ex, Double_t m, Double_t ener, Double_t ebeam)
    : fMult(0),
    fPx(0.), fPy(0.), fPz(0.),
    fR(0.), fOcal(0.), fOcalUncert(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(NULL),  fQ(0), fBeamSpotIsSet(kFALSE), fNomEner(0.),
    fA(0), fEbeam(0)
    {
      fgNIon++;
      fMult = mult;
      //fPx   = Double_t(a) * px;
      //fPy   = Double_t(a) * py;
      //fPz   = Double_t(a) * pz;
      fA  = a;
      fNomEner = ener;
      fEbeam = ebeam;
      //fVx   = vx;
      //fVy   = vy;
      //fVz   = vz;
      char buffer[20];
      sprintf(buffer, "FairIon%d", fgNIon);
      fIon= new FairIon(buffer, z, a, q,Ex,m);
      //  cout <<" Beam Ion mass : "<<fIon->GetMass()<<endl;
      gATVP->SetBeamMass(fIon->GetMass());
      gATVP->SetBeamNomE(ener);
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
    fA(right.fA), fEbeam(right.fEbeam)
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

      Double_t xFocus, yFocus, focz, angularFWHM, x0, y0, tanXPrime, tanYPrime, tanOffset;


      // if ( ! fIon ) {
      //   cout << "-W- FairIonGenerator: No ion defined! \n";
      //   return kFALSE;
      // }

      TParticlePDG* thisPart =
      TDatabasePDG::Instance()->GetParticle(fIon->GetName());
      if ( ! thisPart ) {
        cout << "-W- FairIonGenerator: Ion " << fIon->GetName()
        << " not found in database!\n";
        return kFALSE;
      }

      int pdgType = thisPart->PdgCode();
	
//startRand:

	fR = 1.0; //cm, FWHM of Gaussian
	angularFWHM = 20.*1E-3; //radians, use E-3 to convert from milirad
	fOcal = 50; //cm, distance from entrance
	
	//generate a particle at x0, y0, pick a spot at focus point (xFocus, yFocus) extrapolate between points for vector.
	//x0/y0 is coordinates of beam particle at ATTPC entrance, xFocus is coordinates at focus.
	xFocus = gRandom->Gaus(0,fR / 2.355);
	yFocus = gRandom->Gaus(0,fR / 2.355);
	
	x0 = fabs(gRandom->Gaus(xFocus, (fOcal) * tan(angularFWHM / 2.355)))*sign(xFocus);
	y0 = fabs(gRandom->Gaus(yFocus, (fOcal) * tan(angularFWHM / 2.355)))*sign(yFocus);

//	if(sqrt(x0*x0+y0*y0)>2.) goto startRand;

	//Optional: Offsets in x, x':
	xFocus+= 1.; //cm
	x0+=     1.; //cm
	x0-= tan(0*1E-3) * fOcal;
	 
      //fOcalUncert = 1.;
      //fOcal = 50.;
	fBeamSpotIsSet = kFALSE;  
	  if(fBeamSpotIsSet){  
        fVx   =x0 ;//xFocus; 
        fVy   =y0 ;//yFocus; 
        fVz   =0. ;//fOcal;
		
		//focz is the position where y = 0, x = tangentialOffset. (This can be negative, since it's only used in ATTPC_d2He to extrapolate a position 0<z<100)
		
		//Spatial angles
		tanYPrime = ((yFocus - y0)/fOcal);
		tanXPrime = ((xFocus - x0)/fOcal);
		
		focz = (-y0 / tanYPrime);
		tanOffset = focz * tanXPrime + x0; 

        //fPx   = sign(fVx)*(-sqrt( pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2) )*cos(atan(focz/fVx)));
        //fPy   = sign(fVy)*(-sqrt( pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2) )*cos(atan(sqrt(pow(focz,2)+pow(fVx,2))/fVy)));
		
        fPx   = sign(tanXPrime)*fabs((sqrt( pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2) )*cos(atan(fOcal/(xFocus - x0))))); 
        fPy   = sign(tanYPrime)*fabs((sqrt( pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2) )*cos(atan(sqrt(pow(focz,2)+pow(x0 - tanOffset,2))/y0))));
		fPz   = sqrt(pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2) - fPx*fPx - fPy*fPy);
		
/*		cout << "Debug: zFocus:" << focz<< " test focz "<<sqrt(x0*x0+y0*y0)*fOcal/(sqrt(x0*x0+y0*y0)-sqrt(xFocus*xFocus+yFocus*yFocus)) << " xPrime: " << atan(tanXPrime) << " Momentum x angle: " << fabs(cos(atan(focz/(tanOffset - x0))))*sign(fPx) << " yPrime:" << atan(tanYPrime) << " Momentum y angle: " << fabs(cos(atan(sqrt(pow(focz,2)+pow(x0 - tanOffset,2))/y0)))*sign(fPy) << endl;
		cout << "x0: " << x0 <<", y0: " << y0 << ", xFocus: " << xFocus << ", yFocus: " << yFocus << ", x-Offset:" << tanOffset << std::endl;
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
	gATVP->SetTanOffset(tanOffset);
	gATVP->Setfocz(focz/10.);//mm to cm

     /* if (fBeamSpotIsSet){
		  cout << "-I- FairIonGenerator: Generating " << fMult <<" with mass "<<thisPart->Mass() << " ions of type "
		  << fIon->GetName() << " (PDG code " << pdgType << ")\n";
		  cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz
		  << ") Gev from vertex (" << fVx << ", " << fVy << ", " << fVz << ") cm. TanOffset: " << tanOffset << " "<< fR << " test mom " <<sqrt( pow(fEbeam * fA / 1000.0 + thisPart->Mass(),2) - pow(thisPart->Mass(),2) )/fA<<endl; 
	    }
*/
      // cout << "-I- FairIonGenerator: Generating " << fMult <<" with mass "<<thisPart->Mass() << " ions of type "
      // << fIon->GetName() << " (PDG code " << pdgType << ")\n";
      // cout << "    Momentum (" << fPx << ", " << fPy << ", " << fPz
      // << ") Gev from vertex (" << fVx << ", " << fVy
      // << ", " << fVz << ") cm " <<endl;


      gATVP->IncBeamEvtCnt();


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
