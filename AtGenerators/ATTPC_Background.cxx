#include "ATTPC_Background.h"

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


#include "FairRunSim.h"
#include "FairIon.h"
#include <iostream>
#include "TParticle.h"

#include "AtStack.h"
#include "AtTpcPoint.h"
#include "ATVertexPropagator.h"
#include "ATEulerTransformation.h"

#include "TVector3.h"

#define amu 931.494

Int_t ATTPC_Background::fgNIon = 0;


ATTPC_Background::ATTPC_Background()
  : fMult(0),
    fPx(0.), fPy(0.), fPz(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(0)
{
//  cout << "-W- ATTPCIonGenerator: "
//      << " Please do not use the default constructor! " << endl;
}

// -----   Default constructor   ------------------------------------------
ATTPC_Background::ATTPC_Background(const char* name,std::vector<Int_t> *z,std::vector<Int_t> *a,std::vector<Int_t> *q, Int_t mult, std::vector<Double_t> *px,
	std::vector<Double_t>* py,std::vector<Double_t> *pz, std::vector<Double_t> *mass, std::vector<Double_t> *Ex)
 : fMult(0),
    fPx(0.), fPy(0.), fPz(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(0)
{


  fgNIon++;
  fMult = mult;
  fIon.reserve(fMult);
  
  char buffer[20];
  TDatabasePDG* pdgDB = TDatabasePDG::Instance();
  TParticlePDG* kProtonPDG = pdgDB->GetParticle(2212);
  TParticle* kProton = new TParticle();
  kProton->SetPdgCode(2212);

	
	

      for(Int_t i=0;i<fMult;i++){


       	fPx.push_back( px->at(i) );
	fPy.push_back( py->at(i) );
	fPz.push_back( pz->at(i) );
	Masses.push_back(mass->at(i));
        fExEnergy.push_back(Ex->at(i));
        //fWm.push_back( mass->at(i));

        FairIon *IonBuff;
        FairParticle *ParticleBuff;
        sprintf(buffer, "Product_Ion%d", i);
        if( a->at(i)!=1  ){
          IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i),0.0,mass->at(i)*amu/1000.0);
          ParticleBuff = new FairParticle("dummyPart",1,1,1.0,0,0.0,0.0);
          fPType.push_back("Ion");
          std::cout<<" Adding : "<<buffer<<std::endl;

        }else if( a->at(i)==1 && z->at(i)==1  ){

          IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i),0.0,mass->at(i)*amu/1000.0);
          ParticleBuff = new FairParticle(2212,kProton);
          fPType.push_back("Proton");
	}

	std::cout<<" Z "<<z->at(i)<<" A "<<a->at(i)<<std::endl;
	//std::cout<<buffer<<std::endl;
        fIon.push_back(IonBuff);
        fParticle.push_back(ParticleBuff);

       }

  FairRunSim* run = FairRunSim::Instance();
  if ( ! run ) {
    std::cout << "-E- FairIonGenerator: No FairRun instantised!" << std::endl;
    Fatal("FairIonGenerator", "No FairRun instantised!");
  }

   for(Int_t i=0;i<fMult;i++){

  	if(fPType.at(i)=="Ion"){
                 std::cout<<" In position "<<i<<" adding an : "<<fPType.at(i)<<std::endl;
		 run->AddNewIon(fIon.at(i));
		 std::cout<<" fIon name :"<<fIon.at(i)->GetName()<<std::endl;
                 std::cout<<" fParticle name :"<<fParticle.at(i)->GetName()<<std::endl;

        }else if(fPType.at(i)=="Proton"){
		 std::cout<<" In position "<<i<<" adding an : "<<fPType.at(i)<<std::endl;
		 run->AddNewParticle(fParticle.at(i));
                 std::cout<<" fIon name :"<<fIon.at(i)->GetName()<<std::endl;
                 std::cout<<" fParticle name :"<<fParticle.at(i)->GetName()<<std::endl;
                 std::cout<<fParticle.at(i)->GetName()<<std::endl;
	}

    }



}

// -----   Destructor   ---------------------------------------------------
ATTPC_Background::~ATTPC_Background()
{
 // if (fIon) delete fIon;
}






Double_t ATTPC_Background::omega(Double_t x, Double_t y, Double_t z){
		return sqrt(x*x + y*y + z*z -2*x*y -2*y*z -2*x*z);
		
	}


 Double_t*  ATTPC_Background::TwoB(Double_t  m1b, Double_t  m2b, Double_t  m3b, Double_t m4b, Double_t Kb, Double_t thetacm){


        static Double_t  kinrec[2];

        double Et1 = Kb + m1b;
        double Et2 = m2b;

        double s = pow(m1b,2) + pow(m2b,2) +2*m2b*Et1;
        double t = 0.;
        double u = 0.;

        double a =4.*m2b*s;
        double b =(pow(m3b,2)-pow(m4b,2)+s)*(pow(m1b,2)-pow(m2b,2)-s);
        double c = ATTPC_Background::omega(s,pow(m1b,2),pow(m2b,2))*ATTPC_Background::omega(s,pow(m3b,2),pow(m4b,2));


        double Et3 = (c*cos((180.-thetacm)*3.1415926535/180)-b)/a; //estamos viendo el recoil en cm (pi-theta)
	double Et4 = (Et1 + m2b - Et3);
	
	double K3 = Et3 - m3b;
	double K4 = Et4 - m4b;

        //------------------Mandestam variables	
	t =  pow(m2b,2) + pow(m4b,2) - 2*m2b*Et4;
	u =  pow(m2b,2) + pow(m3b,2) - 2*m2b*Et3;



        double theta_lab = acos(((s-pow(m1b,2)-pow(m2b,2))*(pow(m2b,2)+pow(m3b,2)-u)+2.*pow(m2b,2)*(pow(m2b,2)+pow(m4b,2)-s-u))/(ATTPC_Background::omega(s,pow(m1b,2),pow(m2b,2))*ATTPC_Background::omega(u,pow(m2b,2),pow(m3b,2))));

        kinrec[0] = K3;
        kinrec[1] = theta_lab;

        //std::cout<<K3 <<"  "<<K4<<"  "<<theta_lab<<std::endl;

        return kinrec;

        }



// -----   Public method ReadEvent   --------------------------------------
Bool_t ATTPC_Background::ReadEvent(FairPrimaryGenerator* primGen) {


		
	  	AtStack* stack = (AtStack*) gMC->GetStack();

   		fIsDecay = kFALSE;


		
  
		fBeamEnergy = gATVP->GetEnergy();
   		std::cout<<" -I- ATTPC_Background Residual energy  : "<<gATVP->GetEnergy()<<std::endl;

   		fPxBeam = gATVP->GetPx();
   		fPyBeam = gATVP->GetPy();
   		fPzBeam = gATVP->GetPz();

                 //fPxBeam = fPx.at(0) ;
   		 //fPyBeam = fPy.at(0) ;
   		 //fPzBeam = fPz.at(0) ;

                
		  if(fBeamEnergy==0){
    			std::cout << "-I- ATTP_Background : No solution!"<<std::endl;    			
    			gATVP->SetValidKine(kFALSE);
   			}
	
		if(!gATVP->GetValidKine()){

		fPx.at(2) = 0.; // To GeV for FairRoot
      		fPy.at(2) = 0.;
	    	fPz.at(2) = 0.;

      		fPx.at(3) = 0.;
      		fPy.at(3) = 0.;
	    	fPz.at(3) = 0.;

		fPx.at(4) = 0.;
      		fPy.at(4) = 0.;
	    	fPz.at(4) = 0.;

		fPx.at(5) = 0.;
      		fPy.at(5) = 0.;
	    	fPz.at(5) = 0.;

		
		}

                
	        	

		
                

		 m1 = Masses.at(0)*amu + fExEnergy.at(0);  
                 m2 = Masses.at(1)*amu + fExEnergy.at(1);  
                 m3 = Masses.at(2)*amu ;  //recoil 1
                 m4 = Masses.at(3)*amu + fExEnergy.at(3);  //ejectile 1
                 m7 = Masses.at(4)*amu ;  //recoil 2
                 m8 = Masses.at(5)*amu + fExEnergy.at(5);  //ejectile 2
		 K1 = sqrt(pow(fPx.at(0),2) + pow( fPy.at(0),2) + pow( fPz.at(0),2) + pow(m1,2)) - m1;
		 //K1 = sqrt(pow(fPxBeam*1000.0,2) + pow(fPyBeam*1000.0,2) + pow( fPzBeam*1000.0,2) + pow(m1,2)) - m1;

		

                Double_t fThetaCmsMin = 0.;
                Double_t fThetaCmsMax = 50;
                

                Double_t costhetamin = TMath::Cos(fThetaCmsMin*TMath::DegToRad());
                Double_t costhetamax = TMath::Cos(fThetaCmsMax*TMath::DegToRad());         



                //proton 1 from (d,p)
                ////uniform thetacm distribution between thetamin and thetamax
                Double_t thetacmsInput = TMath::ACos( (costhetamax - costhetamin )*gRandom->Uniform() + costhetamin )*TMath::RadToDeg(); 
                Double_t* kin2B1 = ATTPC_Background::TwoB(m1, m2, m3, m4, K1, thetacmsInput);
                Double_t phi1 = 2*TMath::Pi() * gRandom->Uniform();         //flat probability in phi
                Double_t krec = *(kin2B1+0);
                Double_t angrec = *(kin2B1+1);
                Prec = sqrt( pow(krec,2) + 2*krec*m3);
                fPx.at(2) = (Prec*sin(angrec)*cos(phi1) )/1000.0; // To GeV for FairRoot
      		fPy.at(2) = (Prec*sin(angrec)*sin(phi1) )/1000.0; // To GeV for FairRoot
	    	fPz.at(2) = (Prec*cos(angrec) )/1000.0; // To GeV for FairRoot
                //std::cout<<"Kin 1  "<< angrec<<"  "<<krec<<std::endl;

                
                
                //proton 2 from (d,p)
                ////uniform thetacm distribution between thetamin and thetamax
                thetacmsInput = TMath::ACos( (costhetamax - costhetamin )*gRandom->Uniform() + costhetamin )*TMath::RadToDeg(); 
                Double_t* kin2B2 = ATTPC_Background::TwoB(m1, m2, m3, m4, K1, thetacmsInput);
                Double_t phi2 = 2*TMath::Pi() * gRandom->Uniform();         //flat probability in phi
                krec = *(kin2B2+0);
                angrec = *(kin2B2+1);
                Prec = sqrt( pow(krec,2) + 2*krec*m3);
                fPx.at(3) = (Prec*sin(angrec)*cos(phi2) )/1000.0; // To GeV for FairRoot
      		fPy.at(3) = (Prec*sin(angrec)*sin(phi2) )/1000.0; // To GeV for FairRoot
	    	fPz.at(3) = (Prec*cos(angrec) )/1000.0; // To GeV for FairRoot
                //std::cout<<"Kin 2  "<< angrec<<"  "<<krec<<std::endl;
                


                 //proton 3 from breakup
                ////uniform thetacm distribution between thetamin and thetamax
                thetacmsInput = TMath::ACos( (costhetamax - costhetamin )*gRandom->Uniform() + costhetamin )*TMath::RadToDeg(); 
                Double_t* kin2B3 = ATTPC_Background::TwoB(m1, m2, m7, m8, K1, thetacmsInput);
                Double_t phi3 = 2*TMath::Pi() * gRandom->Uniform();         //flat probability in phi
                krec = *(kin2B3+0);
                angrec = *(kin2B3+1);
                Prec = sqrt( pow(0.5*krec,2) + 2*0.5*krec*0.5*m3);
                fPx.at(4) = (Prec*sin(angrec)*cos(phi3) )/1000.0; // To GeV for FairRoot
      		fPy.at(4) = (Prec*sin(angrec)*sin(phi3) )/1000.0; // To GeV for FairRoot
	    	fPz.at(4) = (Prec*cos(angrec) )/1000.0; // To GeV for FairRoot
                //std::cout<<"Kin 3  "<< angrec<<"  "<<krec<<std::endl;




                 //proton 4 from breakup
                ////uniform thetacm distribution between thetamin and thetamax
                thetacmsInput = TMath::ACos( (costhetamax - costhetamin )*gRandom->Uniform() + costhetamin )*TMath::RadToDeg(); 
                Double_t* kin2B4 = ATTPC_Background::TwoB(m1, m2, m7, m8, K1, thetacmsInput);
                Double_t phi4 = 2*TMath::Pi() * gRandom->Uniform();         //flat probability in phi
                krec = *(kin2B4+0);
                angrec = *(kin2B4+1);
                Prec = sqrt( pow(0.5*krec,2) + 2*0.5*krec*0.5*m3);
                fPx.at(5) = (Prec*sin(angrec)*cos(phi4) )/1000.0; // To GeV for FairRoot
      		fPy.at(5) = (Prec*sin(angrec)*sin(phi4) )/1000.0; // To GeV for FairRoot
	    	fPz.at(5) = (Prec*cos(angrec) )/1000.0; // To GeV for FairRoot
                //std::cout<<"Kin 4  "<< angrec<<"  "<<krec<<std::endl;
                

                

                do{
	                 //random_z = 100.0*(gRandom->Uniform()); //cm
                         random_r = 1.0*(gRandom->Gaus(0,1)); //cm
                         random_phi = 2.0*TMath::Pi()*(gRandom->Uniform()); //rad

	        }while(  fabs(random_r) > 4.7 ); //cut at 2 sigma

                

    		for(Int_t i=0; i<fMult; i++){
         	
		     int pdgType = 2212;

		    

                         fVx = random_r*cos(random_phi);
			 fVy = random_r*sin(random_phi);
                         fVz =  100.0*(gRandom->Uniform()); //cm




		      if(i>1 && gATVP->GetDecayEvtCnt() && pdgType==2212 ){
			// TODO: Dirty way to propagate only the products (0 and 1 are beam and target respectively)
			
			 //std::cout << "-I- FairIonGenerator: Generating ions of type "
		       //<< fIon.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
			std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
		       << ") Gev from vertex (" << fVx << ", " << fVy
		       << ", " << fVz << ") cm" << std::endl;


			primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);

			 }
			

			}



  		





		

  		gATVP->IncDecayEvtCnt();  //TODO: Okay someone should put a more suitable name but we are on a hurry...




  return kTRUE;

}


ClassImp(ATTPC_Background)

