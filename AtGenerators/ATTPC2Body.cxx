#include "ATTPC2Body.h"

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

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"
#define cBLUE "\033[1;34m"

Int_t ATTPC2Body::fgNIon = 0;


ATTPC2Body::ATTPC2Body()
  : fMult(0),
    fPx(0.), fPy(0.), fPz(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(0),  fQ(0)
{
//  cout << "-W- ATTPCIonGenerator: "
//      << " Please do not use the default constructor! " << endl;
}

// -----   Default constructor   ------------------------------------------
ATTPC2Body::ATTPC2Body(const char* name,std::vector<Int_t> *z,std::vector<Int_t> *a,std::vector<Int_t> *q, Int_t mult, std::vector<Double_t> *px,
	std::vector<Double_t>* py,std::vector<Double_t> *pz, std::vector<Double_t> *mass, std::vector<Double_t> *Ex,Double_t ResEner,Double_t MinCMSAng,Double_t MaxCMSAng)
  : fMult(0),
    fPx(0.), fPy(0.), fPz(0.),
    fVx(0.), fVy(0.), fVz(0.),
    fIon(0),fPType(0.),fQ(0)
{


  fgNIon++;
  fMult = mult;
  fIon.reserve(fMult);
  fThetaCmsMin = MinCMSAng;
  fThetaCmsMax = MaxCMSAng;


  fNoSolution = kFALSE;

  char buffer[20];
  TDatabasePDG* pdgDB = TDatabasePDG::Instance();
  TParticlePDG* kProtonPDG = pdgDB->GetParticle(2212);
  TParticle* kProton = new TParticle();
  kProton->SetPdgCode(2212);

  TParticle* kNeutron = new TParticle();
  kNeutron->SetPdgCode(2112);

  fIsFixedTargetPos = kFALSE;
  fIsFixedMomentum  = kFALSE;
  fPxBeam_buff      = 0.0;
  fPyBeam_buff      = 0.0;
  fPzBeam_buff      = 0.0;

  fBeamEnergy_buff = ResEner;
  
 


      for(Int_t i=0;i<fMult;i++){


       	fPx.push_back( Double_t(a->at(i)) * px->at(i) );
	fPy.push_back( Double_t(a->at(i)) * py->at(i) );
	fPz.push_back( Double_t(a->at(i)) * pz->at(i) );
	Masses.push_back(mass->at(i)*1000.0);
        fExEnergy.push_back(Ex->at(i));
        fWm.push_back( mass->at(i)*1000.0*0.93149401 + Ex->at(i));
        FairIon *IonBuff;
        FairParticle *ParticleBuff;
        sprintf(buffer, "Product_Ion%d", i);

        if( a->at(i)!=1  ){

          IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i),0.0,mass->at(i));
          ParticleBuff = new FairParticle("dummyPart",1,1,1.0,0,0.0,0.0);
          fPType.push_back("Ion");
          std::cout<<" Adding : "<<buffer<<std::endl;

        }else if( a->at(i)==1 && z->at(i)==1  ){

          IonBuff = new FairIon("dummyIon",50,50,0,0.0,100); // We fill the std::vector with a dummy ion
          ParticleBuff = new FairParticle(2212,kProton);
          fPType.push_back("Proton");
	      
        }else if( a->at(i)==1 && z->at(i)==0  ){

          IonBuff = new FairIon("dummyIon",50,50,0,0.0,100); // We fill the std::vector with a dummy ion
          ParticleBuff = new FairParticle(2112,kNeutron);
          fPType.push_back("Neutron");
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
      		             //run->AddNewParticle(fParticle.at(i));
                       std::cout<<" fIon name :"<<fIon.at(i)->GetName()<<std::endl;
                       std::cout<<" fParticle name :"<<fParticle.at(i)->GetName()<<std::endl;
                       std::cout<<fParticle.at(i)->GetName()<<std::endl;

      	  }else if(fPType.at(i)=="Neutron"){

                      std::cout<<" In position "<<i<<" adding an : "<<fPType.at(i)<<std::endl;
                       //run->AddNewParticle(fParticle.at(i));
                       std::cout<<" fIon name :"<<fIon.at(i)->GetName()<<std::endl;
                       std::cout<<" fParticle name :"<<fParticle.at(i)->GetName()<<std::endl;
                       std::cout<<fParticle.at(i)->GetName()<<std::endl;

          }  

    }



}

// -----   Destructor   ---------------------------------------------------
ATTPC2Body::~ATTPC2Body()
{
 // if (fIon) delete fIon;
}

void ATTPC2Body::SetFixedTargetPosition(double vx, double vy, double vz)
{
   std::cout<<" -I- ATTPC2Body : Fixed target position at "<<vx<<"	"<<vy<<"	"<<vz<<std::endl;
   fIsFixedTargetPos = kTRUE;
   fVx = vx;
   fVy = vy;
   fVz = vz;
}

void ATTPC2Body::SetFixedBeamMomentum(double px, double py, double pz)
{
  std::cout<<" -I- ATTPC2Body : Fixed beam momentum "<<px<<"	"<<py<<"	"<<pz<<std::endl;
  fIsFixedMomentum = kTRUE;
  fPxBeam_buff = px;
  fPyBeam_buff = py;
  fPzBeam_buff = pz;

}

// -----   Public method ReadEvent   --------------------------------------
Bool_t ATTPC2Body::ReadEvent(FairPrimaryGenerator* primGen) {



    std::vector<Double_t> Ang;				     // Lab Angle of the products
    std::vector<Double_t> Ene;                                // Lab Energy of the products
    Ang.reserve(2);
    Ang.reserve(2);
    fPx.clear();
    fPy.clear();
    fPx.clear();

    fPx.resize(fMult);
    fPy.resize(fMult);
    fPx.resize(fMult);


   Double_t costhetamin = TMath::Cos(fThetaCmsMin*TMath::DegToRad());
   Double_t costhetamax = TMath::Cos(fThetaCmsMax*TMath::DegToRad());
   //Double_t thetacmsInput = fThetaCmsMin + ((fThetaCmsMax-fThetaCmsMin)*gRandom->Uniform());
   ////uniform thetacm distribution between thetamin and thetamax
   Double_t thetacmsInput = TMath::ACos( (costhetamax - costhetamin )*gRandom->Uniform() + costhetamin )*TMath::RadToDeg();

   std::cout<<cBLUE<<" -I- ATTPC2Body : Random CMS Theta angle in degrees : "<<thetacmsInput<<cNORMAL<<std::endl;
   const Double_t rad2deg = 0.0174532925;

   AtStack* stack = (AtStack*) gMC->GetStack();

   fIsDecay = kFALSE;

    if(fIsFixedTargetPos){
          fBeamEnergy = fBeamEnergy_buff;
          gATVP->SetValidKine(kTRUE);
          std::cout<<cBLUE<<" -I- ATTPC2Body Beam energy (Fixed Target mode) : "<<fBeamEnergy<<cNORMAL<<std::endl;

    }else{
	   fBeamEnergy = gATVP->GetEnergy();
           
	   std::cout<<cBLUE<<" -I- ATTPC2Body Residual energy (Active Target mode) : "<<gATVP->GetEnergy()<<cNORMAL<<std::endl;
    }


   if(fBeamEnergy>0 && (gATVP->GetDecayEvtCnt()%2!=0 || fIsFixedTargetPos)){ //Requires a non zero vertex energy and pre-generated Beam event (not punch thorugh)


     if(fIsFixedTargetPos){
           fPxBeam = fPxBeam_buff; 
           fPyBeam = fPyBeam_buff;
           fPzBeam = fPzBeam_buff;
     
     }else{
  
          
           fPxBeam = gATVP->GetPx(); 
           fPyBeam = gATVP->GetPy();
           fPzBeam = gATVP->GetPz();
     }	         

           Double_t eb=fBeamEnergy+fWm.at(0);
           Double_t pb2=fBeamEnergy*fBeamEnergy+2.0*fBeamEnergy*fWm.at(0);
           Double_t pb=TMath::Sqrt(pb2);
           Double_t beta=pb/(eb+fWm.at(1));
           Double_t gamma=1.0/sqrt(1.0-beta*beta);



           Double_t thetacms=thetacmsInput*rad2deg;  // degree to radian


           Double_t thetacmr = TMath::Pi()-thetacms;
           Double_t e        = fBeamEnergy+fWm.at(0)+fWm.at(1);
           Double_t e_cm2    = e*e-pb2;
           Double_t e_cm     = TMath::Sqrt(e_cm2);
           Double_t t_cm     = e_cm-fWm.at(2)-fWm.at(3);


              if(t_cm<0.0){
                std::cout << "-I- ATTPC2Body : No solution!"<<std::endl;
                fNoSolution=kTRUE;
                gATVP->SetValidKine(kFALSE);
               // return kFALSE;

              }


              if(gATVP->GetValidKine()){

                       Double_t t_cm2=t_cm*t_cm;
                       Double_t t3_cm=(t_cm2+2.*fWm.at(3)*t_cm)/(t_cm+fWm.at(2)+fWm.at(3))/2.0;
                       Double_t t4_cm=(t_cm2+2.*fWm.at(2)*t_cm)/(t_cm+fWm.at(2)+fWm.at(3))/2.0;
                       Double_t p3_cm2=t3_cm*t3_cm+2.0*t3_cm*fWm.at(2);
                       Double_t p3_cm =TMath::Sqrt(p3_cm2);
                       Double_t tg_thetalabs=p3_cm*TMath::Sin(thetacms)/(gamma*(p3_cm*TMath::Cos(thetacms)+beta*TMath::Sqrt(p3_cm*p3_cm+fWm.at(2)*fWm.at(2))));

                      if(tg_thetalabs>=1.0e6){
                        Ang.push_back(TMath::Pi()/2.0);
                      }
                      else{
                        Ang.push_back(TMath::ATan(tg_thetalabs));
                      }

                      if(Ang.at(0)<0.0) Ang.at(0)=TMath::Pi()+Ang.at(0);

                      Double_t p4_cm2=t4_cm*t4_cm+2.*t4_cm*fWm.at(3);
                      Double_t p4_cm =TMath::Sqrt(p4_cm2);
                      Double_t tg_thetalabr=p4_cm*TMath::Sin(thetacmr)/(gamma*(p4_cm*TMath::Cos(thetacmr)+beta*TMath::Sqrt(p4_cm*p4_cm+fWm.at(3)*fWm.at(3))));
                      //std::cout<<" tg_thetalabr : "<<tg_thetalabr<<std::endl;

                      if(tg_thetalabr>1.0e6){
                        Ang.push_back(TMath::Pi()/2.0);
                      }
                      else{
                        Ang.push_back(TMath::ATan(tg_thetalabr));
                      }

                      if(Ang.at(1)<0.0) Ang.at(1)=TMath::Pi()+Ang.at(1);


                    // Lorentz transformations to lab -----

                      Double_t p3_cmx = p3_cm*sin(thetacms);
                      Double_t p3_cmz = p3_cm*cos(thetacms);
                      Double_t p3_labx = p3_cmx;
                      Double_t p3_labz = gamma*(p3_cmz+beta*(t3_cm+fWm.at(2)));
                      Double_t p3_lab = TMath::Sqrt(p3_labx*p3_labx+p3_labz*p3_labz);
                      Ene.push_back(TMath::Sqrt(p3_lab*p3_lab+fWm.at(2)*fWm.at(2))-fWm.at(2));

                      Double_t p4_cmx = p4_cm*sin(thetacmr);
                      Double_t p4_cmz = p4_cm*cos(thetacmr);
                      Double_t p4_labx = p4_cmx;
                      Double_t p4_labz = gamma*(p4_cmz+beta*(t4_cm+fWm.at(3)));
                      Double_t p4_lab = TMath::Sqrt(p4_labx*p4_labx+p4_labz*p4_labz);
                      Ene.push_back(TMath::Sqrt(p4_lab*p4_lab+fWm.at(3)*fWm.at(3))-fWm.at(3));

                        if(!gATVP->GetValidKine()){
                         std::cout << " -I- ===== ATTPC2Body - Kinematics ====== "<<std::endl;
                         std::cout << " -I- ===== No Valid Solution on Beam Event for these kinematics (probably due to threshold energy) ====== "<<std::endl;
                         std::cout << " -I- ===== Setting Values to 0  ====== "<<std::endl;
                         Ene.at(0)=0.0;
                         Ang.at(0)=0.0;
                         Ene.at(1)=0.0;
                         Ang.at(1)=0.0;

                        }else{

                          
			  std::cout <<cBLUE<< " -I- ===== ATTPC2Body - Kinematics ====== "<<std::endl;
                          std::cout << " Scattered energy:" << Ene.at(0)  << " MeV" << std::endl;
                          std::cout << " Scattered  angle:"  << Ang.at(0)*180/TMath::Pi() << " deg" << std::endl;
                          std::cout << " Recoil energy:" << Ene.at(1) << " MeV" << std::endl;
                          std::cout << " Recoiled angle:"  << Ang.at(1)*180.0/TMath::Pi() << " deg" <<cNORMAL<< std::endl;
                      }




                      gATVP->SetRecoilE(Ene.at(1));
                      gATVP->SetRecoilA(Ang.at(1)*180.0/TMath::Pi());
                      gATVP->SetScatterE(Ene.at(0));
                      gATVP->SetScatterA(Ang.at(0)*180.0/TMath::Pi());


                    	  fPx.at(0) = 0.0;
                          fPy.at(0) = 0.0;
                    	  fPz.at(0) = 0.0;

                          fPx.at(1) = 0.0;
                          fPy.at(1) = 0.0;
                    	  fPz.at(1) = 0.0;



                    	  fPx.at(2) = p3_labx/1000.0; // To GeV for FairRoot
                          fPy.at(2) = 0.0;
                    	  fPz.at(2) = p3_labz/1000.0;

                          fPx.at(3) = p4_labx/1000.0;
                          fPy.at(3) = 0.0;
                    	  fPz.at(3) = p4_labz/1000.0;


                           Double_t phiBeam1=0., phiBeam2=0.;

                           phiBeam1 = 2*TMath::Pi() * gRandom->Uniform();         //flat probability in phi
                           phiBeam2 = phiBeam1 + TMath::Pi();

                           //std::cout<<" Propagated Entrance Position 2 - X : "<<gATVP->GetVx()<<" - Y : "<<gATVP->GetVy()<<" - Z : "<<gATVP->GetVz()<<std::endl;
                           //std::cout<<" Propagated Stop Position - X : "<<gATVP->GetInVx()<<" - Y : "<<gATVP->GetInVy()<<" - Z : "<<gATVP->GetInVz()<<std::endl;

                          /* TVector3 BeamPos( gATVP->GetVx() - gATVP->GetInVx() ,  gATVP->GetVy() - gATVP->GetInVy() ,  gATVP->GetVz() - gATVP->GetInVz() );
                           std::cout << " Beam Theta (Pos) : "<<BeamPos.Theta()*180.0/TMath::Pi()<<std::endl;
                           std::cout << " Beam Phi  (Pos) : "<<BeamPos.Phi()*180.0/TMath::Pi()<<std::endl;*/

                           TVector3 BeamPos(fPxBeam*1000,fPyBeam*1000,fPzBeam*1000); //To MeV for Euler Transformation
                           //TVector3 BeamPos(1.0,1.0,0.0);
                           LOG(DEBUG) << " Beam Theta (Mom) : "<<BeamPos.Theta()*180.0/TMath::Pi()<<FairLogger::endl;
                           LOG(DEBUG) << " Beam Phi (Mom) : "<<BeamPos.Phi()*180.0/TMath::Pi()<<FairLogger::endl;


                           Double_t thetaLab1, phiLab1, thetaLab2, phiLab2;
                           ATEulerTransformation* EulerTransformer = new ATEulerTransformation();
                           EulerTransformer->SetBeamDirectionAtVertexTheta(BeamPos.Theta());
                           EulerTransformer->SetBeamDirectionAtVertexPhi(BeamPos.Phi());


                           EulerTransformer->SetThetaInBeamSystem(Ang.at(0));
                           EulerTransformer->SetPhiInBeamSystem(phiBeam1);
                           EulerTransformer->DoTheEulerTransformationBeam2Lab();   // Euler transformation for particle 1
                          //  EulerTransformer->PrintResults();

                           thetaLab1 = EulerTransformer->GetThetaInLabSystem();
                           phiLab1   = EulerTransformer->GetPhiInLabSystem();
                           LOG(DEBUG) << " Scattered  angle Phi :"  << phiBeam1*180.0/TMath::Pi()  << " deg" << FairLogger::endl;
                           LOG(DEBUG) << " Scattered  angle Theta (Euler) :"  << thetaLab1*180.0/TMath::Pi()  << " deg" << FairLogger::endl;
                           LOG(DEBUG) << " Scattered  angle Phi (Euler) :"  << phiLab1*180.0/TMath::Pi()  << " deg" << FairLogger::endl;


                           /*TVector3 direction1 = TVector3(sin(thetaLab1)*cos(phiLab1),
                                                                 sin(thetaLab1)*sin(phiLab1),
                                                                 cos(thetaLab1));*/

                          TVector3 direction1 = TVector3(sin(thetaLab1)*cos(phiLab1),
                                                                 sin(thetaLab1)*sin(phiLab1),
                                                                 cos(thetaLab1));


                           EulerTransformer->SetThetaInBeamSystem(Ang.at(1));
                           EulerTransformer->SetPhiInBeamSystem(phiBeam2);
                           EulerTransformer->DoTheEulerTransformationBeam2Lab();   // Euler transformation for particle 2
                           //EulerTransformer->PrintResults();

                           thetaLab2 = EulerTransformer->GetThetaInLabSystem();
                           phiLab2   = EulerTransformer->GetPhiInLabSystem();

                           TVector3 direction2 = TVector3(sin(thetaLab2)*cos(phiLab2),
                    					     sin(thetaLab2)*sin(phiLab2),
                    					     cos(thetaLab2));

                           LOG(DEBUG) << " Recoiled  angle Phi :"  << phiBeam2*180.0/TMath::Pi()  << " deg" << FairLogger::endl;
                           LOG(DEBUG) << " Recoiled  angle Theta (Euler) :"  << thetaLab2*180.0/TMath::Pi()  << " deg" << FairLogger::endl;
                           LOG(DEBUG) << " Recoiled  angle Phi (Euler) :"  << phiLab2*180.0/TMath::Pi()  << " deg" << FairLogger::endl;

                           LOG(DEBUG) << "  Phi Diference :"  << (phiBeam1*180.0/TMath::Pi()) -  (phiBeam2*180.0/TMath::Pi())  << " deg" << FairLogger::endl;
                           LOG(DEBUG) << "  Phi Diference (Euler) :"  << (phiLab1*180.0/TMath::Pi()) -  (phiLab2*180.0/TMath::Pi())  << " deg" << FairLogger::endl;

                           delete EulerTransformer;

                            LOG(DEBUG)<<" Direction 1 Theta : "<<direction1.Theta()*180.0/TMath::Pi()<<FairLogger::endl;
                            LOG(DEBUG)<<" Direction 1 Phi : "<<direction1.Phi()*180.0/TMath::Pi()<<FairLogger::endl;
                            LOG(DEBUG)<<" Direction 2 Theta : "<<direction2.Theta()*180.0/TMath::Pi()<<FairLogger::endl;
                            LOG(DEBUG)<<" Direction 2 Phi : "<<direction2.Phi()*180.0/TMath::Pi()<<FairLogger::endl;

                            fPx.at(2) = p3_lab*direction1.X()/1000.0; // To GeV for FairRoot
                            fPy.at(2) = p3_lab*direction1.Y()/1000.0;
                            fPz.at(2) = p3_lab*direction1.Z()/1000.0;

                            fPx.at(3) = p4_lab*direction2.X()/1000.0;
                            fPy.at(3) = p4_lab*direction2.Y()/1000.0;
                            fPz.at(3) = p4_lab*direction2.Z()/1000.0;

              }else{

                   fPx.at(2) = 0.0; // To GeV for FairRoot
                   fPy.at(2) = 0.0;
                   fPz.at(2) = 0.0;

                   fPx.at(3) = 0.0;
                   fPy.at(3) = 0.0;
                   fPz.at(3) = 0.0;



              }



                         /* if(!gATVP->GetValidKine()){

                            fPx.at(2) = 0.0; // To GeV for FairRoot
                            fPy.at(2) = 0.0;
                          	fPz.at(2) = 0.0;

                            fPx.at(3) = 0.0;
                            fPy.at(3) = 0.0;
                    	      fPz.at(3) = 0.0;

                          }else{

                            fPx.at(2) = p3_lab*direction1.X()/1000.0; // To GeV for FairRoot
                            fPy.at(2) = p3_lab*direction1.Y()/1000.0;
                          	fPz.at(2) = p3_lab*direction1.Z()/1000.0;

                            fPx.at(3) = p4_lab*direction2.X()/1000.0;
                            fPy.at(3) = p4_lab*direction2.Y()/1000.0;
                    	      fPz.at(3) = p4_lab*direction2.Z()/1000.0;

                          }*/



                             // Debugging purposes

                                  /*TVector3 debug(fPx.at(2),fPy.at(2),fPz.at(2));
                                  std::cout<<" p3_lab         : "<<p3_lab<<std::endl;
                                  std::cout<<" Debug momentum : "<<debug.Mag()<<std::endl;
                                  std::cout<<" Debug Phi : "<<debug.Phi()*180.0/TMath::Pi()<<std::endl;
                                  std::cout<<" Debug Theta : "<<debug.Theta()*180.0/TMath::Pi()<<std::endl;
                                  TVector3 debug2(fPx.at(3),fPy.at(3),fPz.at(3));
                                  std::cout<<" p4_lab         : "<<p4_lab<<std::endl;
                                  std::cout<<" Debug momentum 2 : "<<debug2.Mag()<<std::endl;
                                  std::cout<<" Debug Phi 2 : "<<debug2.Phi()*180.0/TMath::Pi()<<std::endl;
                                  std::cout<<" Debug Theta 2 : "<<debug2.Theta()*180.0/TMath::Pi()<<std::endl;*/


                          // Particle transport begins here
		

                          for(Int_t i=0; i<fMult; i++){

                                 TParticlePDG* thisPart;

                              	 if(fPType.at(i)=="Ion") thisPart = TDatabasePDG::Instance()->GetParticle(fIon.at(i)->GetName());
                                 else if(fPType.at(i)=="Proton")  thisPart = TDatabasePDG::Instance()->GetParticle(fParticle.at(i)->GetName());
                                 else if(fPType.at(i)=="Neutron") thisPart = TDatabasePDG::Instance()->GetParticle(fParticle.at(i)->GetName());


                                 if ( ! thisPart ) {

                              		    if(fPType.at(i)=="Ion") std::cout << "-W- FairIonGenerator: Ion " << fIon.at(i)->GetName()<< " not found in database!" << std::endl;
                                      else if(fPType.at(i)=="Proton")  std::cout << "-W- FairIonGenerator: Particle " << fParticle.at(i)->GetName()<< " not found in database!" << std::endl;
                                      else if(fPType.at(i)=="Neutron") std::cout << "-W- FairIonGenerator: Particle " << fParticle.at(i)->GetName()<< " not found in database!" << std::endl;

                              		   return kFALSE;
                              	   }

                              	
                                 int pdgType = thisPart->PdgCode();

                              	 // Propagate the vertex of the previous event
				if(fIsFixedTargetPos){
				 
				 //fVx = 0.0;
                              	 //fVy = 0.0;
                              	 //fVz = 0.0;
						
				}else{
                              	 fVx = gATVP->GetVx();
                              	 fVy = gATVP->GetVy();
                              	 fVz = gATVP->GetVz();
				 
				}

				//For decay generators
				TVector3 ScatP(fPx.at(2),fPy.at(2),fPz.at(2));
				gATVP->SetScatterP(ScatP);
    				gATVP->SetScatterEx(fExEnergy.at(2));


                      		      if(i>1 && (gATVP->GetDecayEvtCnt() || fIsFixedTargetPos)  && pdgType!=1000500500 && fPType.at(i)=="Ion" ){// TODO: Dirty way to propagate only the products (0 and 1 are beam and target respectively)


                                  			 std::cout <<cBLUE<< "-I- FairIonGenerator: Generating ions of type "
                                  		   << fIon.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
                                  			 std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
                                  		   << ") Gev from vertex (" << fVx << ", " << fVy
                                  		   << ", " << fVz << ") cm" << std::endl;
                                  			 primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);

                      		       }else if(i>1 && (gATVP->GetDecayEvtCnt() || fIsFixedTargetPos) && pdgType==2212 && fPType.at(i)=="Proton" ){

                                  			std::cout << "-I- FairIonGenerator: Generating ions of type "
                                  		  << fParticle.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
                                  			std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
                                  		  << ") Gev from vertex (" << fVx << ", " << fVy
                                  		  << ", " << fVz << ") cm" << std::endl;
                                  			primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);

                      		      }else if(i>1 && (gATVP->GetDecayEvtCnt() || fIsFixedTargetPos) && pdgType==2112 && fPType.at(i)=="Neutron" ){

                                        std::cout << "-I- FairIonGenerator: Generating ions of type "
                                        << fParticle.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
                                        std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
                                        << ") Gev from vertex (" << fVx << ", " << fVy
                                        << ", " << fVz << ") cm" <<cNORMAL<< std::endl;
                                        primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);

                                      }

                            		    


                          }

				

        }//if residual energy > 0

                      
                       gATVP->IncDecayEvtCnt();  

                       return kTRUE;

}


ClassImp(ATTPC2Body)
