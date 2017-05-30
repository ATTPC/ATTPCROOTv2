#include "ATd2HeAnalysis.hh"
#include "TMath.h"
#include <memory>


ClassImp(ATd2HeAnalysis)

ATd2HeAnalysis::ATd2HeAnalysis()
{
      
      
      THcm = 0.0;
      MissingMass = 0.0;
      theta_fit = 0.0;
      phi_fit = 0.0;
      ok = false;
      vertex = 0.0;
}

ATd2HeAnalysis::~ATd2HeAnalysis()
{
}

void ATd2HeAnalysis::SetThetaCM(Double_t value)    { THcm=value;}
void ATd2HeAnalysis::SetMissingMass(Double_t  value) { MissingMass=value;}

Double_t  ATd2HeAnalysis::GetThetaCM()    { return THcm;}
Double_t  ATd2HeAnalysis::GetMissingMass() { return MissingMass;}
Double_t  ATd2HeAnalysis::GetThetaFit() { return theta_fit;}
Double_t  ATd2HeAnalysis::GetPhiFit() { return phi_fit;}
bool         ATd2HeAnalysis::GetFitStatus() { return ok;}
Double_t  ATd2HeAnalysis::GetVertexFit() { return vertex;}

Double_t ATd2HeAnalysis::omega(Double_t x, Double_t  y, Double_t z) { return sqrt(x*x + y*y + z*z -2*x*y -2*y*z -2*x*z);}


Double_t ATd2HeAnalysis::EnergyFluctuation(Double_t energypoint){

        TRandom3* gRandome = new TRandom3();
        Double_t fanofactor = 0.2;
	Double_t ion_pot = 40e-6; //in MeV
	Int_t  Nelectrons = 0;
        Double_t eout ;
        
        Nelectrons = TMath::Floor(energypoint/ion_pot);
	Nelectrons = gRandome -> Gaus(Nelectrons, sqrt(Nelectrons*fanofactor));
	eout = Nelectrons*ion_pot;
        delete  gRandome;

        return eout;


}


void ATd2HeAnalysis::kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t  thetalab, Double_t  K_eject){

 //in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
        double Et1 = K_proj + m1;
        double Et2 = m2;
        double Et3 = K_eject + m3;
        double Et4  = Et1 + Et2 - Et3;
        double m4_ex, Ex, theta_cm;
        double s,t,u; //---Mandelstam variables
        
        

        s = pow(m1,2) + pow(m2,2) +2*m2*Et1;
        u = pow(m2,2) + pow(m3,2) - 2*m2*Et3;

        m4_ex = sqrt(  (cos(thetalab) * ATd2HeAnalysis::omega(s,pow(m1,2),pow(m2,2)) * ATd2HeAnalysis::omega(u,pow(m2,2),pow(m3,2)) - (s - pow(m1,2) - pow(m2,2))*(pow(m2,2) + pow(m3,2) - u) )/(2*pow(m2,2)) + s + u - pow(m2,2)  );
        Ex = m4_ex - m4;
        
        t =   pow(m2,2) + pow(m4_ex,2) - 2*m2*Et4; 

        
        //for inverse kinematics Note: this angle corresponds to the recoil
        theta_cm = TMath::Pi() - acos( ( pow(s,2) +s*(2*t - pow(m1,2) - pow(m2,2) - pow(m3,2) - pow(m4_ex,2)) + (pow(m1,2) - pow(m2,2))*(pow(m3,2) - pow(m4_ex,2)) )/( ATd2HeAnalysis::omega(s,pow(m1,2),pow(m2,2))*ATd2HeAnalysis::omega(s,pow(m3,2),pow(m4_ex,2))) ) ;

      


	 THcm = theta_cm*TMath::RadToDeg();
         MissingMass =  Ex;


}


void ATd2HeAnalysis::FitTrack(TGraph2D *f2dtrack, TGraph *fpadtrack, std::vector<Double_t> *iniguess ){



	           ROOT::Fit::Fitter  fitter;
   		// make the functor objet
   		   SumDistance2 sdist(f2dtrack);

       	           ROOT::Math::Functor fcn(sdist,4);
   		   // set the function and the initial parameter values
   		   

		   
	           Double_t pStart[4] = {iniguess->at(0),iniguess->at(1),iniguess->at(2),iniguess->at(3)};
   		   fitter.SetFCN(fcn,pStart);
		   
		   fitter.Config().ParSettings(0).SetLimits(-10.0,10.0);
		   fitter.Config().ParSettings(1).SetLimits(-20,20);
		   fitter.Config().ParSettings(2).SetLimits(-10,1000);
		   fitter.Config().ParSettings(3).SetLimits(-20,20);
		   
   		   // set step sizes different than default ones (0.3 times parameter values)
   		   for (int i = 0; i < 4; ++i) fitter.Config().ParSettings(i).SetStepSize(0.01);
   		   ok = fitter.FitFCN();
		   
   		   if (!ok) {
      		        Error("line3Dfit","Line3D Fit failed");
   		   }
   		   const ROOT::Fit::FitResult & result = fitter.Result();
   		   
		   // get fit parameters
   		   const double * parFit = result.GetParams();  

                    vertex = parFit[2];
                   
                   theta_fit = fabs(atan(sqrt(parFit[1]*parFit[1] +1)/parFit[3] )*TMath::RadToDeg());
                   if(vertex>iniguess->at(5)) theta_fit = 180.0- theta_fit;
                   

                   TFitResultPtr  res1 = fpadtrack->Fit("pol1","Sq"); 
                   phi_fit = atan2(signo(iniguess->at(4))*signo(res1->Value(1))*fabs(res1->Value(1)),signo(iniguess->at(4)))*TMath::RadToDeg();
                   if(phi_fit<0) phi_fit+=360;
                   

}





