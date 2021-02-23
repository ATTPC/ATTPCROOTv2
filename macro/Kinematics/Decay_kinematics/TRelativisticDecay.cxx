/////////////**************//////////////
// Relativistic Kinematics calculator  //
//				19-02-2013			   //
/////////////////////////////////////////

#include <iostream>

#include "TRelativisticDecay.hh"

using namespace std;

TRelativisticDecay::TRelativisticDecay()
{

   // Default constructor -> Masses in MeV (u), incident energy in MeV (lab)

   m1 = 16.; // Mother mass

   m3 = 4.0; // Daughter 1 mass
   m4 = 10.; // Daughter 2 mass

   ex1 = 0.0;

   ex3 = 0.0;
   ex4 = 0.0;

   // tb = 100; // Incident beam energy

   thetacm1Input = 12.3456789; // CM angle of the daughter particle 1 in degrees
   ebInput = 10.0;             // Kin energy of the recoil

   ANGA1 = new double[2];
   ANGA2 = new double[2];

   for (int i = 0; i < 2; i++) {
      ANGA1[i] = 0.;
      ANGA2[i] = 0.;
   }

   bool NoSolution = false;
}

TRelativisticDecay::~TRelativisticDecay()
{
   // Destructor
   delete ANGA1;
   delete ANGA2;
}

void TRelativisticDecay::Kinematics()
{

   const double U = 931.49401;
   const double rad2deg = 0.0174532925;
   const double PI = 3.14159265358979323846;

   double wm1 = m1 * U + ex1; // Rest mass of the mother nucleus M P=(E,0) in CMS

   double wm3 = m3 * U + ex3;
   double wm4 = m4 * U + ex4;

   // cout<<wm1<<" "<<wm3<<" "<<wm4<<endl;

   double E3 = ((wm1 * wm1) + (wm3 * wm3) - (wm4 * wm4)) / (2.0 * wm1); // Energy of daughter 1 in CMS
   double E4 = ((wm1 * wm1) + (wm4 * wm4) - (wm3 * wm3)) / (2.0 * wm1); // Energy of daughter 2 in CMS
   // double P =  1/(2.0*wm1) ; // CM momentum
   // double P2 = (wm1*wm1) - ((wm3-wm4)*(wm3-wm4));
   // double P3 = (wm1*wm1) - ((wm3+wm4)*(wm3+wm4));
   // double Ptot=P*sqrt(P2*P3);
   double Ptot = (1 / (2.0 * wm1)) * sqrt((wm1 * wm1 + wm3 * wm3 - wm4 * wm4) * (wm1 * wm1 + wm3 * wm3 - wm4 * wm4) -
                                          4 * wm1 * wm1 * wm3 * wm3);
   // cout<< Ptot<<endl;

   double eb = ebInput + wm1; // This is the kinetic energy of the recoil nucleus, the total energy includes the mass
   // cout<<eb<<endl;
   double pb = sqrt((eb * eb) - (wm1 * wm1)); // Momentum in the lab system of the recoil nucleus
   double beta = pb / eb;
   double gamma = eb / wm1;
   // double gamma=1.0/sqrt(1.0-beta*beta);
   // cout<<gamma<<endl;

   double thetacm1 = thetacm1Input * rad2deg; // degree to radian
   double thetacm2 = PI - thetacm1;
   // cout<<thetacm1<<endl;

   if (wm1 < wm3 + wm4) {
      cout << "No solution!";
      NoSolution = true;
      return;
   }

   // Lorentz transformations to lab -----

   double p3_cmx = Ptot * sin(thetacm1);
   double p3_cmz = Ptot * cos(thetacm1);
   double p3_labx = p3_cmx;
   double p3_labz = gamma * (p3_cmz + (beta * (E3)));
   double p3_lab = sqrt(p3_labx * p3_labx + p3_labz * p3_labz);
   // cout<<E3-wm3<<" "<<thetacm1<<endl;

   ANGA1[1] = (gamma * (E3 + (beta * p3_cmz))) - wm3;
   // ANGA1[1]=sqrt(p3_lab*p3_lab+wm3*wm3)-wm3;
   // cout<<" Anga1  "<<ANGA1[1]<<endl;

   double p4_cmx = Ptot * sin(thetacm2);
   double p4_cmz = Ptot * cos(thetacm2);
   double p4_labx = p4_cmx;
   double p4_labz = gamma * (p4_cmz + (beta * (E4)));
   double p4_lab = sqrt(p4_labx * p4_labx + p4_labz * p4_labz);

   ANGA2[1] = (gamma * (E4 + (beta * p4_cmz))) - wm4;
   // ANGA2[1] = sqrt(p4_lab*p4_lab+wm4*wm4)-wm4;		// Kinetic energy, T + m = sqrt(p^2 + m^2)
   // cout<<"Decay "<<p4_cmx<<endl;
   //  Check for momentum conservation
   // cout<<p3_labz<<"     "<<p4_labz<<"      "<<pb<<"      "<<thetacm2<<endl;

   double tg_thetalab1 = p3_labx / p3_labz;
   // cout<<thetacm1<<"   "<<tg_thetalab1<<endl;

   if (tg_thetalab1 >= 1.0e6) {
      ANGA1[0] = PI / 2;
   } else {
      ANGA1[0] = atan(tg_thetalab1);
   }

   if (ANGA1[0] < 0.0)
      ANGA1[0] = PI + ANGA1[0];

   double tg_thetalab2 = p4_labx / p4_labz;

   if (tg_thetalab2 > 1.0e6) {
      ANGA2[0] = PI / 2.0;
   } else {
      ANGA2[0] = atan(tg_thetalab2);
   }

   if (ANGA2[0] < 0.0)
      ANGA2[0] = PI + ANGA2[0];

   // cout<<ANGA1[0]<<" "<<ANGA2[0]<<endl;

   //  PrintResults();

   return;
}

void TRelativisticDecay::Dump()
{

   // Dump

   double rad2deg = 0.0174532925;

   cout << "Incident Mass: " << m1 << "  "
        << "Target Mass: " << m2 << endl;
   cout << "Scattered Mass: " << m3 << "  "
        << "Recoil Mass: " << m4 << endl;
   cout << "Theta CM Angle: " << thetacmsInput << "  "
        << "LAB energy :" << tb << endl;
   cout << "Scattered excitation energy: " << ex3 << endl;

   cout << "Lab Scattering angle:" << ANGAs[0] / rad2deg << ", Scattering energy=" << ANGAs[1] << endl;
   cout << "      Lab Recoil angle:" << ANGAr[0] / rad2deg << ", Recoil energy=" << ANGAr[1] << endl;
}

void TRelativisticDecay::PrintResults()
{

   // print the results for each solution

   double rad2deg = 0.0174532925;

   int prec = cout.precision(6);

   cout << endl << " PrintResult()" << endl;

   cout << "Incident Mass: " << m1 << "  "
        << "Target Mass: " << m2 << endl;
   cout << "Scattered Mass: " << m3 << "  "
        << "Recoil Mass: " << m4 << endl;
   cout << "Theta CM Angle: " << thetacmsInput << "  "
        << "LAB energy :" << tb << endl;
   cout << "Scattered excitation energy: " << ex3 << endl;

   cout << "Lab Scattering angle:" << ANGAs[0] / rad2deg << ", Scattering energy=" << ANGAs[1] << endl;
   cout << "      Lab Recoil angle:" << ANGAr[0] / rad2deg << ", Recoil energy=" << ANGAr[1] << endl;

   cout.precision(prec);
}
