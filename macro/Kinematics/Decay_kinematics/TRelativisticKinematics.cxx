/////////////**************//////////////
// Relativistic Kinematics calculator  //
//				19-02-2013			   //
/////////////////////////////////////////

#include "TRelativisticKinematics.hh"

#include <iostream>

using namespace std;

TRelativisticKinematics::TRelativisticKinematics()
{

   // Default constructor -> Masses in MeV (u), incident energy in MeV (lab)

   m1 = 16.;  // Projectile mass
   m2 = 4.;   // Target mass
   m3 = 16.0; // Scattered particle mass
   m4 = 4.;   // Recoil mass

   ex1 = 0.0;
   ex2 = 0.0;
   ex3 = 0.0;
   ex4 = 0.0;

   tb = 100; // Incident energy

   thetacmsInput = 5.00; // CM angle of the scattered particle in degrees

   ANGAs = new double[2];
   ANGAr = new double[2];

   for (int i = 0; i < 2; i++) {
      ANGAs[i] = 0.;
      ANGAr[i] = 0.;
   }

   bool NoSolution = false;
}

TRelativisticKinematics::~TRelativisticKinematics()
{
   // Destructor
   delete ANGAs;
   delete ANGAr;
}

void TRelativisticKinematics::Kinematics()
{

   const double U = 931.49401;
   const double rad2deg = 0.0174532925;
   const double PI = 3.14159265358979323846;

   double wm1 = m1 * U + ex1;
   double wm2 = m2 * U + ex2;
   double wm3 = m3 * U + ex3;
   double wm4 = m4 * U + ex4;

   double eb = tb + wm1; // Total energy of beam LAB system
   // cout<<tb<<"	"<<m1<<"	"<<wm1<<"	"<<eb<<endl;
   double pb2 =
      tb * tb +
      2.0 * tb *
         wm1; // Momentum of the beam as a function of kinetic energy of the beam Tb = sqrt(p^2c^2+m^2c^4) - mc^2
   double pb = sqrt(pb2);
   double beta = pb / (eb + wm2);                // Beta
   double gamma = 1.0 / sqrt(1.0 - beta * beta); // Gamma
   // cout<<gamma<<endl;

   double thetacms = thetacmsInput * rad2deg; // degree to radian

   double thetacmr = PI - thetacms;
   double e = tb + wm1 + wm2;  // Total energy of the system
   double e_cm2 = e * e - pb2; // Total energy of CM is just the mass (Lorentz Invariant M^2=(E1+E2)-(p1+p2)), therefore
                               // we subtract the momentum to the total energy P=(M,0)
   double e_cm = sqrt(e_cm2);
   double t_cm = e_cm - wm3 - wm4; // Kinetic energy of CM subtract the masses
                                   // cout<<e<<endl;

   std::cout << " Beam energy in lab (e) : " << eb << " Kinetic energy (tb) : " << tb << " Mass (wm1) : " << wm1
             << " momemtum (pb) : " << pb << "\n";

   if (t_cm < 0.0) {
      cout << "No solution!";
      NoSolution = true;
      return;
   }

   double t_cm2 = t_cm * t_cm;
   double t3_cm = (t_cm2 + 2. * wm4 * t_cm) / (t_cm + wm3 + wm4) / 2.0; // Energy of particle 3 E3 = 2*p3/sqrt(M)
   double t4_cm = (t_cm2 + 2. * wm3 * t_cm) / (t_cm + wm3 + wm4) / 2.0;
   double p3_cm2 = t3_cm * t3_cm + 2.0 * t3_cm * wm3;
   double p3_cm = sqrt(p3_cm2);
   double tg_thetalabs =
      p3_cm * sin(thetacms) / (gamma * (p3_cm * cos(thetacms) + beta * sqrt(p3_cm * p3_cm + wm3 * wm3)));

   if (tg_thetalabs >= 1.0e6) {
      ANGAs[0] = PI / 2;
   } else {
      ANGAs[0] = atan(tg_thetalabs);
   }

   if (ANGAs[0] < 0.0)
      ANGAs[0] = PI + ANGAs[0];

   double p4_cm2 = t4_cm * t4_cm + 2. * t4_cm * wm4;
   double p4_cm = sqrt(p4_cm2);
   double tg_thetalabr =
      p4_cm * sin(thetacmr) / (gamma * (p4_cm * cos(thetacmr) + beta * sqrt(p4_cm * p4_cm + wm4 * wm4)));

   if (tg_thetalabr > 1.0e6) {
      ANGAr[0] = PI / 2.0;
   } else {
      ANGAr[0] = atan(tg_thetalabr);
   }

   if (ANGAr[0] < 0.0)
      ANGAr[0] = PI + ANGAr[0];

   // Lorentz transformations to lab -----

   double p3_cmx = p3_cm * sin(thetacms);
   double p3_cmz = p3_cm * cos(thetacms);
   double p3_labx = p3_cmx;
   double p3_labz = gamma * (p3_cmz + beta * (t3_cm + wm3));
   double p3_lab = sqrt(p3_labx * p3_labx + p3_labz * p3_labz);
   // cout<<p3_cm<<" "<<thetacms<<endl;

   ANGAs[1] = sqrt(p3_lab * p3_lab + wm3 * wm3) - wm3;
   // cout<<" Anga1  "<<ANGAs[1]<<endl;

   double p4_cmx = p4_cm * sin(thetacmr);
   double p4_cmz = p4_cm * cos(thetacmr);
   double p4_labx = p4_cmx;
   double p4_labz = gamma * (p4_cmz + beta * (t4_cm + wm4));
   double p4_lab = sqrt(p4_labx * p4_labx + p4_labz * p4_labz);
   ANGAr[1] = sqrt(p4_lab * p4_lab + wm4 * wm4) - wm4;
   // cout<<"Kinematics "<<t4_cm+wm4<<endl;

   //  PrintResults();

   // cout<<thetacmr<<"   "<<tg_thetalabr<<endl;

   return;
}

void TRelativisticKinematics::Dump()
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

void TRelativisticKinematics::PrintResults()
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
