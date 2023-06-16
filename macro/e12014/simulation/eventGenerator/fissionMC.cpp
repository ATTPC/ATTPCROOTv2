/* Simplified Monte-Carlo simulation for AT-TPC fission observables
 * Monte Carlo simulation for the fission of Pb region
 * nuclei in the AT-TPC at MSU
 *
 * Adam Anthony 2/25/19
 * Modified 5/24/21, random angle was wrong
 * Modified 8/17/21, added different ways of generating Z distributions
 */

#include "TCanvas.h"
#include "TFile.h"
#include "TH2.h"
#include "TLorentzVector.h"
#include "TMath.h"
#include "TRandom3.h"
#include "TString.h"
#include "TTree.h"

#include <fstream>
#include <iostream>
#include <numeric>
#include <set>

#include "Math/Vector3D.h"
#include "Math/Vector4D.h"

const double AtoE = 939.0;
const double c = 2.998e8; // In m/s
using VecXYZE = ROOT::Math::LorentzVector<ROOT::Math::PxPyPzE4D<>>;
using VecPolar = ROOT::Math::Polar3D<double>;
using XYZVector = ROOT::Math::XYZVector;
using vecInt = std::vector<Int_t>;
using test = AtTools::AtKinematics; // Literally this is just to force ROOT to load the AtTools lib

TF1 *fAsym = nullptr;

// Function prototypes
vecInt getProducMasses(Int_t A, Float_t massFrac, Float_t massDev, TRandom *rand = new TRandom3());
vecInt getProductChargeSameDistro(Int_t Z, const vecInt &masses);
vecInt getProductChargeMaxBE(Int_t Z, const vecInt &masses);
vecInt getProductChargeMaxBEA(Int_t Z, const vecInt &masses);
XYZVector SampleUniform()
{
   Double_t x, y, z;
   gRandom->Sphere(x, y, z, 1.0);
   return {x, y, z};
}
XYZVector SampleAsym()
{

   if (fAsym == nullptr) {
      fAsym = new TF1("asym",
                      "1 + [0] * cos(x) * cos(x) + [1] * cos(x) * cos(x) * cos(x) * cos(x)+ [2] * cos(x) * cos(x)* "
                      "cos(x) * cos(x)* cos(x) * cos(x)",
                      0, TMath::Pi());
      fAsym->SetParameters(0.2676, 0.7282, 0.5472);
   }

   double theta = fAsym->GetRandom();
   VecPolar decayAng(1, theta, gRandom->Uniform(TMath::TwoPi()));
   return {decayAng.x(), decayAng.y(), decayAng.z()};
}

std::vector<VecXYZE> getProductMomenta(const vecInt &fragA, const vecInt &fragZ, TRandom *rand, VecPolar &decayAng);

Int_t sumVector(const vecInt &vec)
{
   return std::accumulate(vec.cbegin(), vec.cend(), 0);
}

// Returns the average total kinetic energy from viola systematics in MeV
double violaEn(int A, int Z)
{
   return 0.1189 * Z * Z / TMath::Power(A, 1.0 / 3.0) + 7.3;
}

/* Routine to simulate N fission events of Nuclus with charge Z and mass A at beam energy E

Input:
beamEn: Beam Energy (MeV/u)
N: Number of events
Z: Atomic number
A: Atomic mass number

optional:
massFrac: Central value of the mass distro for the more massive product as a fraction of A
massDev: Deviation of the gaussian in amu
polarAng: Angle of decay with respect to beam axis (in degrees)

Output:
fissionFragments.root with histograms of results and a TTree of generated events in rest frame
ion_list.dat a file listing all of the ions in fissionFragments.root for TPC simulation
A1 is always the more massive fragment, and A2 is the lighter.
*/
int fissionMC(int N, int Z = 82, int A = 196, float massFrac = 0.56, float massDev = 6, float polarAng = 0,
              TString fName = "fissionFragmentsIsotropic.root")
{
   TRandom3 rand;

   // Create a file to save the generated histograms
   TFile *f = new TFile(fName, "RECREATE");
   TTree tr("fragments", "Fission Fragments in CoM frame");
   std::vector<VecXYZE> decayMomenta;
   std::vector<Int_t> fragA;
   std::vector<Int_t> fragZ;

   // Add branches to tree
   tr.Branch("decayFragments", &decayMomenta);
   tr.Branch("A", &fragA);
   tr.Branch("Z", &fragZ);

   // Create set to hold the ions...
   std::set<std::pair<int, int>> ionSet; //<A,Z>

   for (int i = 0; i < N; ++i) {
      if (i % (N / 10) == 0)
         std::cout << "Processing event: " << i << "/" << N << std::endl;

      fragA = getProducMasses(A, massFrac, massDev, &rand);
      // fragZ = getProductChargeSameDistro(Z, fragA);
      // fragZ = getProductChargeMaxBE(Z, fragA);
      fragZ = getProductChargeMaxBEA(Z, fragA);

      // Generate an array of products in the rest frame using the supplied angle
      // or a random angle if 0 was supplied
      VecPolar decayAngle(1, TMath::DegToRad() * polarAng, 0); // r, theta, phi

      decayMomenta = getProductMomenta(fragA, fragZ, &rand, decayAngle);
      for (int j = 0; j < fragA.size(); ++j)
         ionSet.insert(std::pair<int, int>(fragA.at(j), fragZ.at(j)));

      tr.Fill();
   } // End loop over N events

   // Write out the file
   f->Write();
   f->Close();

   // Print out the ion list
   std::ofstream ionFile("ion_list.csv");
   if (!ionFile) {
      std::cerr << "Failed to open ion file!" << std::endl;
      return -1;
   }

   ionFile << "A,Z" << std::endl;
   for (const auto &ion : ionSet)
      ionFile << ion.first << "," << ion.second << std::endl;

   ionFile.close();
   return 0;
}

vecInt getProducMasses(Int_t A, Float_t massFrac, Float_t massDev, TRandom *rand)
{
   Int_t A1 = TMath::Nint(rand->Gaus(A * massFrac, massDev));
   if (A1 < A / 2)
      A1 = A - A1;
   vecInt fragA;
   fragA.push_back(A1);
   fragA.push_back(A - A1);
   return fragA;
}

vecInt getProductChargeSameDistro(Int_t Z, const vecInt &masses)
{
   Int_t A = sumVector(masses);
   Int_t Z1 = TMath::Nint((double)masses.at(0) * Z / A);
   vecInt fragZ;
   fragZ.push_back(Z1);
   fragZ.push_back(Z - Z1);
   return fragZ;
}

vecInt getProductChargeMaxBE(Int_t Z, const vecInt &masses)
{
   auto asym = 23.2;
   auto coul = 0.714;
   Int_t A = sumVector(masses);

   Float_t numerator = 8 * asym * masses.at(0) * Z;
   numerator += coul * TMath::Power(masses.at(0), 2.0 / 3.0) * masses.at(1);
   numerator += coul * TMath::Power(masses.at(1), 2.0 / 3.0) * masses.at(0) * (2 * Z - 1);

   Float_t denominator = 8 * asym * A;
   denominator += 2 * coul * TMath::Power(masses.at(1), 2.0 / 3.0) * masses.at(0);
   denominator += 2 * coul * TMath::Power(masses.at(0), 2.0 / 3.0) * masses.at(1);

   Int_t Z1 = TMath::Nint(numerator / denominator);
   vecInt fragZ;
   fragZ.push_back(Z1);
   fragZ.push_back(Z - Z1);
   return fragZ;
}

vecInt getProductChargeMaxBEA(Int_t Z, const vecInt &masses)
{
   // LDM parameters
   auto asym = 23.2;
   auto coul = 0.714;

   Int_t A = sumVector(masses);
   auto A1 = masses.at(0);
   auto A2 = masses.at(1);

   Float_t numAsym = A2 * A2 - A1 * A2 + 2 * Z * A1;
   Float_t numerator = 4 * asym * A1 * numAsym;

   numerator += coul * TMath::Power(A1, 2.0 / 3.0) * A2 * A2;
   numerator += coul * TMath::Power(A2, 2.0 / 3.0) * A1 * A1 * (2 * Z - 1);

   Float_t denominator = 8 * asym * (A1 * A1 + A2 * A2);
   denominator += 2 * coul * TMath::Power(A2, 2.0 / 3.0) * A1 * A1;
   denominator += 2 * coul * TMath::Power(A1, 2.0 / 3.0) * A2 * A2;

   Int_t Z1 = TMath::Nint(numerator / denominator);
   vecInt fragZ;
   fragZ.push_back(Z1);
   fragZ.push_back(Z - Z1);
   return fragZ;
}

std::vector<VecXYZE> getProductMomenta(const vecInt &fragA, const vecInt &fragZ, TRandom *rand, VecPolar &decayAng)
{

   Int_t Z = sumVector(fragZ);
   Int_t A = sumVector(fragA);

   // Get the masses of the fragments in MeV
   double m[2];
   for (int i = 0; i < 2; ++i)
      m[i] = fragA.at(i) * AtoE;

   // Get the KE of the products
   double kE = violaEn(A, Z);

   double gamma1 = AtTools::Kinematics::GetGamma(kE, m[0], m[1]);
   double p1 = AtTools::Kinematics::GetRelMom(gamma1, m[0]);

   /*
      // Assume the kinetic energy is evenly split between the particles
      // Get the energies of the two particles
      double E[2];
      E[1] = m[0] * m[1] + m[1] * m[1] + kE * (m[0] + m[1]) + kE * kE / 2.0;
      E[1] /= m[0] + m[1] + kE;
      E[0] = TMath::Sqrt(E[1] * E[1] + m[0] * m[0] - m[1] * m[1]);
      */

   // Set the momentum unit vector if nullptr
   if (decayAng.Theta() != 0)
      decayAng.SetPhi(rand->Uniform(TMath::TwoPi()));
   else {
      // decayAng.SetPhi(rand->Uniform(TMath::TwoPi()));
      // decayAng.SetTheta(rand->Uniform(TMath::Pi()));
      decayAng = SampleUniform();
      //  decayAng = SampleAsym();
   }

   // Set the momentum of first particles
   // decayAng.SetR(TMath::Sqrt(E[0] * E[0] - m[0] * m[0]));
   decayAng.SetR(p1);
   ROOT::Math::XYZVector pDir(decayAng);

   std::vector<VecXYZE> ret;
   // ret.push_back(VecXYZE(decayAng.X(), decayAng.Y(), decayAng.Z(), E[0]));
   // ret.push_back(VecXYZE(-decayAng.X(), -decayAng.Y(), -decayAng.Z(), E[1]));
   ret.push_back(AtTools::Kinematics::Get4Vector(pDir, m[0]));
   ret.push_back(AtTools::Kinematics::Get4Vector(-pDir, m[1]));

   return ret;
}

// Run and generate a bunch of images
void run(double beamEn, double beamEn2)
{
   // Simulation parameters
   int N = 100000;
   int Z = 82, A = 196;
   float massFrac = 0.56, massDev = 6;
   TFile *f;
}
