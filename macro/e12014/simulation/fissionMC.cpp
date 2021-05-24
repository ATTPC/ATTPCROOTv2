/* Simplified Monte-Carlo simulation for AT-TPC fission observables
 * Monte Carlo simulation for the fission of Pb region
 * nuclei in the AT-TPC at MSU
 *
 * Adam Anthony 2/25/19
 * Modified 5/24/21, random angle was wrong
 */

#include "TRandom3.h"
#include "TH2.h"
#include "TFile.h"
#include "TMath.h"
#include "TLorentzVector.h"
#include "TString.h"
#include "TCanvas.h"
#include "Math/GenVector/LorentzVector.h"
#include "TTree.h"

#include <iostream>
#include <set>
#include <fstream>

const double AtoE = 939.0;
const double c = 2.998e8; // In m/s
using VecXYZE = ROOT::Math::LorentzVector<ROOT::Math::PxPyPzE4D<>>;

// Function prototypes
std::vector<VecXYZE> genFissionProducts(int Z, int A, std::vector<Int_t> &fragA, std::vector<Int_t> &fragz,
                                        float massFrac = 0.56, int massDev = 6, TRandom *rand = new TRandom3());

std::vector<VecXYZE> genFissionProducts(int Z, int A, std::vector<Int_t> &fragA, std::vector<Int_t> &fragz,
                                        float massFrac, int massDev, TRandom *rand, TVector3 *p);
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

Output:
fissionFragments.root with histograms of results and a TTree of generated events in rest frame
ion_list.dat a file listing all of the ions in fissionFragments.root for TPC simulation
*/
int fissionMC(int N, int Z = 82, int A = 196, float massFrac = 0.56, float massDev = 6, float polarAng = 0)
{
   TRandom3 rand;

   // Create a file to save the generated histograms
   TFile *f = new TFile("fissionFragments.root", "RECREATE");
   TTree tr("fragments", "Fission Fragments in CoM frame");
   std::vector<VecXYZE> decayFragments;
   std::vector<Int_t> fragA;
   std::vector<Int_t> fragZ;

   // Add branches to tree
   tr.Branch("decayFragments", &decayFragments);
   tr.Branch("A", &fragA);
   tr.Branch("Z", &fragZ);

   // Create set to hold the ions...
   std::set<std::pair<int, int>> ionSet; //<A,Z>

   for (int i = 0; i < N; ++i) {
      if (i % (N / 10) == 0)
         std::cout << "Processing event: " << i << "/" << N << std::endl;

      // Generate an array of products in the rest frame using the supplied angle
      // or a random angle if 0 was supplied
      if (polarAng == 0)
         decayFragments = genFissionProducts(Z, A, fragA, fragZ, massFrac, massDev, &rand);
      else {
         TVector3 *p = new TVector3(1, 0, 0);
         p->SetTheta(polarAng);
         decayFragments = genFissionProducts(Z, A, fragA, fragZ, massFrac, massDev, &rand, p);
         delete p;
      }

      // Put the ion in the set
      for (int i = 0; i < fragA.size(); ++i)
         ionSet.insert(std::pair<int, int>(fragA.at(i), fragZ.at(i)));

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

/*
 * Generate an array of 4-momneta for fission nuclea with Z and A with
 * the given mass distribution
 */

std::vector<VecXYZE> genFissionProducts(int Z, int A, std::vector<Int_t> &fragA, std::vector<Int_t> &fragz,
                                        float massFrac, int massDev, TRandom *rand, TVector3 *p)

{

   // Get the masses of the two paricles
   double m[2];
   Int_t A1 = TMath::Nint(rand->Gaus(A * massFrac, massDev));
   Int_t Z1 = TMath::Nint((double)A1 * Z / A);
   if (A1 < A / 2)
      A1 = A - A1;
   m[0] = A1 * AtoE;
   m[1] = (A - A1) * AtoE;

   fragA.clear();
   fragz.clear();
   fragA.push_back(A1);
   fragz.push_back(Z1);
   fragA.push_back(A - A1);
   fragz.push_back(Z - Z1);

   // Get the KE of the products
   double kE = violaEn(A, Z);

   // Assume the kinetic energy is evenly split between the particles

   // Get the energies of the two particles
   double E[2];
   E[1] = m[0] * m[1] + m[1] * m[1] + kE * (m[0] + m[1]) + kE * kE / 2.0;
   E[1] /= m[0] + m[1] + kE;
   E[0] = TMath::Sqrt(E[1] * E[1] + m[0] * m[0] - m[1] * m[1]);

   // Set the momentum of first particles
   p->SetMag(TMath::Sqrt(E[0] * E[0] - m[0] * m[0]));

   std::vector<VecXYZE> ret;
   ret.push_back(VecXYZE(p->X(), p->Y(), p->Z(), E[0]));
   ret.push_back(VecXYZE(-p->X(), -p->Y(), -p->Z(), E[1]));
   return ret;
}
/*
 * Generate an array of 4-momneta for fission nuclea with Z and A with
 * the given mass distribution
 */

std::vector<VecXYZE> genFissionProducts(int Z, int A, std::vector<Int_t> &fragA, std::vector<Int_t> &fragz,
                                        float massFrac = 0.56, int massDev = 6, TRandom *rand)
{
   // Randomly generate the unit vector for momenta
   Double_t x, y, z;
   rand->Sphere(x, y, z, 1.0);
   TVector3 *p = new TVector3(x, y, z);

   // Actually generate the fission products
   return genFissionProducts(Z, A, fragA, fragz, massFrac, massDev, rand, p);
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
