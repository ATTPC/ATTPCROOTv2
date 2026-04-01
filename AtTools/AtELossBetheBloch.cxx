#include "AtELossBetheBloch.h"

#include <FairLogger.h>

#include <cmath>
#include <vector>

namespace AtTools {

AtELossBetheBloch::AtELossBetheBloch(double part_q, double part_mass, int mat_Z, int mat_A, double density, double I_eV)
   : AtELossModel(density), fPart_q(part_q), fPart_mass(part_mass)
{
   SetMaterial(mat_Z, mat_A, density, I_eV);
}

void AtELossBetheBloch::SetMaterial(int mat_Z, int mat_A, double density, double I_eV)
{
   fMat_Z = mat_Z;
   fMat_A = mat_A;
   fDensity = density;
   if (I_eV <= 0)
      fI_MeV = 13.5 * mat_Z * 1e-6; // Bloch approximation: I ≈ 13.5·Z eV
   else
      fI_MeV = I_eV * 1e-6;
   BuildSpline();
}

void AtELossBetheBloch::SetI(double I_eV)
{
   fI_MeV = I_eV * 1e-6;
   BuildSpline();
}

void AtELossBetheBloch::SetDensity(double density)
{
   fDensity = density;
   BuildSpline();
}

void AtELossBetheBloch::BuildSpline(double E_min_MeV, double E_max_MeV, int nPoints)
{
   std::vector<double> energies(nPoints);
   std::vector<double> dedxValues(nPoints);

   double logMin = std::log(E_min_MeV);
   double logMax = std::log(E_max_MeV);

   for (int i = 0; i < nPoints; ++i) {
      double t = static_cast<double>(i) / (nPoints - 1);
      energies[i] = std::exp(logMin + t * (logMax - logMin));
      dedxValues[i] = GetdEdx_formula(energies[i]);
   }

   // The Bethe-Bloch formula breaks down at low energies where the log argument drops below 1.
   // Find the lowest-energy valid grid point, then extrapolate below it using dEdx ∝ sqrt(E),
   // consistent with the low-energy Lindhard stopping power behavior.
   int firstValid = -1;
   for (int i = 0; i < nPoints; ++i) {
      if (dedxValues[i] > 0) {
         firstValid = i;
         break;
      }
   }

   if (firstValid < 0) {
      LOG(error) << "Bethe-Bloch formula returned zero for all energy grid points!";
      return;
   }
   if (firstValid > 0) {
      LOG(debug) << "Bethe-Bloch formula invalid below " << energies[firstValid]
                 << " MeV; extrapolating with dEdx ∝ sqrt(E).";
      for (int i = 0; i < firstValid; ++i)
         dedxValues[i] = dedxValues[firstValid] * std::sqrt(energies[i] / energies[firstValid]);
   }

   std::vector<double> dXdE(nPoints);
   for (int i = 0; i < nPoints; ++i)
      dXdE[i] = (dedxValues[i] > 0) ? 1.0 / dedxValues[i] : 1e10;

   fdXdE = tk::spline(energies, dXdE);

   // Build Ω²(E) spline: Bohr range variance accumulated from 0 to E.
   // ω²_unit [MeV²/mm] = K · z² · (Z/A) · ρ[g/mm³] · mₑc²
   //   (kK is in MeV cm²/mol; ×100 → MeV mm²/mol; density g/cm³ → g/mm³ = /1000; combined: /10)
   double z = IsElectron() ? 1.0 : fPart_q;
   double omega2_unit = kK * z * z * (static_cast<double>(fMat_Z) / fMat_A) * (fDensity / 10.0) * kM_e;

   // Build spline of the integrand dΩ²/dE = 1/|dEdx|³, then integrate segment-by-segment using
   // Simpson's rule (exact for cubics, O(h⁴)) rather than the O(h²) trapezoidal rule.
   std::vector<double> integrandValues(nPoints);
   for (int i = 0; i < nPoints; ++i)
      integrandValues[i] = (dedxValues[i] > 0) ? 1.0 / (dedxValues[i] * dedxValues[i] * dedxValues[i]) : 0.0;
   tk::spline integrand(energies, integrandValues);

   std::vector<double> rangeVar(nPoints, 0.0);
   for (int i = 1; i < nPoints; ++i)
      rangeVar[i] = rangeVar[i - 1] + omega2_unit * integrand.integrate(energies[i - 1], energies[i]);
   fRangeVariance = tk::spline(energies, rangeVar);
}

double AtELossBetheBloch::GetdEdx_formula(double energy) const
{
   if (IsElectron())
      return GetdEdx_electron(energy);
   return GetdEdx_heavy(energy);
}

double AtELossBetheBloch::GetdEdx_heavy(double energy) const
{
   if (energy <= 0)
      return 0;

   double M = fPart_mass;
   double gamma = 1.0 + energy / M;
   double beta2 = 1.0 - 1.0 / (gamma * gamma);

   if (beta2 <= 0)
      return 0;

   // Maximum kinetic energy transferable in a single collision (PDG 2022, Eq. 34.2)
   double Tmax = 2.0 * kM_e * beta2 * gamma * gamma / (1.0 + 2.0 * gamma * kM_e / M + (kM_e / M) * (kM_e / M));

   // Argument of logarithm: 2mₑc²β²γ²Tmax / I²
   double logArg = 2.0 * kM_e * beta2 * gamma * gamma * Tmax / (fI_MeV * fI_MeV);
   if (logArg <= 0)
      return 0;

   double z = fPart_q;
   // Standard Bethe-Bloch in MeV/cm (PDG 2022, Eq. 34.1), density correction neglected
   double dedx_cm =
      kK * z * z * (static_cast<double>(fMat_Z) / fMat_A) * fDensity / beta2 * (0.5 * std::log(logArg) - beta2);

   if (dedx_cm <= 0)
      return 0;

   return dedx_cm / 10.0; // Convert MeV/cm → MeV/mm
}

double AtELossBetheBloch::GetdEdx_electron(double energy) const
{
   if (energy <= 0)
      return 0;

   double tau = energy / kM_e;
   double beta2 = tau * (tau + 2.0) / ((tau + 1.0) * (tau + 1.0));

   if (beta2 <= 0)
      return 0;

   // Møller exchange correction (Leo 1994, Eq. 2.38)
   double Fminus = (1.0 + tau * tau / 8.0 - (2.0 * tau + 1.0) * std::log(2.0)) / ((tau + 1.0) * (tau + 1.0));

   // Argument of logarithm: τ√(τ+2)·mₑc² / (√2·I)
   double logArg = tau * std::sqrt(tau + 2.0) * kM_e / (std::sqrt(2.0) * fI_MeV);
   if (logArg <= 0)
      return 0;

   // Modified Bethe formula for electrons in MeV/cm
   double dedx_cm = kK * (static_cast<double>(fMat_Z) / fMat_A) * fDensity / beta2 * (std::log(logArg) + Fminus);

   if (dedx_cm <= 0)
      return 0;

   return dedx_cm / 10.0; // Convert MeV/cm → MeV/mm
}

double AtELossBetheBloch::GetdEdx(double energy) const
{
   return 1.0 / fdXdE(energy);
}

double AtELossBetheBloch::GetRange(double energyIni, double energyFin) const
{
   if (energyIni == energyFin)
      return 0;
   if (energyFin < fdXdE.get_x_min()) {
      LOG(debug) << "Attempting to integrate energy to " << energyFin << " when min energy in table is "
                 << fdXdE.get_x_min();
      energyFin = fdXdE.get_x_min();
   }
   return fdXdE.integrate(energyFin, energyIni);
}

double AtELossBetheBloch::GetEnergyLoss(double energyIni, double distance) const
{
   return energyIni - GetEnergy(energyIni, distance);
}

double AtELossBetheBloch::GetEnergy(double energyIni, double distance) const
{
   if (distance == 0)
      return energyIni;
   if (energyIni < 1e-6 || GetRange(energyIni) < distance)
      return 0.0;

   const int maxIt = 100;
   const double distErr = 1e-4; // mm

   double guessEnergy = energyIni - GetdEdx(energyIni) * distance;
   for (int i = 0; i < maxIt; ++i) {
      double range = GetRange(energyIni, guessEnergy);
      if (std::fabs(range - distance) < distErr) {
         LOG(debug) << "Energy converged in " << i + 1 << " iterations.";
         return guessEnergy;
      }
      guessEnergy += GetdEdx(guessEnergy) * (range - distance);
      if (guessEnergy < fdXdE.get_x_min() * 1.01) {
         guessEnergy = fdXdE.get_x_min();
         return guessEnergy;
      }
   }

   LOG(error) << "Energy calculation (" << energyIni << " MeV through " << distance << " mm) failed to converge in "
              << maxIt << " iterations!";
   return -1;
}

double AtELossBetheBloch::GetElossStraggling(double energyIni, double energyFin) const
{
   if (energyIni <= energyFin)
      return 0;
   double omega2 = GetRangeVariance(energyIni) - GetRangeVariance(energyFin);
   if (omega2 <= 0)
      return 0;
   return std::abs(GetdEdx(energyFin)) * std::sqrt(omega2);
}

double AtELossBetheBloch::GetdEdxStraggling(double energyIni, double energyFin) const
{
   double dx_mm = GetRange(energyIni, energyFin);
   if (dx_mm <= 0)
      return 0;
   double omega2 = GetRangeVariance(energyIni) - GetRangeVariance(energyFin);
   if (omega2 <= 0)
      return 0;
   return std::abs(GetdEdx(energyFin)) * std::sqrt(omega2) / dx_mm;
}

double AtELossBetheBloch::GetRangeVariance(double energy) const
{
   if (energy <= fRangeVariance.get_x_min())
      return 0;
   if (energy >= fRangeVariance.get_x_max())
      return fRangeVariance(fRangeVariance.get_x_max());
   return fRangeVariance(energy);
}

} // namespace AtTools
