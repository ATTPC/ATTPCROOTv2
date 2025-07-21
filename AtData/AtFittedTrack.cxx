#include "AtFittedTrack.h"

ClassImp(AtFittedTrack);

void AtFittedTrack::SetKinematics(int particleIdx, Double_t energy, Double_t theta, Double_t phi, Double_t energyxtr,
                                  Double_t thetaxtr, Double_t phixtr)
{
   while (particleIdx >= fKinematics.size()) {
      Kinematics newKinematics;
      fKinematics.push_back(newKinematics);
   }

   fKinematics[particleIdx].kineticEnergy = energy;
   fKinematics[particleIdx].theta = theta;
   fKinematics[particleIdx].phi = phi;
   fKinematics[particleIdx].kineticEnergyXtr = energyxtr;
   fKinematics[particleIdx].thetaXtr = thetaxtr;
   fKinematics[particleIdx].phiXtr = phixtr;
}

void AtFittedTrack::SetParticleInfo(int particleIdx, std::string pdg, Int_t charge, Double_t mass)
{
   while (particleIdx >= fParticleInfo.size()) {
      ParticleInfo newParticleInfo;
      fParticleInfo.push_back(newParticleInfo);
   }

   fParticleInfo[particleIdx].idPDG = TString(pdg);
   fParticleInfo[particleIdx].charge = charge;
   fParticleInfo[particleIdx].mass = mass;
}

void AtFittedTrack::SetVertex(int particleIdx, XYZVector point)
{
   while (particleIdx >= fVertex.size()) {
      XYZVector newVertex;
      fVertex.push_back(newVertex);
   }

   fVertex[particleIdx] = point;
}
