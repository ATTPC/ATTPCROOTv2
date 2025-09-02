#include "AtFittedTrack.h"

ClassImp(AtFittedTrack);

void AtFittedTrack::SetKinematics(int particleIdx, Double_t energy, Double_t theta, Double_t phi)
{
   while (particleIdx >= fKinematics.size()) {
      Kinematics newKinematics;
      fKinematics.push_back(newKinematics);
   }

   fKinematics[particleIdx].kineticEnergy = energy;
   fKinematics[particleIdx].theta = theta;
   fKinematics[particleIdx].phi = phi;
}

void AtFittedTrack::SetKinematicsXtr(int particleIdx, Double_t energyxtr, Double_t thetaxtr, Double_t phixtr)
{
   while (particleIdx >= fKinematicsXtr.size()) {
      Kinematics newKinematics;
      fKinematicsXtr.push_back(newKinematics);
   }

   fKinematicsXtr[particleIdx].kineticEnergy = energyxtr;
   fKinematicsXtr[particleIdx].theta = thetaxtr;
   fKinematicsXtr[particleIdx].phi = phixtr;
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
