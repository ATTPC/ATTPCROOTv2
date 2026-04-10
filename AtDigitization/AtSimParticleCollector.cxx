#include "AtSimParticleCollector.h"

#include <FairLogger.h>

TParticle *AtSimParticleCollector::PopNextTrack(Int_t & /*itrack*/)
{
   LOG(fatal) << "AtSimParticleCollector::PopNextTrack() is not implemented. "
              << "This stub stack only supports PushTrack().";
   return nullptr;
}
TParticle *AtSimParticleCollector::PopPrimaryForTracking(Int_t /*iPrim*/)
{
   LOG(fatal) << "AtSimParticleCollector::PopPrimaryForTracking() is not implemented. "
              << "This stub stack only supports PushTrack().";
   return nullptr;
}
TParticle *AtSimParticleCollector::GetCurrentTrack() const
{
   LOG(fatal) << "AtSimParticleCollector::GetCurrentTrack() is not implemented. "
              << "This stub stack only supports PushTrack().";
   return nullptr;
}

void AtSimParticleCollector::PushTrack(Int_t toBeDone, Int_t /*parentID*/, Int_t pdgCode, Double_t px, Double_t py,
                                       Double_t pz, Double_t e, Double_t vx, Double_t vy, Double_t vz,
                                       Double_t /*time*/, Double_t /*polx*/, Double_t /*poly*/, Double_t /*polz*/,
                                       TMCProcess /*proc*/, Int_t &ntr, Double_t /*weight*/, Int_t /*is*/,
                                       Int_t /*secondparentID*/)
{
   ntr = static_cast<Int_t>(fParticles.size());
   if (toBeDone) {
      fParticles.push_back({ntr, pdgCode, px, py, pz, e, vx, vy, vz});
   }
}
