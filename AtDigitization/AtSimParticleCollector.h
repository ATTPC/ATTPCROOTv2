#ifndef ATSIMPARTICLECOLLECTOR_H
#define ATSIMPARTICLECOLLECTOR_H

#include <FairGenericStack.h>

#include <Rtypes.h> // for Int_t, Double_t, TMCProcess, etc.

#include <vector>

class TRefArray;
class TParticle;

/**
 * @brief Minimal FairGenericStack stub that captures PushTrack() calls as TParticles.
 *
 * Pass an instance of this class to FairPrimaryGenerator::GenerateEvent() instead of the
 * real VMC stack. All generator logic (vertex offsets, beam-angle rotations, PDG lookups,
 * energy calculation) runs unchanged; the resulting particles land here rather than in Geant4.
 * Storing TParticle directly preserves parentage, polarization, weight, and vertex time —
 * none of which the previous bespoke struct captured.
 *
 * Only primary particles marked for tracking (toBeDone == 1) are stored.
 *
 * @note GetCurrentTrack() / PopNextTrack() / PopPrimaryForTracking() exist only to satisfy
 *       TVirtualMCStack's pure-virtual interface. They LOG(fatal) if called and must never
 *       be reached in the SimpleSim pipeline, which only uses PushTrack() and iterates
 *       GetParticles() afterwards.
 */
class AtSimParticleCollector : public FairGenericStack {
public:
   AtSimParticleCollector() = default;
   ~AtSimParticleCollector() override;

   /// Full 19-argument PushTrack: stores a TParticle carrying parent, polarization, weight, time.
   void PushTrack(Int_t toBeDone, Int_t parentID, Int_t pdgCode, Double_t px, Double_t py, Double_t pz, Double_t e,
                  Double_t vx, Double_t vy, Double_t vz, Double_t time, Double_t polx, Double_t poly, Double_t polz,
                  TMCProcess proc, Int_t &ntr, Double_t weight, Int_t is, Int_t secondparentID) override;

   /// 18-argument overload (legacy VMC signature). Delegates with secondparentID = -1.
   void PushTrack(Int_t toBeDone, Int_t parentID, Int_t pdgCode, Double_t px, Double_t py, Double_t pz, Double_t e,
                  Double_t vx, Double_t vy, Double_t vz, Double_t time, Double_t polx, Double_t poly, Double_t polz,
                  TMCProcess proc, Int_t &ntr, Double_t weight, Int_t is) override
   {
      PushTrack(toBeDone, parentID, pdgCode, px, py, pz, e, vx, vy, vz, time, polx, poly, polz, proc, ntr, weight, is,
                -1);
   }

   /// Collected particles. Ownership stays with the collector; vector is cleared on Reset().
   const std::vector<TParticle *> &GetParticles() const { return fParticles; }

   // ---- TVirtualMCStack pure-virtual stubs. See class note. ----
   TParticle *PopNextTrack(Int_t &itrack) override;
   TParticle *PopPrimaryForTracking(Int_t i) override;
   TParticle *GetCurrentTrack() const override;

   void SetCurrentTrack(Int_t itrack) override { fCurrentTrack = itrack; }
   Int_t GetNtrack() const override { return static_cast<Int_t>(fParticles.size()); }
   Int_t GetNprimary() const override { return static_cast<Int_t>(fParticles.size()); }
   Int_t GetCurrentTrackNumber() const override { return fCurrentTrack; }
   Int_t GetCurrentParentTrackNumber() const override;

   // ---- FairGenericStack overrides (non-pure; trivial for a collector) ----
   void FillTrackArray() override {}
   void UpdateTrackIndex(TRefArray *) override {}
   void Reset() override;

private:
   std::vector<TParticle *> fParticles; ///< Owned TParticle pointers.
   Int_t fCurrentTrack{-1};
};

#endif // ATSIMPARTICLECOLLECTOR_H
