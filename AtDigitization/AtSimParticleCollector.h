#ifndef ATSIMPARTICLECOLLECTOR_H
#define ATSIMPARTICLECOLLECTOR_H

#include <FairGenericStack.h>

#include <Rtypes.h> // for Int_t, Double_t, TMCProcess, etc.

#include <vector>

class TRefArray;
class TParticle;

/**
 * @brief Particle captured from FairPrimaryGenerator::GenerateEvent().
 *
 * Units follow FairRoot conventions as they come out of FairPrimaryGenerator::AddTrack():
 *  - position in cm
 *  - momentum (px, py, pz) in GeV/c, total energy e in GeV
 */
struct AtCollectedParticle {
   int trackID;
   int pdgCode;
   double px, py, pz; ///< Momentum in GeV/c
   double e;          ///< Total energy in GeV
   double vx, vy, vz; ///< Vertex position in cm
};

/**
 * @brief Minimal FairGenericStack stub that captures PushTrack() calls.
 *
 * Pass an instance of this class to FairPrimaryGenerator::GenerateEvent() instead of the
 * real VMC stack. All generator logic (vertex offsets, beam-angle rotations, PDG lookups,
 * energy calculation) runs unchanged; the resulting particles land here rather than in Geant4.
 *
 * Only primary particles to be tracked (toBeDone == 1) are stored.
 *
 * @note GetCurrentTrack(), PopNextTrack(), and PopPrimaryForTracking() are not implemented
 * and will LOG(fatal) if called. Generators that only call PushTrack() work correctly. If a
 * generator needs to inspect the stack, these stubs must be extended to return synthesized
 * TParticle objects.
 */
class AtSimParticleCollector : public FairGenericStack {
   std::vector<AtCollectedParticle> fParticles;
   int fCurrentTrack{-1};

public:
   AtSimParticleCollector() = default;

   // ---- PushTrack: the only method we actually need ----
   virtual void PushTrack(Int_t toBeDone, Int_t parentID, Int_t pdgCode, Double_t px, Double_t py, Double_t pz,
                          Double_t e, Double_t vx, Double_t vy, Double_t vz, Double_t time, Double_t polx,
                          Double_t poly, Double_t polz, TMCProcess proc, Int_t &ntr, Double_t weight, Int_t is,
                          Int_t secondparentID);

   const std::vector<AtCollectedParticle> &GetParticles() const { return fParticles; }
   void Clear() { fParticles.clear(); }

   // ---- TVirtualMCStack 18-param PushTrack (delegates to the 19-param FairGenericStack version) ----
   // FairGenericStack implements this in its .cxx (invisible to Cling), so we must provide it
   // in the header to satisfy the pure-virtual requirement during dictionary generation.
   virtual void PushTrack(Int_t toBeDone, Int_t parentID, Int_t pdgCode, Double_t px, Double_t py, Double_t pz,
                          Double_t e, Double_t vx, Double_t vy, Double_t vz, Double_t time, Double_t polx,
                          Double_t poly, Double_t polz, TMCProcess proc, Int_t &ntr, Double_t weight, Int_t is)
   {
      PushTrack(toBeDone, parentID, pdgCode, px, py, pz, e, vx, vy, vz, time, polx, poly, polz, proc, ntr, weight,
                is, -1);
   }

   // ---- TVirtualMCStack pure-virtual stubs ----
   // These are not implemented and will LOG(fatal) if called. See class-level @note.
   virtual TParticle *PopNextTrack(Int_t &itrack);
   virtual TParticle *PopPrimaryForTracking(Int_t i);
   virtual TParticle *GetCurrentTrack() const;
   virtual void SetCurrentTrack(Int_t itrack) { fCurrentTrack = itrack; }
   virtual Int_t GetNtrack() const { return static_cast<Int_t>(fParticles.size()); }
   virtual Int_t GetNprimary() const { return static_cast<Int_t>(fParticles.size()); }
   virtual Int_t GetCurrentTrackNumber() const { return fCurrentTrack; }
   virtual Int_t GetCurrentParentTrackNumber() const { return -1; }

   // ---- FairGenericStack virtual stubs ----
   virtual void AddParticle(TParticle *) {}
   virtual void FillTrackArray() {}
   virtual void UpdateTrackIndex(TRefArray *) {}
   virtual void Reset() { Clear(); }
};

#endif // ATSIMPARTICLECOLLECTOR_H
