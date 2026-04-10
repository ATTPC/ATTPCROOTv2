#ifndef ATFITTRACKMETADATA_H
#define ATFITTRACKMETADATA_H

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, ClassDefOverride
#include <TObject.h>

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * Class for storing the result of the fit of an AtTrack from an AtFitter class.
 */
class AtFitTrackMetadata : public TObject {
protected:
   // Statistics parameters of the fit.
   Double_t fPValue{0};
   Double_t fChi2{0};
   Int_t fNdf{0};
   Bool_t fFitConverged{false};

   // The track ID for which this fit was done for.
   Int_t fTrackID{-1};

public:
   AtFitTrackMetadata() = default;
   ~AtFitTrackMetadata() = default;

   void SetPValue(Double_t value) { fPValue = value; }
   void SetChi2(Double_t value) { fChi2 = value; }
   void SetNdf(Int_t value) { fNdf = value; }
   void SetFitConverged(Bool_t value) { fFitConverged = value; }
   void SetTrackID(Int_t value) { fTrackID = value; }

   Double_t GetPValue() const { return fPValue; }
   Double_t GetChi2() const { return fChi2; }
   Int_t GetNdf() const { return fNdf; }
   Bool_t GetFitConverged() const { return fFitConverged; }
   Int_t GetTrackID() const { return fTrackID; }

   virtual void Print() const;

   ClassDefOverride(AtFitTrackMetadata, 1);
};

#endif
