#ifndef AtTPCFISSIONGENERAtORV3_H
#define AtTPCFISSIONGENERAtORV3_H

#include <FairGenerator.h>

#include <Math/GenVector/Boost.h>
#include <Math/Point3D.h>
#include <Math/Vector4D.h>
#include <Rtypes.h>
#include <TString.h>

#include <vector>

class TFile;
class TTree;
class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;

using VecPE = ROOT::Math::LorentzVector<ROOT::Math::PxPyPzE4D<>>;
using Cartesian3D = ROOT::Math::Cartesian3D<>;

class AtTPCFissionGeneratorV3 : public FairGenerator {

private:
   TFile *fEventFile{}; //!
   TTree *fEventTree{}; //!

   // Variables read from the file for each event
   std::vector<VecPE> *fDecayFrags{}; //!
   std::vector<Int_t> *fA{};          //!
   std::vector<Int_t> *fZ{};          //!

   Int_t fNumEvents{}; //! Number of unique fission simualtion events
   Int_t fCurrEvent{}; //! Track what event we are at. Reset to 0 if we flow over the number of events in the tree

   // Varibles set for each event
   FairPrimaryGenerator *fPrimeGen{}; //!
   ROOT::Math::Boost fBeamBoost;
   Cartesian3D fVertex;

   void loadIonList(TString ionList);
   void loadFissionFragmentTree(TString fissionDistro);

   VecPE getBeam4Vec();
   Cartesian3D getVertex();
   void setBeamParameters();
   void generateEvent();
   void generateFragment(VecPE &P, Int_t A, Int_t Z);

public:
   // Default constructor
   AtTPCFissionGeneratorV3();

   // Generator that takes in a file that specifies the expected distribution of
   // fission particles as a fully realized path. The ion list should have no repeated
   // entries.
   // The name is passed as the title for TNamed
   AtTPCFissionGeneratorV3(const char *name, TString ionList, TString fissionDistro);

   // Deep copy constructor
   AtTPCFissionGeneratorV3(AtTPCFissionGeneratorV3 &copy);

   // The main physics is here
   Bool_t ReadEvent(FairPrimaryGenerator *primGen) override;

   /** Destructor **/
   virtual ~AtTPCFissionGeneratorV3();

   // Internal variables for tracking the physics

   ClassDefOverride(AtTPCFissionGeneratorV3, 6)
};

#endif //#ifndef AtTPCFISSIONGENERAtORV3_H
