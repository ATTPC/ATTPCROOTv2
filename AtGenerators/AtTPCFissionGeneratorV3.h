#ifndef AtTPCFISSIONGENERAtORV3_H
#define AtTPCFISSIONGENERAtORV3_H

/* Fission generator for beam induced fission events
 * Takes a csv file that defines the distribution of fission fragments
 * availible. An example can be found in ______.
 *
 * Adam Anthony 8/23/2019
 *
 */

#include "FairGenerator.h"
#include "FairIon.h"
#include "FairLogger.h"
#include "FairParticle.h"
#include "FairPrimaryGenerator.h"
#include "FairRunSim.h"

#include "TDatabasePDG.h"
#include "TVirtualMC.h" //For gMC
#include "Math/GenVector/LorentzVector.h"

#include "AtStack.h"
#include "AtVertexPropagator.h"

#include <cstdlib>
#include <fstream>
#include <iostream>

class AtTPCFissionGeneratorV3 : public FairGenerator {

private:
   using VecXYZE = ROOT::Math::LorentzVector<ROOT::Math::PxPyPzE4D<>>;
   // Tree
   TFile *fEventFile; //!
   TTree *fEventTree; //!

   // Variables read from the file for each event
   std::vector<VecXYZE> *fDecayFrags; //!
   std::vector<Int_t> *fA;            //!
   std::vector<Int_t> *fZ;            //!

   Int_t fNumEvents; //! Number of unique fission simualtion events
   Int_t fCurrEvent; //! Track what event we are at. Reset to 0 if we flow over the number of events in the tree

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

   ClassDefOverride(AtTPCFissionGeneratorV3, 5)
};

#endif //#ifndef AtTPCFISSIONGENERAtORV3_H
