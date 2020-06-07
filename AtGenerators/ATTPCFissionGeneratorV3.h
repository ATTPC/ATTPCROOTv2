#ifndef ATTPCFISSIONGENERATORV3_H
#define ATTPCFISSIONGENERATORV3_H

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

#include "AtStack.h"
#include "ATVertexPropagator.h"

#include <cstdlib>
#include <fstream>
#include <iostream>


class ATTPCFissionGeneratorV3 : public FairGenerator
{

public:

  //Default constructor
  ATTPCFissionGeneratorV3();

  // Generator that takes in a file that specifies the expected distribution of
  // fission particles as a fully realized path. The ion list should have no repeated
  // entries.
  // The name is passed as the title for TNamed
  ATTPCFissionGeneratorV3(const char *name, TString ionList, TString fissionDistro);

  // Deep copy constructor
  ATTPCFissionGeneratorV3(ATTPCFissionGeneratorV3 &copy);

  // The main physics is here
  Bool_t ReadEvent(FairPrimaryGenerator* primGen) override;

  /** Destructor **/
  virtual ~ATTPCFissionGeneratorV3();

  // Internal variables for tracking the physics 
private:
  //Tree
  TTree *fissionEvents;
  //Variables read from the file for each event
  Int_t nTracks;
  Int_t Aout[100];
  Int_t Zout[100];
  Double_t pX[100];
  Double_t pY[100];
  Double_t pZ[100];
  Double_t pT[100];
  
  Int_t nEvents; // Number of unique fission simualtion events
  Int_t event;   // Track what event we are at. Reset to 0 if we flow over the number of events in the tree
  
  ClassDef(ATTPCFissionGeneratorV3, 3)
    
};


#endif //#ifndef ATTPCFISSIONGENERATORV3_H
