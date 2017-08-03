
#ifndef ATTPCFISSIONGENERATORV2_H
#define ATTPCFISSIONGENERATORV2_H


#include "FairGenerator.h"
#include "FairIon.h"
#include "FairParticle.h"

#include "TTree.h"

#include <iostream>
#include <map>

class FairPrimaryGenerator;
class ATTPCFissionGeneratorV2;


class ATTPCFissionGeneratorV2 : public FairGenerator
{

 public:

  /** Default constructor **/
  ATTPCFissionGeneratorV2();

  ATTPCFissionGeneratorV2(const char* name,TString simfile);

  ATTPCFissionGeneratorV2(const ATTPCFissionGeneratorV2&);


  ATTPCFissionGeneratorV2& operator=(const ATTPCFissionGeneratorV2&) { return *this; }

  virtual Bool_t ReadEvent(FairPrimaryGenerator* primGen);


  /** Destructor **/
  virtual ~ATTPCFissionGeneratorV2();

private:

  static Int_t fgNIon;                      //! Number of the instance of this class
  Int_t    fMult;                           // Multiplicity per event
  Bool_t fIsDecay;
  Bool_t fNoSolution;

  std::vector<TTree*> pTree;                // vector to contain a pointer to the tree
  Double_t fVx, fVy, fVz;                   // Vertex coordinates [cm]
  Double_t fP1x, fP1y, fP1z;                // Momentum components [GeV] per nucleon
  Double_t fP2x, fP2y, fP2z;                // Momentum components [GeV] per nucleon
  Int_t Evnt;
  Int_t event;
  Int_t Aout[100],Zout[100],Ntrack;
  Double_t fOutPx[100],fOutPy[100],fOutPz[100];


  ClassDef(ATTPCFissionGeneratorV2,1)

};


#endif
