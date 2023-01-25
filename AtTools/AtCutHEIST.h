#ifndef ATCUTHEIST_H
#define ATCUTHEIST_H

#include <TString.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>

#include <HTMcp.h>
#include <HTMusicIC.h>

#include <map>    // for map
#include <memory> // for shared_ptr
#include <string>

class TFile;
class TTree;
class TCutG;

/**
 * @brief Class for managing PID cuts from HEIST data.
 * Follows the interface required to be passed as a reduction function to AtDataReductionTask
 */
class AtCutHEIST {
private:
   // Tree and reader for HEIST
   std::shared_ptr<TFile> fFile; //< File containing tree to read
   TTreeReader fReader;
   TTreeReaderValue<HTMusicIC> fMusic;
   TTreeReaderValue<HTMcp> fUsMcp;
   TTreeReaderValue<HTMcp> fDsMcp;

   std::shared_ptr<TFile> fCutFile; //< File all cuts availible
   std::map<std::string, TCutG *> fSpecies;

public:
   AtCutHEIST(TTree *tree, TString cutFile);
   AtCutHEIST(TString treeFile, TString cutFile, TString treeName = "E12014");

   bool AddSpecies(std::string species);
   void AddAllSpecies();
   std::string GetSpecies();

   bool operator()();

private:
   double GetEnergy();
   double GetToF();
};
#endif
