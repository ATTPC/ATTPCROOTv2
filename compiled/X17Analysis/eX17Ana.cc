#include "eX17Ana.h"

//#include "AtFitter.h"
//#include "AtGenfit.h"

#include <chrono>
#include <thread>
#include <iostream>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

int main(int argc, char *argv[])
{

   std::size_t firstEvt = 0;
   std::size_t lastEvt = 0;

   TString inputFileName = "";

   if (argc == 4) {
      firstEvt = std::atoi(argv[1]);
      lastEvt = std::atoi(argv[2]);
      inputFileName = argv[3];

      std::cout << cGREEN << " Processing file " << inputFileName << "\n";
      std::cout << " Processing events from : " << firstEvt << " to " << lastEvt << cNORMAL << "\n";

   } else {
      std::cout << " Wrong number of arguments. Expecting 3: first_event last_event fileNameWithoutExtension"
                << "\n";
      return 0;
   }

   if (lastEvt < firstEvt) {
      std::cerr << " Error!: Inconsistent numbers for first/last event. Exiting... "
                << "\n";
      return 0;
   }

   TString rootFileName = inputFileName + ".root";

   // Paths
   TString dir = getenv("VMCWORKDIR");

   TString filePath = dir + "/macro/Unpack_HDF5/e20009/rootFiles/";

   TString fileNameWithPath = filePath + rootFileName;

   std::cout << " Opening File : " << fileNameWithPath.Data() << std::endl;

   // Output ASCII file
   std::ofstream outputFile("output.txt");

   TFile *file = new TFile(fileNameWithPath.Data(), "READ");

   Int_t nEvents = lastEvt - firstEvt;

   TTree *tree = (TTree *)file->Get("cbmsim");
   std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader Reader1("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
   TTreeReaderValue<TClonesArray> evArray(Reader1, "AtEventH");
   Reader1.SetEntriesRange(firstEvt, lastEvt);

   for (Int_t i = firstEvt; i < lastEvt; i++) {

      std::cout << cGREEN << " Event Number : " << i << cNORMAL << "\n";

      Reader1.Next();

      AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);
      AtEvent *event = (AtEvent *)evArray->At(0);

      if (patternEvent) {

         std::vector<AtPad> *auxPadArray = event->GetAuxPadArray();
         std::cout << " Number of auxiliary pads : " << auxPadArray->size() << "\n";

         std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
         std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

         for (auto track : patternTrackCand) {

            std::cout << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size()
                      << " clusters "
                      << "\n";

            Int_t trackID = track.GetTrackID();

            if (track.GetIsNoise() || track.GetHitClusterArray()->size() < 3) {
               std::cout << cRED << " Track is noise or has less than 3 clusters! " << cNORMAL << "\n";
               continue;
            }

            auto hitArray = track.GetHitArray();

            for (auto hit : *hitArray) {
               TVector3 pos = hit.GetPosition();
               Int_t tb = hit.GetTimeStamp();
               Double_t charge = hit.GetCharge();
               Int_t hitid = hit.GetHitID();
               // std::cout<<pos.X()<<"     "<<pos.Y()<<"   "<<pos.Z()<<"\n";
               outputFile << i << " " << trackID << " " << hitid << " " << pos.X() << " " << pos.Y() << " " << pos.Z()
                          << " " << tb << " " << charge << "\n";
            }
         }
      }
   }

   outputFile.close();
   return 0;
}
