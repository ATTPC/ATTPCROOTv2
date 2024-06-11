#include "AtTPCXSManager.h"

#include <TH2.h>

#include <fstream> // IWYU pragma: keep
#include <iostream>
#include <sstream> // IWYU pragma: keep

// Allow us use std::make_unique using a protected constructor this struct
// is only defined in this translation unit (.cpp file)
namespace {
struct concrete_AtTPCXSManager : public AtTPCXSManager {};
} // namespace

std::unique_ptr<AtTPCXSManager> AtTPCXSManager::fInstance = nullptr;

AtTPCXSManager *AtTPCXSManager::Instance()
{
   if (fInstance == nullptr)
      fInstance = std::make_unique<concrete_AtTPCXSManager>();
   return fInstance.get();
}

Bool_t AtTPCXSManager::SetExcitationFunction(std::string filename)
{
   fExFunctionFile = filename;
   std::ifstream file;
   file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

   try {
      file.open(fExFunctionFile);

      Int_t nLines = 0;
      Int_t nPoints = 0;

      Float_t Ebinsize = 0;
      Float_t Abinsize = 0;
      Float_t ERangeUp = 0;
      Float_t ARangeUp = 0;
      Float_t ERangeDown = 0;
      Float_t ARangeDown = 0;
      Float_t XSTot = 0;

      // read a single line
      std::string line;
      std::getline(file, line);
      std::cout << line << "\n";
      std::getline(file, line);
      std::istringstream isheader(line);
      isheader >> Ebinsize >> Abinsize >> ERangeUp >> ERangeDown >> ARangeUp >> ARangeDown >> XSTot;
      std::cout << " Energy bin size : " << Ebinsize << " - Angular bin size : " << Abinsize
                << " - Energy range Up : " << ERangeUp << " - Energy range down :" << ERangeDown
                << " - Angular range Up : " << ARangeUp << " - Angular range Down : " << ARangeDown
                << " - Total cross section : " << XSTot << "\n";

      auto nEbins = static_cast<Int_t>(((ERangeUp + 0.000001) - ERangeDown) / Ebinsize);
      auto nAbins = static_cast<Int_t>(((ARangeUp + 0.000001) - ARangeDown) / Abinsize);

      std::cout << " Number of energy - angle bins : " << nEbins << " - " << nAbins << "\n";

      fExFunction = std::make_shared<TH2F>("fExFunction", "fExFunction", nEbins + 1, ERangeDown - Ebinsize / 2.0,
                                           ERangeUp + Ebinsize / 2.0, nAbins + 1, ARangeDown - Abinsize / 2.0,
                                           ARangeUp + Abinsize / 2.0);

      while (!file.eof()) {

         std::getline(file, line);

         Float_t ecm, acm, xs, ph; // Energy, cross section, placeholder

         // read with default seperator (space) seperated elements
         std::istringstream isxs(line);

         isxs >> ecm >> xs >> acm >> ph;

         std::cout << " Energy cm (MeV) : " << ecm << " - Cross section (mb/sr or mb) : " << xs
                   << " - Angle cm (deg) : " << acm << " - Place holder : " << ph << "\n";

         Int_t bin = fExFunction->Fill(ecm, acm, xs);
         std::cout << " Bin fill " << bin << "\n";

         ++nPoints;
      }

      file.close();

      std::cout << "Successfully read " << nPoints << " points in " << nLines << " lines!" << std::endl;
      kIsExFunction = kTRUE;
      return true;
   } catch (...) {
      // std::cout <<cGREEN<< "Something bad happened while reading "<<mapPath<<"!" <<cNORMAL<< std::endl;
   }

   return false;
}

ClassImp(AtTPCXSManager)
