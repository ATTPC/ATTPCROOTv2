#ifndef __CLING__
#include "../../build/include/AtCSVReader.h"
#include "../../build/include/AtRawEvent.h"
#endif

#include <algorithm>
#include <fstream>
#include <map>
#include <vector>

/// Class used to filter events accoring to the text files made by Joe.
class TxtEvents {
   std::vector<int> fEventNumbers; // Sorted event of event numbers
   int fLastAcces = 0;             // Index of the last accessed event

public:
   void AddTxtFile(std::string fileName)
   {
      std::ifstream file(fileName);
      if (!file.is_open())
         throw std::invalid_argument("file does not exist " + fileName);
      while (!file.eof()) {
         int evt = -1;
         file >> evt;
         if (evt != -1)
            fEventNumbers.push_back(evt);
      }
      std::sort(fEventNumbers.begin(), fEventNumbers.end());
      fLastAcces = 0;
   }

   // Returns true if the event number is in the added text files
   bool operator()(AtBaseEvent *event)
   {
      int eventNum = event->GetEventID();
      if (fEventNumbers[fLastAcces] > eventNum) {
         if (fLastAcces == 0)
            return false;
         fLastAcces--;
         return (*this)(event);
      }
      while (fEventNumbers[fLastAcces] < eventNum)
         fLastAcces++;

      return fEventNumbers[fLastAcces] == eventNum;
   }

   bool operator()(int eventNum)
   {
      if (fEventNumbers[fLastAcces] > eventNum) {
         if (fLastAcces == 0)
            return false;
         fLastAcces--;
         return (*this)(eventNum);
      }
      while (fEventNumbers[fLastAcces] < eventNum)
         fLastAcces++;

      return fEventNumbers[fLastAcces] == eventNum;
   }
};

class EventMap {
   std::map<int, int> fMap;    // fMap[nscl] = tpc
   std::map<int, int> fInvMap; // fInvMap[tpc] = nscl

public:
   EventMap(TString fileName) : EventMap(std::string(fileName.Data())) {}
   EventMap(std::string fileName)
   {

      std::ifstream file(fileName);
      if (!file.is_open())
         throw std::invalid_argument("file does not exist " + fileName);

      // Clear the header
      std::string header;
      std::getline(file, header);

      for (auto &row : CSVRange<int>(file)) {
         int nscl = row[0];
         int tpc = row[1];
         fMap[nscl] = tpc;
         if (tpc != -1)
            fInvMap[tpc] = nscl;
      }
   }

   int GetNsclRunNum(int tpc)
   {
      if (fInvMap.find(tpc) == fInvMap.end())
         return -1;
      return fInvMap[tpc];
   }
};
