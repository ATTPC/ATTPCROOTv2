namespace radiusCheck {
ifstream fissFile;
ifstream beamFile;
ifstream otherFile;
string fissFileName = "data/fissionEventsSim.txt";
string beamFileName = "data/beamEventsSim.txt";
string otherFileName = "data/otherEventsSim.txt";

string outFileName = "data/finalFissionEventsSim.txt";
string outFileNameBeam = "data/finalBeamEventsSim.txt";
string outFileNameOther = "data/finalOtherEventsSim.txt";
string outFileNameOut = "data/outEventsSim.txt";

int fissionOut = 0;
int beamOut = 0;
int otherOut = 0;
int fissionIn = 0;
int beamIn = 0;
int otherIn = 0;

int fissionEvent = -1;
int beamEvent = -1;
int otherEvent = -1;

vector<int> fissionList;
vector<int> beamList;
vector<int> otherList;

vector<int> outList;

void init()
{
   fissFile.open(fissFileName);
   if (!fissFile.is_open()) {
      cout << fissFileName << " is not open!" << endl;
      return;
   }

   beamFile.open(beamFileName);
   if (!beamFile.is_open()) {
      cout << beamFileName << " is not open!" << endl;
      return;
   }

   otherFile.open(otherFileName);
   if (!otherFile.is_open()) {
      cout << otherFileName << " is not open!" << endl;
      return;
   }

   fissFile >> fissionEvent;
   beamFile >> beamEvent;
   otherFile >> otherEvent;
}

void exec(AtTabInfo *tabInfo)
{
   auto rawEvent = (tabInfo->GetAugment<AtTabInfoFairRoot<AtRawEvent>>("AtRawEvent"))->GetInfo();

   if (rawEvent == nullptr) {
      LOG(info) << "fRawEvent is nullptr for CrossSecPlots! Please set the raw event branch.";
      return;
   }

   int eventNum = (rawEvent->GetEventID() - 1) / 2.;
   bool isGood = rawEvent->IsGood();

   if (eventNum == fissionEvent) {
      if (isGood) {
         fissionOut++;
         fissionList.push_back(eventNum);
         outList.push_back(eventNum);
      } else {
         fissionIn++;
      }
      fissFile >> fissionEvent;
   }

   if (eventNum == beamEvent) {
      if (isGood) {
         beamOut++;
         beamList.push_back(eventNum);
         outList.push_back(eventNum);
      } else {
         beamIn++;
      }
      beamFile >> beamEvent;
   }

   if (eventNum == otherEvent) {
      if (isGood) {
         otherOut++;
         otherList.push_back(eventNum);
         outList.push_back(eventNum);
      } else {
         otherIn++;
      }
      otherFile >> otherEvent;
   }
}

bool CheckRadius(AtTabInfo *tabInfo, double radius)
{
   auto fissionEvent = (tabInfo->GetAugment<AtTabInfoFairRoot<AtFissionEvent>>("AtFissionEvent"))->GetInfo();
   if (fissionEvent == nullptr) {
      LOG(info) << "fissionEvent is nullptr for CrossSecPlots!" << endl;
      return false;
   } else {
      if (fissionEvent->GetYPattern() == nullptr) {
         return false;
      }
      auto vertex = fissionEvent->GetYPattern()->GetVertex();
      auto frag0 = fissionEvent->GetYPattern()->GetFragmentDirection(0);
      auto frag1 = fissionEvent->GetYPattern()->GetFragmentDirection(1);

      auto hitPointX0 = vertex.Z() / frag0.Z() * frag0.X() + vertex.X();
      auto hitPointY0 = vertex.Z() / frag0.Z() * frag0.Y() + vertex.Y();
      auto hitPointR0 = sqrt(pow(hitPointX0, 2) + pow(hitPointY0, 2));
      auto hitPointX1 = vertex.Z() / frag1.Z() * frag1.X() + vertex.X();
      auto hitPointY1 = vertex.Z() / frag1.Z() * frag1.Y() + vertex.Y();
      auto hitPointR1 = sqrt(pow(hitPointX1, 2) + pow(hitPointY1, 2));

      return hitPointR0 > radius && hitPointR1 > radius;
   }
}

bool CheckRadius2(AtTabInfo *tabInfo, double radius)
{
   auto event = (tabInfo->GetAugment<AtTabInfoFairRoot<AtEvent>>("AtEvent"))->GetInfo();
   if (event == nullptr) {
      LOG(info) << "event is nullptr for CrossSecPlots!" << endl;
      return false;
   } else {
      vector<int> pads;
      bool newPad = true;
      for (auto &hit : event->GetHits()) {
         if (sqrt(pow(hit->GetPosition().X(), 2) + pow(hit->GetPosition().Y(), 2)) > radius) {
            for (auto padNum : pads) {
               if (padNum == hit->GetPadNum()) {
                  newPad = false;
               }
            }
            if (newPad)
               pads.push_back(hit->GetPadNum());
         }
      }
      if (pads.size() > 5)
         return true;
   }
   return false;
}

void SaveFile(AtTabInfo *tabInfo) {}

bool CheckOther(AtTabInfo *tabInfo)
{
   auto rawEvent = (tabInfo->GetAugment<AtTabInfoFairRoot<AtRawEvent>>("AtRawEvent"))->GetInfo();

   if (rawEvent == nullptr) {
      LOG(info) << "fRawEvent is nullptr for CrossSecPlots! Please set the raw event branch.";
      return false;
   }

   int eventNum = (rawEvent->GetEventID() - 1) / 2.;

   ifstream checkFile;
   checkFile.open(outFileNameOther);

   if (!checkFile.is_open()) {
      cout << otherFileName << " is not open!" << endl;
      return false;
   }

   string line;
   while (getline(checkFile, line)) {
      if (eventNum == stoi(line)) {
         checkFile.close();
         return true;
      }
   }
   return false;
}

void Finishing()
{
   cout << "Fission outside smart zap: " << fissionOut << endl;
   cout << "Beam outside smart zap: " << beamOut << endl;
   cout << "Other outside smart zap: " << otherOut << endl;
   cout << "Ratio of Fission to total outside: " << fissionOut * 1.0 / (beamOut + fissionOut + otherOut) << endl;
   ;

   cout << "Fission inside smart zap: " << fissionIn << endl;
   cout << "Beam inside smart zap: " << beamIn << endl;
   cout << "Other inside smart zap: " << otherIn << endl;

   fissFile.close();
   beamFile.close();
   otherFile.close();

   ofstream outFile;
   outFile.open(outFileName);
   for (auto val : fissionList)
      outFile << val << endl;
   outFile.close();

   ofstream outFileBeam;
   outFileBeam.open(outFileNameBeam);
   for (auto val : beamList)
      outFileBeam << val << endl;
   outFileBeam.close();

   ofstream outFileOther;
   outFileOther.open(outFileNameOther);
   for (auto val : otherList)
      outFileOther << val << endl;
   outFileOther.close();

   ofstream outFileOut;
   outFileOut.open(outFileNameOut);
   for (auto val : outList)
      outFileOut << val << endl;
   outFileOut.close();
}
} // namespace radiusCheck