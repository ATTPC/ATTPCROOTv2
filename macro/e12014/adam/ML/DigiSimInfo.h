namespace digiSimInfo {

void Init()
{
   FairRootManager *ioManager = FairRootManager::Instance();
   if (ioManager == nullptr) {
      LOG(ERROR) << "Cannot find RootManager!" << std::endl;
      return;
   }

   auto infoBranch = dynamic_cast<TH1D *>(ioManager->GetObject("Info"));
   if (infoBranch == nullptr) {
      cout << "infoBranch is nullptr!" << endl;
      return;
   }

   ioManager->Register("SimInfo", "AtTPC", infoBranch, true);
}

} // namespace digiSimInfo