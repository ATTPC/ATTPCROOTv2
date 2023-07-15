void testHDFUnpackers()
{

   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapFile = "e12014_pad_mapping.xml";
   TString mapDir = dir + "/scripts/" + mapFile;
   auto fAtMapPtr = std::make_shared<AtTpcMap>();
   AtRawEvent rawEvent;

   std::unique_ptr<AtHDFUnpacker> unpacker = std::make_unique<AtFRIBHDFUnpacker>(fAtMapPtr);
   unpacker->SetInputFileName("/home/yassid/Desktop/run_0140.h5");
   unpacker->SetNumberTimestamps(1);
   unpacker->Init();
   unpacker->FillRawEvent(rawEvent);
   std::cout << " Number of events : " << unpacker->GetNumEvents() << "\n";
   // dynamic_cast<AtFRIBHDFUnpacker*>(unpacker.get())->open("/home/yassid/Desktop/run_0140.h5");
}
