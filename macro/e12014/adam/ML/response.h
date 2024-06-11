namespace Response {

double response[512] = {};
TString sharedInfoDir = "~/fission/data/e12014/tpcSharedInfo/";

double scaling = 1.0;

void Init()
{
   AtRawEvent *respAvgEvent;
   TFile *f1 = new TFile(sharedInfoDir + "respAvg.root");
   f1->GetObject("avgResp", respAvgEvent);
   f1->Close();

   auto pad = respAvgEvent->GetPad(0);
   for (int i = 0; i < 512; i++) {
      response[i] = pad->GetADC(i);
   }
}

double GetResponse(int padNum, double time)
{
   return scaling * response[(int)time];
}

} // namespace Response