
void PlotPID(AtTabInfo *info)
{
   TTree *tree = info->GetAugment<AtTabInfoTree>("E12014")->GetTree();
   // Find dE Range
   tree->Draw("(11.17)*(MUSIC.fEnergy[0]+MUSIC.fEnergy[1]+MUSIC.fEnergy[2]+MUSIC.fEnergy[3]+MUSIC.fEnergy[4]+MUSIC."
              "fEnergy[5]+MUSIC.fEnergy[6]+MUSIC.fEnergy[7]+MUSIC.fEnergy[8])>>temp(1000, 0, 500)",
              "", "goff");
   double mean = ((TH2D *)gROOT->FindObject("temp"))->GetMean();
   double sd = ((TH2D *)gROOT->FindObject("temp"))->GetStdDev();
   tree->Draw(
      TString::Format("(11.17)*(MUSIC.fEnergy[0]+MUSIC.fEnergy[1]+MUSIC.fEnergy[2]+MUSIC.fEnergy[3]+MUSIC.fEnergy[4]+"
                      "MUSIC.fEnergy[5]+MUSIC.fEnergy[6]+MUSIC.fEnergy[7]+MUSIC.fEnergy[8])>>temp2(1000, %g, %g)",
                      mean - 2 * sd, mean + 4 * sd),
      "", "goff");
   mean = ((TH2D *)gROOT->FindObject("temp2"))->GetMean();
   sd = ((TH2D *)gROOT->FindObject("temp2"))->GetStdDev();

   // Plot PID
   gStyle->SetNumberContours(20);
   tree->Draw(TString::Format("(11.17)*(MUSIC.fEnergy[0]+MUSIC.fEnergy[1]+MUSIC.fEnergy[2]+MUSIC.fEnergy[3]+MUSIC."
                              "fEnergy[4]+MUSIC.fEnergy[5]+MUSIC.fEnergy[6]+MUSIC.fEnergy[7]+MUSIC.fEnergy[8]):((DSMCP."
                              "fTimeAnode[0] - USMCP.fTimeAnode[0])+728)>>IC PID(101, 286.7, 296.7, 500, %g, %g)",
                              (mean - 3 * sd), (mean + 3 * sd)),
              "", "colz");
   TH1F *hist = (TH1F *)gDirectory->GetList()->FindObject("IC PID");
   hist->GetYaxis()->SetTitle("dE - Ion Chamber");
   hist->GetYaxis()->CenterTitle(true);
   hist->GetXaxis()->SetTitle("TOF (ns)");
   hist->GetXaxis()->CenterTitle(true);
}
