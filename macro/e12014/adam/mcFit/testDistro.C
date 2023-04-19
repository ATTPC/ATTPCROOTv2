TH1D *hist;
TH1D *hist2;

void testDistro()
{
   hist = new TH1D("uniform", "Uniform", 50, -50, 50);
   hist2 = new TH1D("student", "StudentT", 50, -50, 50);

   MCFitter::AtUniformDistribution dist(10, 25);
   MCFitter::AtStudentDistribution distT(10, 5);

   cout << distT.Sample() << endl;

   for (int i = 0; i < 1e4; ++i) {
      hist->Fill(dist.Sample());
      hist2->Fill(distT.Sample());
   }

   hist2->Draw("same");
   hist->Draw("same");
}
