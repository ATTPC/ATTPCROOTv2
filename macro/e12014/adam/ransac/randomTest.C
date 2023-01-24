
std::vector<double> hits;
std::vector<AtHit> atHits;

TH1F *hRand = new TH1F("hRand", "Uniform", 100, 0, 99);
TH1F *hCharge = new TH1F("hCharge", "Charge", 100, 0, 99);
TH1F *hChargeCDF = new TH1F("hChargeCDF", "Charge CDF", 100, 0, 99);
TH1F *hDistanceCDF = new TH1F("hDistanceCDF", "Distance CDF", 100, -99, 99);

void randomTest()
{
   hits.clear();
   atHits.clear();
   for (int i = 0; i < 100; ++i) {
      hits.push_back(i);
      atHits.emplace_back(0, XYZPoint(0, 0, i), 100 - i); // Construct hit at z=i with Q=i
   }
}

class RandomSampleUniform : public RandomSample::AtIndependentSample {
public:
   std::vector<AtHit> SampleHits(int N, std::vector<int> vetoed)
   {
      // Using the sampled indices, return a vector of positions
      std::vector<AtHit> ret;
      for (auto ind : sampleIndicesFromCDF(N, vetoed))
         ret.push_back(fHits->at(ind));
      return ret;
   }
   std::vector<ROOT::Math::XYZPoint> SamplePoints(int N, std::vector<int> vetoed)
   {
      std::vector<ROOT::Math::XYZPoint> ret;
      for (const auto &hit : SampleHits(N, vetoed))
         ret.push_back(hit.GetPosition());
      return ret;
   }

protected:
   virtual std::vector<double> PDF(const AtHit &hit) override { return {1}; }
};

void randomSample(int N)
{
   RandomSample::AtUniform sampler;
   sampler.SetHitsToSample(&atHits);
   for (int i = 0; i < N; ++i) {
      auto pos = sampler.SamplePoints(1);
      hRand->Fill(pos[0].Z());
   }
   hRand->Draw();
}

void randomSampleVeto(int N, bool veto)
{
   RandomSampleUniform sampler;
   sampler.SetHitsToSample(&atHits);
   for (int i = 0; i < N; ++i) {
      if (veto) {
         auto pos = sampler.SamplePoints(1, {});
         hRand->Fill(pos[0].Z());
      } else {
         auto pos = sampler.SamplePoints(1, {10, 11, 12, 13, 14, 15});
         hRand->Fill(pos[0].Z());
      }
   }
   hRand->Draw();
}

void chargeSample(int N)
{
   RandomSample::AtChargeWeighted sampler;
   sampler.SetHitsToSample(&atHits);
   for (int i = 0; i < N; ++i) {
      auto pos = sampler.SamplePoints(1);
      hCharge->Fill(pos[0].Z());
   }
   hCharge->Draw();
}

void distanceSample(int N, double sigma = 20)
{
   RandomSample::AtGaussian sampler(sigma);
   sampler.SetHitsToSample(&atHits);
   /*   for (int i = 0; i < N; ++i) {
         auto pos = sampler.SampleHits(2);
         hDistanceCDF->Fill(std::abs((pos[0] - pos[1]).Z()));
         hDistanceCDF->Fill(-std::abs((pos[0] - pos[1]).Z()));
      }
   */
   auto pos = sampler.SamplePoints(N);
   auto &ref = sampler.GetReferenceHit();
   for (int i = 0; i < N; ++i) {
      hDistanceCDF->Fill(std::abs(pos[i].Z()));
      // hDistanceCDF->Fill(-std::abs((ref.GetPosition() - pos[i]).Z()));
   }

   hDistanceCDF->Draw();
}
void randomSampleOld(int N)
{

   for (int i = 0; i < N; ++i) {

      int ind = gRandom->Uniform() * hits.size();
      hRand->Fill(hits[ind]);
   }
   hRand->Draw();
}

void chargeSampleOld(int N)
{
   double qTotal = std::accumulate(hits.begin(), hits.end(), 0, [](double sum, double a) { return sum + a; });
   std::cout << "total charge: " << qTotal << endl;

   for (int i = 0; i < N; ++i) {

      int ind = gRandom->Uniform() * hits.size();
      double pdf = hits[ind] / *(std::max_element(hits.begin(), hits.end()));
      double r = gRandom->Uniform();
      if (pdf > r)
         hCharge->Fill(hits[ind]);
   }
   hCharge->Draw();
}

void chargeCDF(int N)
{
   double qTotal = std::accumulate(hits.begin(), hits.end(), 0, [](double sum, double a) { return sum + a; });

   std::vector<double> CDF;

   for (auto q : hits) {
      CDF.push_back(CDF.back() + q / qTotal);
      std::cout << q << " " << q / qTotal << " " << CDF.back() << endl;
   }

   for (int i = 0; i < N; ++i) {
      double r = gRandom->Uniform();
      auto pos = std::upper_bound(CDF.begin(), CDF.end(), r);
      int index = std::distance(CDF.begin(), pos);
      std::cout << "r: " << r << " index: " << index << " CDF: " << CDF[index] << endl;
      hChargeCDF->Fill(hits[index]);
   }
   hChargeCDF->Draw();
}

void distanceCDF(int N, double sigma = 20)
{
   std::vector<double> CDF;

   for (auto dist : hits) {
      if (CDF.size() == 0)
         CDF.push_back(ROOT::Math::gaussian_pdf(dist, sigma));
      else
         CDF.push_back(CDF.back() + ROOT::Math::gaussian_pdf(dist, sigma));
   }
   auto normalization = CDF.back();
   for (auto &elem : CDF)
      elem /= normalization;

   for (int i = 0; i < N; ++i) {
      double r = gRandom->Uniform();
      auto pos = std::upper_bound(CDF.begin(), CDF.end(), r);
      int index = std::distance(CDF.begin(), pos);
      std::cout << "r: " << r << " index: " << index << " CDF: " << CDF[index] << endl;
      hDistanceCDF->Fill(hits[index]);
   }
   hDistanceCDF->Draw();
}
