#include "AtPulse.h"

#include <Math/SpecFuncMathCore.h> // for tgamma
#include <TF1.h>
#include <TRandom3.h>

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

namespace {
constexpr int nSamples = 100000;
constexpr double meanGain = 2000;
constexpr double polyaB = 1;

struct Stats {
   double mean{};
   double var{};
};

Stats ComputeStats(const std::vector<double> &vals)
{
   Stats s;
   for (auto v : vals)
      s.mean += v;
   s.mean /= vals.size();
   for (auto v : vals)
      s.var += (v - s.mean) * (v - s.mean);
   s.var /= (vals.size() - 1);
   return s;
}

/// Two-sample Kolmogorov-Smirnov statistic (max distance between the empirical CDFs).
double KSStatistic(std::vector<double> a, std::vector<double> b)
{
   std::sort(a.begin(), a.end());
   std::sort(b.begin(), b.end());
   size_t i = 0, j = 0;
   double d = 0;
   while (i < a.size() && j < b.size()) {
      double x = std::min(a[i], b[j]);
      while (i < a.size() && a[i] <= x)
         ++i;
      while (j < b.size() && b[j] <= x)
         ++j;
      d = std::max(d, std::fabs(double(i) / a.size() - double(j) / b.size()));
   }
   return d;
}

/// The Polya gain function that AtPulse used to sample one electron at a time with TF1::GetRandom.
TF1 MakeOldGainFunc()
{
   TF1 func("gain", "pow([1]+1,[1]+1)/ROOT::Math::tgamma([1]+1)*pow((x/[0]),[1])*exp(-([1]+1)*(x/[0]))", 0,
            meanGain * 5);
   func.SetParameter(0, meanGain);
   func.SetParameter(1, polyaB);
   return func;
}
} // namespace

TEST(AtPulseTest, MeanPolyaGainMatchesPerElectronLoop)
{
   auto func = MakeOldGainFunc();
   TRandom3 oldRand(1);
   TRandom3 newRand(2);

   for (int n : {1, 2, 5, 8, 10}) {
      std::vector<double> oldVals(nSamples), newVals(nSamples);
      for (auto &val : oldVals) {
         double g = 0;
         for (int i = 0; i < n; ++i)
            g += func.GetRandom(&oldRand);
         val = g / n;
      }
      for (auto &val : newVals)
         val = AtPulse::MeanPolyaGain(meanGain, polyaB, n, newRand);

      auto oldStats = ComputeStats(oldVals);
      auto newStats = ComputeStats(newVals);
      double ks = KSStatistic(oldVals, newVals);
      double ksCrit = 1.95 * std::sqrt(2.0 / nSamples); // about p = 0.001
      std::cout << "N=" << n << " old mean " << oldStats.mean << " var " << oldStats.var << " | new mean "
                << newStats.mean << " var " << newStats.var << " | exact mean " << meanGain << " var "
                << meanGain * meanGain / (n * (polyaB + 1)) << " | KS D " << ks << " (crit " << ksCrit << ")\n";

      EXPECT_NEAR(newStats.mean, oldStats.mean, 0.01 * meanGain);
      // The old TF1 is cut off at 5 * gain and interpolated from a table, which lowers its mean by about 0.2% and its
      // variance by about 2% for one electron; the sampling error of the variance of 1e5 samples is about 1%.
      EXPECT_NEAR(newStats.var / oldStats.var, 1, 0.06);
      EXPECT_LT(ks, ksCrit);
   }
}

TEST(AtPulseTest, MeanPolyaGainHasExactMoments)
{
   TRandom3 rand(3);
   for (int n : {1, 2, 5, 8, 9, 10, 50}) { // Erlang path up to 8 electrons, std::gamma_distribution above
      std::vector<double> vals(nSamples);
      for (auto &val : vals)
         val = AtPulse::MeanPolyaGain(meanGain, polyaB, n, rand);
      auto stats = ComputeStats(vals);
      double expectedVar = meanGain * meanGain / (n * (polyaB + 1));
      double meanError = meanGain / std::sqrt(n * (polyaB + 1.) * nSamples); // Standard error of the mean
      EXPECT_NEAR(stats.mean, meanGain, 5 * meanError) << "N = " << n;
      EXPECT_NEAR(stats.var / expectedVar, 1, 0.03) << "N = " << n;
   }
}

TEST(AtPulseTest, MeanPolyaGainDrawsFromGivenGenerator)
{
   TRandom3 a(5);
   TRandom3 b(5);
   for (int i = 0; i < 10; ++i)
      EXPECT_EQ(AtPulse::MeanPolyaGain(meanGain, polyaB, 3, a), AtPulse::MeanPolyaGain(meanGain, polyaB, 3, b));
}
