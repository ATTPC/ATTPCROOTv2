#include "AtRandom.h"

#include <TRandom.h>

#include <gtest/gtest.h>
#include <random>
#include <thread>
#include <vector>

using namespace AtTools::Random;

static std::vector<double> Draw(TRandom &rand, int n = 5)
{
   std::vector<double> vals;
   for (int i = 0; i < n; ++i)
      vals.push_back(rand.Gaus(0, 1));
   return vals;
}

TEST(AtRandomTest, FallsBackToGRandom)
{
   EXPECT_EQ(&Get(), gRandom);
}

TEST(AtRandomTest, ScopedGeneratorInstallsAndRestores)
{
   {
      ScopedGenerator outer(1);
      TRandom *outerRand = &Get();
      EXPECT_NE(outerRand, gRandom);
      {
         ScopedGenerator inner(2);
         EXPECT_NE(&Get(), outerRand);
      }
      EXPECT_EQ(&Get(), outerRand);
   }
   EXPECT_EQ(&Get(), gRandom);
}

TEST(AtRandomTest, SameSeedSameSequence)
{
   std::vector<double> first, second, other;
   {
      ScopedGenerator gen(42);
      first = Draw(Get());
   }
   {
      ScopedGenerator gen(42);
      second = Draw(Get());
   }
   {
      ScopedGenerator gen(43);
      other = Draw(Get());
   }
   EXPECT_EQ(first, second);
   EXPECT_NE(first, other);
}

TEST(AtRandomTest, GeneratorIsPerThread)
{
   ScopedGenerator gen(7);
   TRandom *mainRand = &Get();
   TRandom *threadRand = nullptr;
   std::thread th([&threadRand] { threadRand = &Get(); });
   th.join();
   EXPECT_EQ(threadRand, gRandom);
   EXPECT_EQ(&Get(), mainRand);
}

TEST(AtRandomTest, MixSeed)
{
   EXPECT_EQ(MixSeed({1, 2, 3}), MixSeed({1, 2, 3}));
   EXPECT_NE(MixSeed({1, 2, 3}), MixSeed({1, 2, 4}));
   EXPECT_NE(MixSeed({1, 2, 3}), MixSeed({1, 3, 2}));
   EXPECT_NE(MixSeed({0}), 0U);
}

TEST(AtRandomTest, UniformBitGeneratorWorksWithStdDistributions)
{
   std::student_t_distribution<> distro{1};
   std::vector<double> first, second;
   {
      ScopedGenerator gen(5);
      UniformBitGenerator bitGen(Get());
      for (int i = 0; i < 5; ++i)
         first.push_back(distro(bitGen));
   }
   distro.reset();
   {
      ScopedGenerator gen(5);
      UniformBitGenerator bitGen(Get());
      for (int i = 0; i < 5; ++i)
         second.push_back(distro(bitGen));
   }
   EXPECT_EQ(first, second);

   std::uniform_real_distribution<> uni{-1, 1};
   UniformBitGenerator bitGen(Get());
   for (int i = 0; i < 1000; ++i) {
      auto val = uni(bitGen);
      EXPECT_GE(val, -1);
      EXPECT_LT(val, 1);
   }
}
