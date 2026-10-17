#include "AtRandom.h"

#include <TRandom.h>

namespace AtTools::Random {

namespace {
thread_local TRandom *tlRand = nullptr;

std::uint64_t SplitMix64(std::uint64_t x)
{
   x += 0x9E3779B97F4A7C15ULL;
   x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
   x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
   return x ^ (x >> 31);
}
} // namespace

TRandom &Get()
{
   return tlRand ? *tlRand : *gRandom;
}

std::uint32_t MixSeed(std::initializer_list<std::uint64_t> values)
{
   std::uint64_t hash = 0;
   for (auto val : values)
      hash = SplitMix64(hash ^ val);
   auto seed = static_cast<std::uint32_t>(hash ^ (hash >> 32));
   return seed == 0 ? 1 : seed;
}

ScopedGenerator::ScopedGenerator(std::uint32_t seed) : fRand(seed), fPrevious(tlRand)
{
   tlRand = &fRand;
}

ScopedGenerator::~ScopedGenerator()
{
   tlRand = fPrevious;
}

} // namespace AtTools::Random
