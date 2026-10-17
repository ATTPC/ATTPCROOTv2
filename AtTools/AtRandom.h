#ifndef ATRANDOM_H
#define ATRANDOM_H

#include <TRandom3.h>

#include <cstdint>
#include <initializer_list>
#include <limits>

/**
 * Per-thread random number generators.
 *
 * Code that may run on a worker thread draws from Get() instead of gRandom. With no generator
 * installed on the calling thread Get() is *gRandom, so single-threaded code behaves exactly as if
 * it used gRandom directly. A ScopedGenerator installs a seeded generator for the calling thread.
 */
namespace AtTools::Random {

/// The generator installed on the calling thread, or *gRandom if there is none. A thread other than the main one must
/// install a ScopedGenerator, otherwise it shares gRandom with every other thread (the race this exists to avoid).
TRandom &Get();

/// Mix the values into a nonzero 32-bit seed for TRandom3 (zero would make TRandom3 seed randomly).
std::uint32_t MixSeed(std::initializer_list<std::uint64_t> values);

/**
 * Installs a TRandom3 with the given seed as the calling thread's generator for the lifetime of
 * this object, then restores the previous one. Construct and destroy it on the same thread.
 */
class ScopedGenerator {
   TRandom3 fRand;
   TRandom *fPrevious;

public:
   explicit ScopedGenerator(std::uint32_t seed);
   ~ScopedGenerator();
   ScopedGenerator(const ScopedGenerator &) = delete;
   ScopedGenerator &operator=(const ScopedGenerator &) = delete;
};

/// Adapts a TRandom to the standard UniformRandomBitGenerator interface, for use with <random>.
class UniformBitGenerator {
   TRandom &fRand;

public:
   using result_type = std::uint32_t;
   explicit UniformBitGenerator(TRandom &rand) : fRand(rand) {}
   static constexpr result_type min() { return 0; }
   static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }
   result_type operator()()
   {
      // Map Rndm(), in (0, 1) for TRandom3, onto the full range of result_type
      double val = fRand.Rndm() * (static_cast<double>(max()) + 1.0);
      return val >= max() ? max() : static_cast<result_type>(val);
   }
};

} // namespace AtTools::Random

#endif // ATRANDOM_H
