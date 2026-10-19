#include "AtMCFission.h"

#include "AtDummyMap.h"
#include "AtFissionEvent.h"
#include "AtPulseLine.h"

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

namespace {
/// Exposes the parameter setup of AtMCFission.
class TestMCFission : public MCFitter::AtMCFission {
public:
   using AtMCFission::AtMCFission;
   using AtMCFission::SetParamDistributions;
};
} // namespace

TEST(AtMCFissionTest, EventWithoutYPatternThrows)
{
   auto pulse = std::make_shared<AtPulseLine>(std::make_shared<AtDummyMap>(), [](int, double) { return 0.; });
   TestMCFission fitter(nullptr, nullptr, pulse);
   AtFissionEvent event; // No tracks, so no Y pattern
   EXPECT_THROW(fitter.SetParamDistributions(event), std::invalid_argument);
}
