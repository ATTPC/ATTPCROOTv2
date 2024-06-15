
set(TEST_SRC
  AtTPCIonDecayTest.cxx
  AtReactionGeneratorTest.cxx
)

attpcroot_generate_tests(AtGeneratorTests
  SRCS ${TEST_SRC}
  DEPS AtGenerators
  )