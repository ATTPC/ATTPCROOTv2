
set(TEST_SRC
  AtTPCIonDecayTest.cxx
)

attpcroot_generate_tests(AtGeneratorTests
  SRCS ${TEST_SRC}
  DEPS AtGenerators
  )