
set(TEST_SRC
  AtVertexPropagatorTest.cxx
)

attpcroot_generate_tests(AtSimulationDataTests
  SRCS ${TEST_SRC}
  DEPS AtSimulationData
  )