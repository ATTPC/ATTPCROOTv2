
set(TEST_SRC
  AtBaseEventTest.cxx
)

attpcroot_generate_tests(AtDataTests
  SRCS ${TEST_SRC}
  DEPS AtData
  )