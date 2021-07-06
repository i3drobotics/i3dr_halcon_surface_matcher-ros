#include "halconmatcher/halconmatcher.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class Foo.
class HalconMatcherTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  HalconMatcherTest() {
    // You can do set-up work for each test here.
  }

  virtual ~HalconMatcherTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(HalconMatcherTest, LoadPointCloud) {
  const std::string reference_model = "./reference_model.stl";
  const std::string test_scene = "./test_scene.ply";
  HalconMatcher matcher;
  matcher.init_model(reference_model);
  matcher.load_scene(test_scene);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}