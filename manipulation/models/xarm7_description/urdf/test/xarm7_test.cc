#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that a model can be loaded into a MultibodyPlant. This unit test also
// verifies that the URDF can be parsed by the URDF parser (similarly for the
// remaining tests in this file).
GTEST_TEST(XArmTest, TestLoad) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/xarm7_description/urdf/"
      "xarm7_with_gripper.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);

  EXPECT_EQ(plant.num_actuators(), 8);
  EXPECT_EQ(plant.num_bodies(), 16);
}


}  // namespace
}  // namespace manipulation
}  // namespace drake
