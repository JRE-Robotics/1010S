#include "chassis.hpp"

std::shared_ptr<okapi::ChassisController> build_chassis_controller() {
  using namespace okapi;    // simplifies things

  std::shared_ptr<ChassisController> cc = ChassisControllerBuilder()
    // Right motors reversed
    .withMotors(
      {LEFT_FRONT_MOTOR_PORT, LEFT_BACK_MOTOR_PORT},
      {-RIGHT_FRONT_MOTOR_PORT, -RIGHT_BACK_MOTOR_PORT}
    )
    // Green gears + 2.75" wheel ⌀, 6.5" wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{3.5_in, 6.5_in}, imev5GreenTPR})
    // Enable odometry
    .withOdometry()
    .buildOdometry();

  return cc;
}
