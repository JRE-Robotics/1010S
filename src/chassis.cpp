#include "chassis.hpp"

std::shared_ptr<okapi::OdomChassisController> build_chassis_controller() {
  using namespace okapi;    // simplifies things

  std::shared_ptr<OdomChassisController> cc = ChassisControllerBuilder()
    // Right motors reversed
    .withMotors(
      {LEFT_FRONT_MOTOR_PORT, LEFT_BACK_MOTOR_PORT},
      {-RIGHT_FRONT_MOTOR_PORT, -RIGHT_BACK_MOTOR_PORT}
    )
    // Green gears + 2.75" wheel ⌀, 6.5" wheelbase
    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 6.5_in}, imev5GreenTPR})
    // Enable odometry
    .withOdometry(StateMode::CARTESIAN)
    .buildOdometry();

  // Reset odom state
  cc->setState({0_in, 0_in, 0_deg});

  return cc;
}
