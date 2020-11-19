#include "main.h"
#include "pros/misc.h"

#include "chassis.hpp"
#include "logging.hpp"
#include "lcd.hpp"
#include "ports.h"
#include "enums.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Init logger in non-competition mode
  okapi::Logger::setDefaultLogger(build_logger(false, false));
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // Override logger with competition mode
  okapi::Logger::setDefaultLogger(build_logger(true, false));
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
  // Init chassis controller and set brake mode
  std::shared_ptr<okapi::ChassisController> chassis = build_chassis_controller();
  chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  chassis->setMaxVelocity(50);
  chassis->moveDistance(0.1_m);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // Init chassis controller and V5 controller
  std::shared_ptr<okapi::ChassisController> chassis = build_chassis_controller();
  okapi::Controller controller;

  // Default modes
  DRIVETRAIN_MODE dt_mode = FAST;
  CONTROL_MODE ctrl_mode = ARCADE;

  // Set brake mode
  chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  // Initialize LCD
  lcd::init();
  lcd::display_mode(controller, dt_mode);
  lcd::display_mode(controller, ctrl_mode);

  int count = 0;    // controller LCD update timer

  // Main loop
  while (true) {
    // ----------
    // Buttons
    // ----------

    // Switch drivetrain mode
    if (controller.getDigital(okapi::ControllerDigital::Y)) {
      pros::delay(50);
      if (controller.getDigital(okapi::ControllerDigital::Y)) {
        switch (dt_mode) {
          case FAST:
            dt_mode = SLOW; break;
          case SLOW:
            dt_mode = FAST; break;
        }
        lcd::display_mode(controller, dt_mode);
      }
    }

    // Switch control mode
    if (controller.getDigital(okapi::ControllerDigital::B)) {
      pros::delay(50);
      if (controller.getDigital(okapi::ControllerDigital::B)) {
        switch (ctrl_mode) {
          case ARCADE:
            ctrl_mode = TANK; break;
          case TANK:
            ctrl_mode = ARCADE; break;
        }
        lcd::display_mode(controller, ctrl_mode);
      }
    }

    // Manual auton
    if (controller.getDigital(okapi::ControllerDigital::A)) {
      pros::delay(50);
      if (controller.getDigital(okapi::ControllerDigital::A)) {
        autonomous();
      }
    }

    // ----------
    // Drive
    // ----------

    // Arcade drive
    if (ctrl_mode == ARCADE) {
      float y = controller.getAnalog(okapi::ControllerAnalog::leftY);
      float x = controller.getAnalog(okapi::ControllerAnalog::leftX);

      double forward = (dt_mode == FAST) ? y : y / 4.0;
      double yaw = (dt_mode == FAST) ? x / 1.5 : x / 4.0;

      chassis->getModel()->arcade(forward, yaw, 0.15);
    }

    // Tank drive
    else if (ctrl_mode == TANK) {
      float left_y = controller.getAnalog(okapi::ControllerAnalog::leftY);
      float right_y = controller.getAnalog(okapi::ControllerAnalog::rightY);

      double left = (dt_mode == FAST) ? left_y : left_y / 4.0;
      double right = (dt_mode == FAST) ? right_y : right_y / 4.0;

      chassis->getModel()->tank(left, right);
    }

    // ----------
    // Misc.
    // ----------

    // Report battery leveL
    if ((count % 25) == 0) {
      lcd::display_battery_info(controller);
    }

    count++;          // Increment counter for controller LCD
    pros::delay(10);  // Loop delay
  }
}
