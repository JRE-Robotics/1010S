#include "lcd.hpp"

namespace lcd {
  // Constants for display
  const std::string VERSION = "1010S v0.1";

  const std::string DT_MODE_FAST = "DT: Fast";
  const std::string DT_MODE_SLOW = "DT: Slow";

  const std::string CTRL_MODE_ARCADE = "CTRL: Arcade";
  const std::string CTRL_MODE_TANK = "CTRL: Tank  ";

  void init() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, VERSION);
  }

  // Overloaded display_mode() for drivetrain mode
  void display_mode(okapi::Controller controller, DRIVETRAIN_MODE mode) {
    pros::delay(50);

    switch (mode) {
      case FAST:
        controller.setText(1, 0, DT_MODE_FAST); break;
      case SLOW:
        controller.setText(1, 0, DT_MODE_SLOW); break;
    }
  }

  // Overloaded display_mode() for control mode
  void display_mode(okapi::Controller controller, CONTROL_MODE mode) {
    pros::delay(50);

    switch (mode) {
      case ARCADE:
        controller.setText(2, 0, CTRL_MODE_ARCADE); break;
      case TANK:
        controller.setText(2, 0, CTRL_MODE_TANK); break;
    }
  }

  // Display battery info on brain and controller
  void display_battery_info(okapi::Controller controller) {
    char buf[19];
    double capacity = pros::battery::get_capacity();
    double current = pros::battery::get_current() / 1000.0;

    snprintf(buf, sizeof(buf), "BAT: %.1f%% | %.2fA", capacity, current);
    controller.setText(0, 0, buf);
    pros::lcd::set_text(3, buf);
  }
}
