// lcd.hpp - header file for lcd.cpp

#ifndef _LCD_H_
#define _LCD_H_

#include "main.h"
#include "enums.h"

namespace lcd {
  // Functions
  void init();
  void display_mode(okapi::Controller, DRIVETRAIN_MODE);
  void display_mode(okapi::Controller, CONTROL_MODE);
  void display_battery_info(okapi::Controller);
}

#endif  // #ifndef _LCD_H_
