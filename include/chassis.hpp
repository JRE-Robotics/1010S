// chassis.hpp - header file for chassis.cpp

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "ports.h"

// Functions
std::shared_ptr<okapi::OdomChassisController> build_chassis_controller();

#endif  // #ifndef _CHASSIS_H_
