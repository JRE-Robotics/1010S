// logging.hpp - header file for logging.cpp

#ifndef _LOGGING_H_
#define _LOGGING_H_

#include "main.h"

// Functions
std::shared_ptr<okapi::Logger> build_logger(bool competition, bool debug);

#endif  // #ifndef _LOGGING_H_
