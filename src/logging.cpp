#include "logging.hpp"

std::shared_ptr<okapi::Logger> build_logger(bool competition, bool debug) {
  using namespace okapi;    // simplifies things

  std::shared_ptr<Logger> logger = std::make_shared<okapi::Logger> (
    okapi::TimeUtilFactory::createDefault().getTimer(),       // required timer
    competition ? "/usd/log.txt" : "/ser/sout",               // log to SD if competition
    debug ? Logger::LogLevel::debug : Logger::LogLevel::warn  // debug flag
  );

  return logger;
}
