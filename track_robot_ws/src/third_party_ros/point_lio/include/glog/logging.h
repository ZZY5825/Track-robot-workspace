#pragma once

#include <iostream>

namespace google
{
inline void InitGoogleLogging(const char *) {}
}  // namespace google

enum GLogCompatSeverity
{
  INFO,
  WARNING,
  ERROR,
  FATAL
};

#define LOG(level) std::cerr
#define VLOG(level) if (true) {} else std::cerr
#define CHECK(condition) if (condition) {} else std::cerr
