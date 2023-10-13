/*
 * time.hpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#pragma once
#include <chrono>
#include <ostream>
#include <ratio>
#include "icp_localization/common/typedefs.hpp"
#include <ros/time.h>


namespace icp_loco{

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration fromSeconds(double seconds);
Duration fromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double toSeconds(Duration duration);
double toSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
Time fromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64 toUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
std::ostream& operator<<(std::ostream& os, Time time);

std::string toString(const Time &time);

ros::Time toRos(Time time);

Time fromRos(const ::ros::Time& time);

double readable(const Time &t);

} //namespace icp_loco
