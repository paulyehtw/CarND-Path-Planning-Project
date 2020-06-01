#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <stdint.h>

static const float kAccelerationLimit = 10.0F;

namespace Road
{
    static const float kTrackLength = 6945.554; // meters
    static const float kWidth = 4.0F;           // meter
    static const float kSpeedLimit = 21.5;      // m/s
    static const float kIntervalS = 30;         // m
} // namespace Road

namespace Planner
{
    static const uint8_t kNumWaypointsBehind = 5;
    static const uint8_t kNumWaypointsAhead = 5;
    static const uint8_t kNumKeptPreviousPath = 25;
    static const uint8_t kNumPathPoints = 50;
    static const uint8_t kNumSamples = 20;
    static const float kInterpolatedWaypointsInterval = 0.5F; // meter
    static const float dt = 0.02F;                            // second
    static const float kSampleDt = 0.2F;                      // second
    static const float kSaveDistance = 15.0F;                 // meter
    static const float kCollisionBuffer = 2.5F;               // meter
} // namespace Planner

namespace Cost
{
    static const float kVelocityIncrementLimit = 0.125F; // m/s
    static const float kCollisionCost = 100000;
    static const float kNoneMiddleLaneCost = 100;
    static const float kEfficiencyCost = 10000;
    static const float kLeadingCarCost = 1000;
} // namespace Cost

#endif // PARAMETERS_H