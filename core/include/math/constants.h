#ifndef MATH_CONSTANTS_H_
#define MATH_CONSTANTS_H_

namespace math {
constexpr double kGravity = 9.80665;
constexpr float kGravityFloat = 9.80665f;

constexpr double kPi = 3.14159265358979323846;
constexpr float kPiFloat = 3.14159265358979323846f;
constexpr double k2Pi = kPi * 2.0;
constexpr float k2PiFloat = kPiFloat * 2.0f;

constexpr double kRadiansPerDegree = 0.01745329251994329577;
constexpr float kRadiansPerDegreeFloat = 0.01745329251994329577f;

constexpr double kDegreesPerRadian = 57.29577951308232087680;
constexpr float kDegreesPerRadianFloat = 57.29577951308232087680f;

constexpr int kSecondToMilliSecond = 1000;
constexpr double kNanoSecondToSecond = 1e-9;
constexpr double kMilliSecondToSecond = 1e-3;

constexpr int kWorldDimensions = 3;
constexpr int kQuaternionDimensions = 4;
}  // namespace math

#endif  // MATH_CONSTANTS_H_
