#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#pragma once

#ifndef CONSTANTS_H
#define CONSTANTS_H


const int FRONT_LEFT = 0;
const int FRONT_RIGHT = 1;
const int BACK_LEFT = 2;
const int BACK_RIGHT = 3;

const int FRONT_LEFT_ENCODER[] = {0, 1};
const int FRONT_RIGHT_ENCODER[] = {2, 3};
const int BACK_LEFT_ENCODER[] = {4, 5};
const int BACK_RIGHT_ENCODER[] = {6, 7};

const int GYRO = 0;

const frc::Translation2d FL{.254_m, .305_m};
const frc::Translation2d FR{.254_m, -.305_m};
const frc::Translation2d BL{-.254_m, .305_m};
const frc::Translation2d BR{-.254_m, -.305_m};
const double WIDTH = 0.508;

const units::meters_per_second_t MAX_SPEED{3};
const units::meters_per_second_squared_t MAX_ACCEL{1.5};

const units::radians_per_second_t MAX_ROT_SPEED{2};
const frc::DifferentialDriveKinematics differentialKinematics{units::meter_t{WIDTH}};
const frc::MecanumDriveKinematics kinematics{FL, FR, BL, BR};

const auto kS = 0.22_V;
const auto kV = 4.25 * 1_V * 1_s / 1_m;
const auto kA = 0.2 * 1_V * 1_s * 1_s / 1_m;

const double ramseteB = 2;
const double ramseteZeta = 0.7;

#endif