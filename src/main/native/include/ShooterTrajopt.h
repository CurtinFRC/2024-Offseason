// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include "wpi/array.h"

struct traj {
  double yaw;
  double pitch;
  double angular_velocity;
};

wpi::array<double, 3> calculate_trajectory(const double x_meter, const double y_meter, const double vel_x,
                                           const double vel_y);
