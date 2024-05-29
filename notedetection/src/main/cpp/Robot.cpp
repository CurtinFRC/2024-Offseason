// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

#include <iostream>

Robot::Robot() {}

void Robot::RobotPeriodic() {
  if (!camerastream.open(0)) {
    std::cerr << "Couldn't open camera stream\n";
    return;
  }

  camerastream.read(currentframe);

  if (currentframe.empty()) {
    std::cerr << "Read empty frame\n";
    return;
  }

  cv::imshow("Current Frame", currentframe);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
