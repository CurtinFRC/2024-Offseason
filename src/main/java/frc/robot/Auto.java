// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface Auto {
  public void configureBindings();

  public Command followTrajectory();
}
