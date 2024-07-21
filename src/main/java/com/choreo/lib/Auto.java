// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package com.choreo.lib;

import edu.wpi.first.wpilibj2.command.Command;

public interface Auto {
  /** A function to setup Triggers for the Auto routine. */
  public void configureBindings();

  /**
   * The Command to follow the trajectory, should be your autonomous command.
   *
   * <p>Should call the {@link configureBindings} method.
   *
   * @return The trajectory following command.
   */
  public Command followTrajectory();
}
