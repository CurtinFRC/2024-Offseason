// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Flex implements Auto {
  private final Command m_follower;

  public Flex(CommandSwerveDrivetrain drivetrain) {
    m_follower = drivetrain.followTrajectory("4-note-flex", false);
  }

  @Override
  public void configureBindings() {}

  @Override
  public Command followTrajectory() {
    return m_follower;
  }
}
