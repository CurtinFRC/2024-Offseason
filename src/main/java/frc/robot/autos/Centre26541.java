// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Centre26541 implements Auto {
  private Command m_follower;

  public Centre26541(CommandSwerveDrivetrain drivetrain, boolean isRed) {
    m_follower = drivetrain.followTrajectory("Centre2_6_5_4_1", isRed);
  }

  @Override
  public void configureBindings() {}

  @Override
  public Command followTrajectory() {
    return m_follower;
  }
}
