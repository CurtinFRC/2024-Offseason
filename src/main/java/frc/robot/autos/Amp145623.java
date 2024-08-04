// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Amp145623 implements Auto {
  private final Command m_followerCommand;

  public Amp145623(CommandSwerveDrivetrain drivetrain, boolean isRed) {
    m_followerCommand = drivetrain.followTrajectory("Amp1_4_5_6_23", isRed);
  }

  @Override
  public void configureBindings() {}

  @Override
  public Command followTrajectory() {
    return m_followerCommand;
  }
}
