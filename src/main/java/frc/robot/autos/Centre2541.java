// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import com.choreo.lib.Auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Centre2541 implements Auto {
  private final Command m_followerCommand;

  public Centre2541(CommandSwerveDrivetrain drivetrain, boolean isRed) {
    m_followerCommand = drivetrain.followTrajectory("Centre2_5_4_1", isRed);
  }

  @Override
  public void configureBindings() {}

  @Override
  public Command followTrajectory() {
    configureBindings();
    return m_followerCommand;
  }
}
