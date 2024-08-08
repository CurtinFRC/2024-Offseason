// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import com.choreo.lib.Auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Amp123_8_ implements Auto {
  private final Command m_follower;

  public Amp123_8_(CommandSwerveDrivetrain drivetrain, boolean isRed) {
    m_follower = drivetrain.followTrajectory("Centre2_6_5_4_1", isRed);
  }

  @Override
  public void configureBindings() {}

  @Override
  public Command followTrajectory() {
    configureBindings();
    return m_follower;
  }
}