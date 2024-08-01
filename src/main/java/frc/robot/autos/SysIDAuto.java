// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import com.choreo.lib.Auto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sysid;

public class SysIDAuto implements Auto {
  private final Command m_follower;

  public SysIDAuto(Sysid sysid) {
    m_follower = sysid.getAllCommands();
  }

  @Override
  public void configureBindings() {}

  @Override
  public Command followTrajectory() {
    configureBindings();
    return m_follower;
  }
}
