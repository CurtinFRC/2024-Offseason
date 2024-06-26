// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto;
import frc.robot.FollowTrajectory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TwoNote implements Auto {
  Shooter m_shooter;
  Intake m_intake;
  CommandSwerveDrivetrain m_drivetrain;

  FollowTrajectory m_pathfollower;

  public TwoNote(
      CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, boolean isRed) {
    m_shooter = shooter;
    m_intake = intake;
    m_drivetrain = drivetrain;

    m_pathfollower = drivetrain.followTrajectory("example-two-note", isRed);
  }

  public Command followTrajectory() {
    return m_pathfollower;
  }

  private Command shoot() {
    return m_shooter
        .spinup(500)
        .andThen(m_shooter.maintain())
        .withTimeout(1)
        .andThen(m_shooter.stop());
  }

  public void configureBindings() {
    m_pathfollower.event("first-shot").onTrue(shoot());
    m_pathfollower.event("intake").onTrue(m_intake.intake());
    m_pathfollower.event("second-shot").onTrue(shoot());
  }
}
