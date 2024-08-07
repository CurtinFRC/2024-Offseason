// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import com.choreo.lib.Auto;
import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TwoNote implements Auto {
  Shooter m_shooter;
  // Intake m_intake;

  Command m_pathfollower;

  public TwoNote(
      // CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, boolean isRed) {
      CommandSwerveDrivetrain drivetrain, Shooter shooter, boolean isRed) {
    m_shooter = shooter;
    // m_intake = intake;

    m_pathfollower = drivetrain.followTrajectory("example-two-note", isRed);
  }

  @Override
  public Command followTrajectory() {
    configureBindings();
    return m_pathfollower;
  }

  private Command shoot() {
    return m_shooter
        .spinup(500)
        .andThen(m_shooter.maintain())
        .withTimeout(1)
        .andThen(m_shooter.stop());
  }

  @Override
  public void configureBindings() {
    Choreo.event("first-shot").onTrue(shoot());
    // Choreo.event("intake").onTrue(m_intake.spinUntilBeamBreak(200));
    Choreo.event("second-shot").onTrue(shoot());
  }
}
