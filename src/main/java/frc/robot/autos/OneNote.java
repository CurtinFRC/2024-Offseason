// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.autos;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Auto;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OneNote implements Auto {
  Shooter m_shooter;
  Intake m_intake;

  public OneNote(Shooter shooter, Intake intake) {
    m_shooter = shooter;
    m_intake = intake;
  }

  @Override
  public Command followTrajectory() {
    return Commands.print("No path to follow");
  }

  @Override
  public void configureBindings() {
    Choreo.event("shoot")
        .onTrue(
            m_shooter
                .spinup(300)
                .andThen(Commands.parallel(m_intake.pass(), m_shooter.maintain())));
  }
}
