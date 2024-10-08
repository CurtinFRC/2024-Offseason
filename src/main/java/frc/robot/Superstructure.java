// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Superstructure {
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final Index m_index;

  public Superstructure(Shooter shooter, Intake intake, Index index) {
    m_shooter = shooter;
    m_intake = intake;
    m_index = index;
  }

  public Command intake() {
    return m_intake.intake(10).until(m_index.m_intaking);
  }

  public Command shoot() {
    // return m_shooter.applyVolts(12);
    return m_shooter
        .spinup(1500)
        .withTimeout(1.5)
        .andThen(Commands.parallel(m_shooter.maintain(), m_index.shoot()));
  }

  public Command outake_shooter() {
    return m_shooter.applyVolts(-6);
  }

  public Command stop() {
    return Commands.parallel(m_shooter.stop(), m_intake.stop(), m_index.stop());
  }

  public Command outake() {
    return Commands.parallel(m_intake.outake(-8), m_index.outake(8));
  }
}
