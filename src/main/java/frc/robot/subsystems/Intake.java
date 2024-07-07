// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final DoubleLogEntry log_output =
      new DoubleLogEntry(DataLogManager.getLog(), "/intake/output");

  PWM lEDPwm = new PWM(0);

  public Intake() {
    m_motor = new CANSparkMax(Constants.intakePort, MotorType.kBrushless);
    lEDPwm = new PWM(0);
  }

  public Command intake() {
    lEDPwm.setSpeed(0.59); // dark red //
    lEDPwm.setSpeed(0.59); // dark red //
    return Commands.run(
            () -> {
              log_output.append(4);
              m_motor.setVoltage(4);
            })
        .withTimeout(2)
        .andThen(runOnce(() -> m_motor.setVoltage(0)));
  }

  public Command stop() {
    return runOnce(() -> m_motor.set(0));
  }

  public Command outake() {
    lEDPwm.setSpeed(0.63); // red orange, 0.65 for normal orange //
    lEDPwm.setSpeed(0.63); // red orange, 0.65 for normal orange //
    return Commands.run(
            () -> {
              log_output.append(-4);
              m_motor.setVoltage(-4);
            })
        .withTimeout(4)
        .andThen(runOnce(() -> m_motor.setVoltage(0)));
  }

  public Command pass() {
    return intake();
  }
}
