// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Our Crescendo shooter Subsystem */
public class Shooter extends SubsystemBase {
  private final PIDController m_pid;
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry log_pid_output = new DoubleLogEntry(m_log, "/shooter/pid/output");

  public final Trigger m_atSetpoint;

  /**
   * Creates a new {@link Shooter} {@link edu.wpi.first.wpilibj2.command.Subsystem}.
   *
   * @param motor The motor that the shooter controls.
   */
  public Shooter() {
    m_motor = new CANSparkMax(Constants.shooterPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pid = new PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD);
    m_atSetpoint = new Trigger(m_pid::atSetpoint);
  }

  /** Acheives and maintains speed. */
  private Command achieveSpeeds(double speed) {
    m_pid.reset();
    m_pid.setSetpoint(speed);
    return Commands.run(
        () -> {
          var output =
              m_pid.calculate(
                  -1 * Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity()));
          log_pid_output.append(output);
          m_motor.setVoltage(output);
        });
  }

  /**
   * Speeds up the shooter until it hits the specified speed then stops.
   *
   * @param speed The desired speed in Radians per second.
   * @return a {@link Command} to get to the desired speed.
   */
  public Command spinup(double speed) {
    LED.Spinup();
    return achieveSpeeds(speed).until(m_pid::atSetpoint);
  }

  public Command stop() {
    LED.Stop();
    return runOnce(() -> m_motor.set(0));
  }

  /**
   * Holds the shooter at the current speed setpoint.
   *
   * @return A {@link Command} to hold the speed at the setpoint.
   */
  public Command maintain() {
    LED.Maintain();
    return achieveSpeeds(m_pid.getSetpoint());
  }
}
