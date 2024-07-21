// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** Our Arm Subsystem */
public class Arm extends SubsystemBase {
  public enum Setpoint {
    kAmp,
    kIntake,
    kSpeaker,
    kStowed
  }

  private final PIDController m_pid;
  private final CANSparkMax m_primaryMotor;
  private final CANSparkMax m_followerMotor;
  private final DutyCycleEncoder m_encoder;
  private final ArmFeedforward m_feedforward;
  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry log_pid_output = new DoubleLogEntry(m_log, "/arm/pid/output");
  private final DoubleLogEntry log_pid_setpoint = new DoubleLogEntry(m_log, "/arm/pid/setpoint");
  private final DoubleLogEntry log_ff_position_setpoint =
      new DoubleLogEntry(m_log, "/arm/ff/position_setpoint");
  private final DoubleLogEntry log_ff_velocity_setpoint =
      new DoubleLogEntry(m_log, "/arm/ff/velocity_setpoint");
  private final DoubleLogEntry log_ff_output = new DoubleLogEntry(m_log, "/arm/ff/output");
  private final StringLogEntry log_setpoint = new StringLogEntry(m_log, "/arm/setpoint");

  public final Trigger m_atSetpoint;

  /**
   * Creates a new {@link Arm} {@link edu.wpi.first.wpilibj2.command.Subsystem}.
   *
   * @param primaryMotor The primary motor that controls the arm.
   */
  public Arm() {
    m_primaryMotor = new CANSparkMax(Constants.armLeadPort, MotorType.kBrushless);
    m_followerMotor = new CANSparkMax(Constants.armFollowerPort, MotorType.kBrushless);
    m_followerMotor.follow(m_primaryMotor);

    m_encoder = new DutyCycleEncoder(Constants.armEncoderPort);
    m_pid = new PIDController(Constants.armP, Constants.armI, Constants.armD);
    m_pid.setTolerance(0.2);
    m_feedforward =
        new ArmFeedforward(Constants.armS, Constants.armG, Constants.armV, Constants.armA);

    m_atSetpoint = new Trigger(m_pid::atSetpoint);
  }

  /** Achieves and maintains speed for the primary motor. */
  private Command achievePosition(double position) {
    return Commands.run(
        () -> {
          var pid_output = m_pid.calculate(m_encoder.getAbsolutePosition() * 2 * Math.PI, position);
          log_pid_output.append(pid_output);
          log_pid_setpoint.append(m_pid.getSetpoint());
          var ff_output = m_feedforward.calculate(position, (5676 / 250));
          log_ff_output.append(ff_output);
          log_ff_position_setpoint.append(position);
          log_ff_velocity_setpoint.append((5676 / 250));
          m_primaryMotor.setVoltage(-1 * ff_output + pid_output);
        });
  }

  public Command stop() {
    return runOnce(() -> m_primaryMotor.set(0));
  }

  /**
   * Moves the arm to the specified position then stops.
   *
   * @param position The desired position.
   * @return a {@link Command} to get to the desired position.
   */
  private Command moveToPosition(double position) {
    return achievePosition(position)
        .until(
            () ->
                m_pid.atSetpoint()
                    && ((m_encoder.getAbsolutePosition() * 2 * Math.PI) - position) < 0.001);
  }

  /**
   * Holds the arm at the current position setpoint.
   *
   * @return A {@link Command} to hold the position at the setpoint.
   */
  public Command maintain() {
    return achievePosition(m_pid.getSetpoint());
  }

  /**
   * Allows for manual control of the arm using a joystick.
   *
   * @param speed The speed from the joystick input.
   * @return A {@link Command} to control the arm manually.
   */
  public Command manualControl(DoubleSupplier speed) {
    return Commands.run(() -> m_primaryMotor.set(speed.getAsDouble()), this);
  }

  /*
   * Makes the arm go to a setpoint from the {@link Setpoint} enum
   *
   * @param setpoint The setpoint to go to, a {@link Setpoint}
   *
   * @return A {@link Command} to go to the setpoint
   */
  public Command goToSetpoint(Setpoint setpoint) {
    double position = 0;
    log_setpoint.append(setpoint.name());

    switch (setpoint) {
      case kAmp:
        position = 5.34;
        break;

      case kIntake:
        position = 3.7;
        break;

      case kSpeaker:
        position = 3.7;
        break;

      case kStowed:
        position = 3.7;
        break;
    }

    return moveToPosition(position);
  }
}
 