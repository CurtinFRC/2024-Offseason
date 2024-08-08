// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final CANSparkMax m_primaryMotor =
      new CANSparkMax(Constants.armLeadPort, MotorType.kBrushless);
  private final CANSparkMax m_followerMotor =
      new CANSparkMax(Constants.armFollowerPort, MotorType.kBrushless);
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(Constants.armEncoderPort);

  private final PIDController m_pid =
      new PIDController(Constants.armP, Constants.armI, Constants.armD);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(Constants.armS, Constants.armG, Constants.armV, Constants.armA);

  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry log_pid_output = new DoubleLogEntry(m_log, "/arm/pid/output");
  private final DoubleLogEntry log_pid_setpoint = new DoubleLogEntry(m_log, "/arm/pid/setpoint");
  private final DoubleLogEntry log_ff_position_setpoint =
      new DoubleLogEntry(m_log, "/arm/ff/position_setpoint");
  private final DoubleLogEntry log_ff_velocity_setpoint =
      new DoubleLogEntry(m_log, "/arm/ff/velocity_setpoint");
  private final DoubleLogEntry log_ff_output = new DoubleLogEntry(m_log, "/arm/ff/output");
  private final StringLogEntry log_setpoint = new StringLogEntry(m_log, "/arm/setpoint");

  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Arm");
  private final DoublePublisher output = driveStats.getDoubleTopic("Output").publish();
  private final DoublePublisher ff_output = driveStats.getDoubleTopic("FF Output").publish();
  private final DoublePublisher pid_output = driveStats.getDoubleTopic("PID Output").publish();
  private final DoublePublisher pid_error = driveStats.getDoubleTopic("PID Error").publish();
  private final DoublePublisher pid_setpoint = driveStats.getDoubleTopic("PID Setpoint").publish();
  private final DoublePublisher angle = driveStats.getDoubleTopic("Angle").publish();
  public final Trigger m_atSetpoint = new Trigger(m_pid::atSetpoint);

  /** Creates a new {@link Arm} {@link edu.wpi.first.wpilibj2.command.Subsystem}. */
  public Arm() {
    m_followerMotor.follow(m_primaryMotor);
    m_encoder.reset();
    m_encoder.setPositionOffset(0.1);
  }

  /** Achieves and maintains speed for the primary motor. */
  private Command achievePosition(double position) {
    return run(
        () -> {
          var pid_output = m_pid.calculate(m_encoder.getAbsolutePosition() * 2 * Math.PI, position);
          log_pid_output.append(pid_output);
          log_pid_setpoint.append(m_pid.getSetpoint());
          var ff_output = m_feedforward.calculate(position, (5676 / 250));
          log_ff_output.append(ff_output);
          log_ff_position_setpoint.append(position);
          log_ff_velocity_setpoint.append((5676 / 250));
          output.set(ff_output + pid_output);
          this.ff_output.set(ff_output);
          this.pid_output.set(0.5 + pid_output);
          this.pid_error.set(m_pid.getPositionError());
          this.pid_setpoint.set(m_pid.getSetpoint());
          m_primaryMotor.setVoltage(0.5 + pid_output * -1);
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
    return achievePosition(position).until(m_pid::atSetpoint);
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
    return run(
        () -> {
          output.set(speed.getAsDouble());
          m_primaryMotor.set(speed.getAsDouble());
        });
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
      case kAmp -> position = Math.PI / 2 + 0.1;
      case kIntake -> position = 0;
      case kSpeaker -> position = 0.2;
      case kStowed -> position = 0;
    }

    return moveToPosition(position);
  }

  @Override
  public void periodic() {
    angle.set(m_encoder.getAbsolutePosition() * 2 * Math.PI);
  }
}
