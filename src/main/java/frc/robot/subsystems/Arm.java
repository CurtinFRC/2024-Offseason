// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private PIDController m_pid;
  private final CANSparkMax m_motor = new CANSparkMax(Constants.armLeadPort, MotorType.kBrushless);
  private DataLog m_log = DataLogManager.getLog();
  private DoubleLogEntry log_trapezoid_goal = new DoubleLogEntry(m_log, "/arm/trapezoid/goal");
  private DoubleLogEntry log_pid_output = new DoubleLogEntry(m_log, "/arm/pid/output");
  private DoubleLogEntry log_pid_setpoint = new DoubleLogEntry(m_log, "/arm/pid/setpoint");
  private DoubleLogEntry log_ff_position_setpoint =
      new DoubleLogEntry(m_log, "/arm/ff/position_setpoint");
  private DoubleLogEntry log_ff_velocity_setpoint =
      new DoubleLogEntry(m_log, "/arm/ff/velocity_setpoint");
  private DoubleLogEntry log_ff_output = new DoubleLogEntry(m_log, "/arm/ff/output");
  private StringLogEntry log_setpoint = new StringLogEntry(m_log, "/arm/setpoint");
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(Constants.armEncoderPort);
  private ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.armS, Constants.armG,
          Constants.armV, Constants.armA);

  public enum Setpoint {
    kAmp,
    kIntake,
    kSpeaker,
    kStowed
  }

  private void useOutput(double output, Double position, Double velocity) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(position, velocity);

    log_trapezoid_goal.append(m_goal.position);
    log_trapezoid_goal.append(m_goal.velocity);
    log_pid_output.append(output);
    log_pid_setpoint.append(m_pid.getSetpoint());
    log_ff_output.append(feedforward);
    log_ff_position_setpoint.append(position);
    log_ff_velocity_setpoint.append(velocity);

    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired goal state.
   */
  private void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired goal position.
   */
  private void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0);
  }

  /**
   * Gets the goal for the ProfiledPIDController.
   *
   * @return The goal.
   */
  private TrapezoidProfile.State getGoal() {
    return m_goal;
  }

  private double getPeriod() {
    return m_pid.getPeriod();
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @return The controller's next output.
   */
  private double calculate(double measurement) {

    m_setpoint = m_profile.calculate(getPeriod(), m_setpoint, m_goal);
    return m_pid.calculate(measurement, m_setpoint.position);
  }

  private double calculate(double measurement, TrapezoidProfile.State goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  private double calculate(double measurement, double goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  public Arm() {

    m_pid = new PIDController(Constants.armP, Constants.armI, Constants.armD);
    new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.armMaxVelRadPerSec, Constants.armMaxAccelRadPerSec));

    // Start arm at rest in neutral position
    setGoal(Constants.armOffsetRad);
  }

  public double getMeasurement() {
    return m_encoder.getDistance() + Constants.armOffsetRad;
  }

  public void goToSetpoint(Setpoint setpoint) {
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
  }
}
;
