// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

/** Our Crescendo shooter Subsystem */
public class Shooter extends SubsystemBase {
  private final PIDController m_pid =
      new PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD);
  private final CANSparkMax m_motor = new CANSparkMax(Constants.shooterPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry log_pid_output = new DoubleLogEntry(m_log, "/shooter/pid/output");

  public final Trigger m_atSetpoint = new Trigger(m_pid::atSetpoint);
  ;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                m_motor.setVoltage(volts.in(Volts));
              },
              log -> {
                log.motor("climber")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(m_encoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(m_encoder.getVelocity(), RotationsPerSecond));
              },
              this));

  /** Creates a new {@link Shooter} {@link edu.wpi.first.wpilibj2.command.Subsystem}. */
  public Shooter() {}

  /** Acheives and maintains speed. */
  private Command achieveSpeeds(double speed) {
    m_pid.reset();
    m_pid.setSetpoint(speed);
    return Commands.run(
        () -> {
          var output =
              m_pid.calculate(
                  -1 * Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity()));

          NetworkTableInstance.getDefault()
              .getTable("shooter")
              .getEntry("error")
              .setNumber(m_pid.getVelocityError());
          NetworkTableInstance.getDefault()
              .getTable("shooter")
              .getEntry("setpoint")
              .setNumber(m_pid.getSetpoint());
          NetworkTableInstance.getDefault()
              .getTable("shooter")
              .getEntry("current_speed")
              .setNumber(Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity()));
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
    return achieveSpeeds(speed).until(m_pid::atSetpoint);
  }

  public Command stop() {
    return runOnce(() -> m_motor.set(0));
  }

  /** Holds the shooter at the current speed setpoint. */
  public Command maintain() {
    return achieveSpeeds(m_pid.getSetpoint());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
