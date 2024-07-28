// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

/** Our Crescendo climber Subsystem */
public class Climber extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.climberPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final PIDController m_pid =
      new PIDController(Constants.climberP, Constants.climberI, Constants.climberD);

  private final DoubleLogEntry log_pid_output =
      new DoubleLogEntry(DataLogManager.getLog(), "/climber/pid/output");

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
          log.motor("climber").voltage(
            m_appliedVoltage.mut_replace(
              m_motor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(
              m_angle.mut_replace(m_encoder.getPosition(), Rotations)
            )
            .angularVelocity(
              m_velocity.mut_replace(m_encoder.getVelocity(), RotationsPerSecond)
            );
        },
        this
      )
    );

  /** Creates a new {@link Climber} {@link edu.wpi.first.wpilibj2.command.Subsystem}. */
  public Climber() {}

  /**
   * Climb the robot!
   *
   * @return A {@link Command} to climb the robot.
   */
  public Command climb() {
    return Commands.run(
        () -> {
          var output =
              m_pid.calculate(Units.rotationsToRadians(m_encoder.getPosition() * -1), -Math.PI);
          log_pid_output.append(output);
          m_motor.setVoltage(output);
        });
  }

  public Command stop() {
    return runOnce(() -> m_motor.set(0));
  }

  public Command maintain() {
    return Commands.run(
        () -> {
          var output =
              m_pid.calculate(Units.rotationsToRadians(m_encoder.getPosition() * -1), m_pid.getSetpoint());
          log_pid_output.append(output);
          m_motor.setVoltage(output);
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
