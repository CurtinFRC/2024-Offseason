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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.intakePort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(Constants.intakeS, Constants.intakeV, Constants.intakeA);

  private final PIDController m_pid =
      new PIDController(Constants.intakeP, Constants.intakeI, Constants.intakeD);

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
              // log -> {
              //   log.motor("intake")
              //       .voltage(
              //           m_appliedVoltage.mut_replace(
              //               m_motor.getBusVoltage(), Volts))
              //       .angularPosition(m_angle.mut_replace(m_encoder.getPosition(), Rotations))
              //       .angularVelocity(
              //           m_velocity.mut_replace(m_encoder.getVelocity(), RotationsPerSecond));
              // },
              null,
              this,
              "intake"));

  public Intake() {}

  public Command achieveSpeed(double speed) {
    return run(
        () ->
            m_motor.setVoltage(
                m_pid.calculate(Units.rotationsToRadians(m_encoder.getVelocity()), speed)));
  }

  public Command spinup(double speed) {
    return achieveSpeed(speed).until(m_pid::atSetpoint);
  }

  public Command stop() {
    return run(() -> m_motor.stopMotor());
  }

  public Command outake(double voltage) {
    return run(() -> m_motor.setVoltage(voltage));
  }

  public Command intake(double voltage) {
    return run(() -> m_motor.setVoltage(voltage));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
