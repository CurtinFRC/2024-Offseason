// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.intakePort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final DigitalInput m_beamBreakSensor = new DigitalInput(Constants.intakebeambreak);

  private final PIDController m_pid = new PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD);

  public final Trigger m_isbeambreakInterrupted = new Trigger(m_beamBreakSensor::get).negate();;

  public Intake() {}

  public Command achieveSpeed(double speed) {
    return Commands.run(
        () ->
            m_motor.setVoltage(
                m_pid.calculate(Units.rotationsToRadians(m_encoder.getVelocity()), speed)));
  }

  public Command spinup(double speed) {
    return achieveSpeed(speed).until(m_pid::atSetpoint);
  }

  public Command spinUntilBeamBreak(double speed) {
    return achieveSpeed(speed).until(m_isbeambreakInterrupted).andThen(stopIntake());
  }

  public boolean isBeamBroken() {
    return !m_beamBreakSensor.get();
  }

  public Command stopIntake() {
    return new RunCommand(() -> m_motor.stopMotor(), this);
  }

  public Command outake() {
    return spinup(-300);
  }
}
