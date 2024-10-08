// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.intakePort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Intake");
  private final StringPublisher m_activeCommand =
      driveStats.getStringTopic("Active Command").publish();
  private final BooleanPublisher m_activeCommandFinished =
      driveStats.getBooleanTopic("Active Command Finished").publish();

  private final PIDController m_pid =
      new PIDController(Constants.intakeP, Constants.intakeI, Constants.intakeD);

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

  @Override
  public void periodic() {
    var currentcommand = getCurrentCommand();
    if (currentcommand != null) {
      m_activeCommand.set(currentcommand.toString());
      m_activeCommandFinished.set(currentcommand.isFinished());
    }
  }
}
