// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.indexerPort, MotorType.kBrushless);
  private final DigitalInput frontBeambreak = new DigitalInput(Constants.intakeFrontBeambreak);
  private final DigitalInput backBeamBreak = new DigitalInput(Constants.intakeBackBeambreak);

  public final Trigger m_hasNote = new Trigger(backBeamBreak::get);
  public final Trigger m_intaking = new Trigger(frontBeambreak::get);

  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Index");
  private final BooleanPublisher m_ntHasNote = driveStats.getBooleanTopic("Has Note").publish();
  private final BooleanPublisher m_ntIntaking = driveStats.getBooleanTopic("Intaking").publish();

  public Index() {}

  public Command shoot() {
    return run(() -> m_motor.setVoltage(-8)).until(m_hasNote.negate());
  }

  public Command intake(double voltage) {
    return run(() -> m_motor.setVoltage(voltage)).until(m_hasNote);
  }

  public Command stop() {
    return runOnce(() -> m_motor.stopMotor());
  }

  @Override
  public void periodic() {
    m_ntHasNote.set(m_hasNote.getAsBoolean());
    m_ntIntaking.set(m_intaking.getAsBoolean());
  }
}
