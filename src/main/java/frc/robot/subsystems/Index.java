// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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
  public final Trigger m_intaking = new Trigger(frontBeambreak::get).negate();

  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Index");
  private final StringPublisher m_activeCommand =
      driveStats.getStringTopic("Active Command").publish();
  private final BooleanPublisher m_activeCommandFinished =
      driveStats.getBooleanTopic("Active Command Finished").publish();

  private final NetworkTable indexStats = NetworkTableInstance.getDefault().getTable("Index");
  private final BooleanPublisher m_ntHasNote = indexStats.getBooleanTopic("Has Note").publish();
  private final BooleanPublisher m_ntIntaking = indexStats.getBooleanTopic("Intaking").publish();

  public Index() {}

  public Command shoot() {
    // if (!m_hasNote.getAsBoolean()) {
    //   return runOnce(() -> {}).withName("Empty Index");
    // } else {
      return run(() -> m_motor.setVoltage(-8))
          // .until(m_hasNote.negate())
          .withName("Index PassThrough");
    // }
  }

  public Command shootAuto() {
    // if (!m_hasNote.getAsBoolean()) {
    //   return runOnce(() -> {}).withName("Empty Index");
    // } else {
      return run(() -> m_motor.setVoltage(-8))
          .until(m_hasNote.negate())
          .withName("Index PassThrough");
    // }
  }

  public Command intake(double voltage) {
    // return run(() -> m_motor.setVoltage(voltage));
    return run(() -> m_motor.setVoltage(voltage)).until(m_hasNote);
  }

  public Command outake(double voltage) {
    return run(() -> m_motor.setVoltage(voltage));
  }

  public Command stop() {
    return runOnce(() -> m_motor.stopMotor());
  }

  @Override
  public void periodic() {
    m_ntHasNote.set(m_hasNote.getAsBoolean());
    m_ntIntaking.set(m_intaking.getAsBoolean());

    var currentcommand = getCurrentCommand();
    if (currentcommand != null) {
      m_activeCommand.set(currentcommand.toString());
      m_activeCommandFinished.set(currentcommand.isFinished());
    }
  }
}
